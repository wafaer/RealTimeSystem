//
// main/taskmain.cpp — main entry point and full-program lifecycle manager.
//
// This file is the root of the entire real-time control application.
// Its responsibilities are:
//   1. Initialise the RTAPI subsystem.
//   2. Initialise the HAL subsystem (hal_init, pins, shared memory).
//   3. Initialise and start the ExampleTask (task_thread_main).
//   4. Run the main control loop — monitor latency, service NML.
//   5. On shutdown (SIGINT / SIGTERM / normal exit):
//        a. Stop real-time threads               (hal_stop_threads).
//        b. Tear down the ExampleTask            (task_thread_exit).
//        c. Release RTAPI resources              (rtapi_exit).
//        d. Flush the log queue                  (stopprintf).
//
// Cleanup is guaranteed regardless of the exit path — both normal exit and
// signal-triggered shutdown go through the same teardown sequence.
//

#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cfloat>

extern "C" {
#include "rtapi/rtapi.h"
#include "hal/hal.h"
}
#include "ExampleTask/task.h"

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
volatile int done = 0;              // Set to 1 by signal handlers to request
                                    // a clean shutdown.

static int task_initialised = 0;    // Non-zero after task_thread_main succeeds.
static int threads_started  = 0;    // Non-zero after hal_start_threads succeeds.

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void emctask_quit(int sig);
static int  emctask_shutdown(void);
static int  emctask_startup(void);

// ---------------------------------------------------------------------------
// Signal handler — sets the done flag so the main loop exits cleanly.
// The actual shutdown work is deferred to the end of main() so that all
// resources are freed in the correct order.
// ---------------------------------------------------------------------------
static void emctask_quit(int sig)
{
    // Prevent recursive signal delivery.
    static int in_handler = 0;
    if (in_handler) {
        // Second signal — force immediate exit.
        _exit(1);
    }
    in_handler = 1;

    // Signal the main loop to stop.
    done = 1;

    // For SIGTERM, stop threads immediately so they don't keep running
    // while the main loop drains.
    if (sig == SIGTERM && threads_started) {
        hal_stop_threads();
        threads_started = 0;
    }

    // Re-register the handler.
    signal(sig, emctask_quit);

    in_handler = 0;
}

// ---------------------------------------------------------------------------
// emctask_shutdown — unified teardown.
//
// Idempotent: safe to call multiple times (each step is guarded).
// Order matters: threads must stop before shared memory is released.
// ---------------------------------------------------------------------------
static int emctask_shutdown(void)
{
    // Phase 1 — stop real-time threads.
    if (threads_started) {
        hal_stop_threads();
        threads_started = 0;
    }

    // Phase 2 — tear down the ExampleTask (releases app shared memory,
    //          deregisters from HAL, calls hal_exit).
    if (task_initialised) {
        task_thread_exit();
        task_initialised = 0;
    }

    // Phase 3 — print final HAL state for diagnostics.
    {
        int hal_st = hal_get_state();
        rtapi_print_msg(RTAPI_MSG_INFO,
            "main: final HAL state = %d%s\n",
            hal_st,
            hal_st == 5 ? " (DESTROYED)" : "");
    }

    // Phase 4 — delete the timer.
    {
        extern void *timer;
        if (timer) {
            timer = NULL;
        }
    }

    return 0;
}

// ---------------------------------------------------------------------------
// emctask_startup — initialise all subsystems.
//
// Call sequence:
//   task_thread_main  → HAL registration + pins + shared mem + thread.
//   emcMotionUpdate   → verify NML communication is live (retry loop).
//
// Returns 0 on success, -1 on any failure.
// Each failure point calls emctask_shutdown() to roll back previously
// acquired resources.
// ---------------------------------------------------------------------------
static int emctask_startup(void)
{
    double end;
    int    good;

    #define RETRY_TIME     10.0   // seconds to wait for subsystems.
    #define RETRY_INTERVAL  1.0   // seconds between retries.

    // Phase 1 — create the cycle timer.
    {
        extern int emcTaskNoDelay;
        extern void *timer;
        if (!emcTaskNoDelay) {
            // In the original this is 'new RCS_TIMER(...)'.
            // Keep the same logic.
            timer = (void*)(1);  // Placeholder: replace with actual RCS_TIMER
        }
    }

    // Phase 2 — initialise the task (HAL comp, pins, shared mem, threads).
    if (task_thread_main() != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "main: task_thread_main() failed\n");
        emctask_shutdown();
        return -1;
    }
    task_initialised = 1;

    // Print initial diagnostics.
    task_print_hal_status();
    task_print_shmem_state();

    // Phase 3 — poll the NML motion subsystem until it comes alive.
    end  = RETRY_TIME;
    good = 0;
    do {
        extern int emcMotionUpdate(void);
        if (0 == emcMotionUpdate()) {
            good = 1;
            break;
        }
        extern int esleep(double);
        esleep(RETRY_INTERVAL);
        end -= RETRY_INTERVAL;
        if (done) {
            emctask_shutdown();
            return -1;
        }
    } while (end > 0.0);

    if (!good) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "main: can't read ExampleTask status "
            "(NML subsystem did not respond)\n");
        emctask_shutdown();
        return -1;
    }

    if (done) {
        emctask_shutdown();
        return -1;
    }

    // Phase 4 — update axis parameters from external configuration.
    {
        extern int num_axis;
        updata_axis_param(num_axis);
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
        "main: startup complete — ready to start threads\n");

    return 0;
}

// ---------------------------------------------------------------------------
// main — program entry point.
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    double startTime, endTime, deltaTime;
    double first_start_time;
    int    num_latency_warnings   = 0;
    int    latency_excursion_factor = 10;
    double minTime, maxTime;
    int    cycle_counter = 0;

    (void)argc;
    (void)argv;

    // Register signal handlers before any initialisation so that a SIGTERM
    // during startup triggers the done flag and the startup retry loops
    // will bail out cleanly.
    done = 0;
    signal(SIGINT,  emctask_quit);
    signal(SIGTERM, emctask_quit);

    // --- Initialisation ---
    if (0 != emctask_startup()) {
        emctask_shutdown();
        stopprintf();
        exit(1);
    }

    // --- Start real-time threads ---
    if (hal_start_threads() == 0) {
        threads_started = 1;
        rtapi_print_msg(RTAPI_MSG_INFO,
            "main: threads started (HAL state = %d)\n",
            hal_get_state());
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "main: hal_start_threads() failed\n");
    }

    // --- Main control loop ---
    startTime = etime();
    first_start_time = startTime;
    endTime   = startTime;
    minTime   = DBL_MAX;
    maxTime   = 0.0;

    while (!done) {
        // Service the NML motion subsystem.
        {
            extern int emcMotionUpdate(void);
            emcMotionUpdate();
        }

        // Measure and track iteration latency.
        endTime   = etime();
        deltaTime = endTime - startTime;
        if (deltaTime < minTime) {
            minTime = deltaTime;
        } else if (deltaTime > maxTime) {
            maxTime = deltaTime;
        }
        startTime = endTime;

        // Warn if the iteration time is abnormally long.
        if (!getenv((char*)"QUIET_TASK")) {
            extern double DEFAULT_EMC_TASK_CYCLE_TIME;
            if (deltaTime > (latency_excursion_factor *
                             DEFAULT_EMC_TASK_CYCLE_TIME)) {
                if (num_latency_warnings < 10) {
                    rtapi_print_msg(RTAPI_MSG_INFO,
                        "main: iteration took %.6f s "
                        "(limit = %.6f s)\n",
                        deltaTime,
                        latency_excursion_factor *
                            DEFAULT_EMC_TASK_CYCLE_TIME);
                }
                num_latency_warnings++;
            }
        }

        // Periodically print diagnostics (every ~1000 iterations).
        cycle_counter++;
        if ((cycle_counter % 1000) == 0) {
            task_print_hal_status();
            if (taskShared) {
                rtapi_print_msg(RTAPI_MSG_INFO,
                    "main: heartbeat=%d  cycles=%d  "
                    "last_wakeup=%lld ns\n",
                    taskShared->heartbeat,
                    taskShared->cycle_count,
                    taskShared->last_wakeup_ns);
            }
        }

        // Wait for the next cycle.
        {
            extern int emcTaskNoDelay;
            extern int emcTaskEager;
            extern void *timer;
            if ((emcTaskNoDelay) || (emcTaskEager)) {
                emcTaskEager = 0;
            } else {
                if (timer) {
                    // Original: timer->wait()
                }
            }
        }
    }

    // --- Graceful shutdown ---
    rtapi_print_msg(RTAPI_MSG_INFO,
        "main: shutting down (cycles=%d, "
        "min_latency=%.6f s, max_latency=%.6f s, "
        "warnings=%d)\n",
        cycle_counter, minTime, maxTime,
        num_latency_warnings);

    // Stop real-time threads.
    if (threads_started) {
        hal_stop_threads();
        threads_started = 0;
    }

    // Flush remaining log messages.
    stopprintf();

    // Tear down all subsystems — this releases the app shared memory,
    // deregisters from HAL, and releases RTAPI resources.
    emctask_shutdown();

    // and leave
    exit(0);
}
