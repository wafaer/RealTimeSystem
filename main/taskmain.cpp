#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cfloat>
#include <sys/time.h>
#include "../time/timer.h"

extern "C" {
#include "rtapi/rtapi.h"
#include "hal/hal.h"
}
#include "ExampleTask/task.h"

#define DEFAULT_EMC_TASK_CYCLE_TIME 0.100

volatile int done = 0;

static RCS_TIMER *timer = 0;
static int task_initialised = 0;
static int threads_started  = 0;

static int emcTaskNoDelay = 0;
static int emcTaskEager = 0;

static void emctask_quit(int sig);
static int  emctask_shutdown(void);
static int  emctask_startup(void);

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
        if (timer) {
            delete timer;
            timer = 0;
        }
    }

    return 0;
}

static int emctask_startup(void)
{
    double end;
    int    good;

    #define RETRY_TIME     10.0   // seconds to wait for subsystems.
    #define RETRY_INTERVAL  1.0   // seconds between retries.

    if (printfMaster() !=0)
    {
        return -1;
    }

    //初始化hal
    if(hal_app_main() != 0)
    {
        return -1;
    }

    // get the timer
    if (!emcTaskNoDelay)
    {
        timer = new RCS_TIMER(0.1, "", "");
    }

    // Phase 2 — initialise the task (HAL comp, pins, shared mem, threads).
    if (task_thread_main() != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "main: task_thread_main() failed\n");
        emctask_shutdown();
        return -1;
    }

    task_initialised = 1;

    rtapi_print_msg(RTAPI_MSG_INFO,
        "main: startup complete — ready to start threads\n");

    return 0;
}

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

        cycle_counter++;
        if ((cycle_counter % 1000) == 0) {
            if (taskShared) {
                rtapi_print_msg(RTAPI_MSG_INFO,
                    "main: heartbeat=%d  cycles=%d  "
                    "last_wakeup=%lld ns\n",
                    taskShared->heartbeat,
                    taskShared->cycle_count,
                    taskShared->last_wakeup_ns);
            }
        }

        {
            if ((emcTaskNoDelay) || (emcTaskEager)) {
                emcTaskEager = 0;
            } else {
                if (timer) {
                    timer->wait();
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

    stopprintf();

    emctask_shutdown();

    exit(0);
}
