#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cfloat>
#include <cerrno>
#include <unistd.h>
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

// 常规退出信号
// 拦截SIGINT(CTRL+C),SIGTERM,SIGHUP
static void emctask_quit(int sig)
{
    (void)sig;

    done = 1;
}

// 拦截段错误（SIFSEVG）
static void emctask_fatal(int sig)
{
    if (emc_shmem_id > 0) {
        rtapi_shmem_delete(emc_shmem_id, mot_comp_id);
    }

    signal(sig, SIG_DFL);
    raise(sig);
}

//
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

static int shutdown_done = 0;

static void emctask_atexit(void)
{
    if (shutdown_done) return;
    shutdown_done = 1;
    emctask_shutdown();
    stopprintf();
}

int main(int argc, char *argv[])
{
    double startTime, endTime, deltaTime;
    double first_start_time;
    int    num_latency_warnings   = 0;
    int    latency_excursion_factor = 10;
    double minTime, maxTime;
    int    cycle_counter = 0;
    int    motion_was_active = 0;

    (void)argc;
    (void)argv;

    done = 0;

    // Single point of cleanup — runs on normal return/exit() paths and after
    // emctask_quit flags the main loop to exit.  Idempotent via shutdown_done.
    atexit(emctask_atexit);

    // Graceful-stop signals: only flag done; full cleanup runs in atexit.
    {
        struct sigaction sa = {};
        sa.sa_handler = emctask_quit;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;            // no SA_RESTART, so the main loop's
                                    // timer->wait() returns promptly on EINTR.
        sigaction(SIGINT,  &sa, nullptr);
        sigaction(SIGTERM, &sa, nullptr);
        sigaction(SIGHUP,  &sa, nullptr);
    }

    // Fatal signals: best-effort detach/RMID of task shmem, then re-raise so
    // the default handler produces a core dump.
    {
        struct sigaction fa = {};
        fa.sa_handler = emctask_fatal;
        sigemptyset(&fa.sa_mask);
        fa.sa_flags = SA_RESETHAND; // run once, then revert to default.
        sigaction(SIGSEGV, &fa, nullptr);
        sigaction(SIGABRT, &fa, nullptr);
        sigaction(SIGBUS,  &fa, nullptr);
        sigaction(SIGFPE,  &fa, nullptr);
    }

    // --- Initialisation ---
    if (0 != emctask_startup()) {
        // emctask_atexit will run on exit() and complete cleanup.
        exit(1);
    }

    // --- Start real-time threads ---
    {
        int r = hal_start_threads();
        if (r == 0 || r == -EALREADY) {
            threads_started = 1;
            rtapi_print_msg(RTAPI_MSG_INFO,
                "main: threads started (HAL state = %d)\n",
                hal_get_state());
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "main: hal_start_threads() failed (r=%d)\n", r);
        }
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

        // Auto-exit when the startup trajectory finishes. We only trigger
        // *after* we have observed an active phase, so the very first cycles
        // (before the command handler runs) don't terminate the program.
        if (taskShared) {
            if (taskShared->tp.active) {
                motion_was_active = 1;
            } else if (motion_was_active) {
                rtapi_print_msg(RTAPI_MSG_INFO,
                    "main: motion complete (curr_pos=%.3f, "
                    "target=%.3f) — exiting\n",
                    taskShared->tp.curr_pos,
                    taskShared->tp.pos_cmd);
                done = 1;
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

    // Cleanup (shmem release + hal_exit + timer delete) is handled by
    // emctask_atexit registered via atexit().
    exit(0);
}
