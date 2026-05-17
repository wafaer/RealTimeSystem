/// \file taskmain.cpp
///
/// 实时系统主程序入口。
///
/// 负责初始化 HAL 组件、启动实时线程、主控制循环及安全退出。

#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cfloat>
#include <cerrno>
#include <unistd.h>
#include <sys/time.h>
#include "time/timer.h"

extern "C" {
#include "rtapi/rtapi.h"
#include "hal/hal.h"
}
#include "ExampleTask/task.h"

#define DEFAULT_EMC_TASK_CYCLE_TIME 0.100

volatile int done = 0;

static RCS_TIMER *timer = 0;
static int task_initialised = 0;
static int threads_started = 0;

static int emcTaskNoDelay = 0;
static int emcTaskEager = 0;

static void emctask_quit(int sig);
static int emctask_shutdown(void);
static int emctask_startup(void);

/// \brief 常规退出信号处理器。
///
/// 仅设置 done 标志，实际资源清理由 atexit(emctask_atexit) 完成。
///
/// \param sig 捕获的信号编号（未使用）。
static void emctask_quit(int sig) {
    (void)sig;
    done = 1;
}

/// \brief 致命信号处理器。
///
/// 尝试释放共享内存后重新触发信号，由系统默认处理器产生 core dump。
///
/// \param sig 捕获的信号编号（SIGSEGV/SIGABRT/SIGBUS/SIGFPE）。
static void emctask_fatal(int sig) {
    if (emc_shmem_id > 0) {
        rtapi_shmem_delete(emc_shmem_id, mot_comp_id);
    }
    signal(sig, SIG_DFL);
    raise(sig);
}

/// \brief 阶段性关闭系统（4 步）。
///
/// 1. 停止实时线程；2. 销毁 ExampleTask（释放共享内存、注销 HAL）；
/// 3. 打印最终 HAL 状态；4. 删除定时器。
///
/// \return 始终返回 0。
static int emctask_shutdown(void) {
    if (threads_started) {
        hal_stop_threads();
        threads_started = 0;
    }

    if (task_initialised) {
        task_thread_exit();
        task_initialised = 0;
    }

    {
        int hal_st = hal_get_state();
        rtapi_print_msg(RTAPI_MSG_INFO,
                        "main: final HAL state = %d%s\n",
                        hal_st,
                        hal_st == 5 ? " (DESTROYED)" : "");
    }

    {
        if (timer) {
            delete timer;
            timer = 0;
        }
    }

    return 0;
}

/// \brief 初始化主任务系统。
///
/// 依次完成：printf 主端注册、HAL 应用初始化、
/// 周期定时器创建、任务线程初始化。
///
/// \return 0 成功；-1 失败。
static int emctask_startup(void) {
    if (printfMaster() != 0) {
        return -1;
    }

    if (hal_app_main() != 0) {
        return -1;
    }

    if (!emcTaskNoDelay) {
        timer = new RCS_TIMER(0.1, "", "");
    }

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

/// \brief atexit 注册的最终清理函数。
///
/// 通过 shutdown_done 确保幂等性，调用 emctask_shutdown 并停止 printf。
static void emctask_atexit(void) {
    if (shutdown_done) return;
    shutdown_done = 1;
    emctask_shutdown();
    stopprintf();
}

/// \brief 主程序入口。
///
/// 1. 注册信号处理器（常规退出 + 致命异常）；
/// 2. 调用 emctask_startup 完成初始化；
/// 3. 启动实时线程；
/// 4. 进入主控制循环，周期性等待、同步轨迹运行；
/// 5. 轨迹完成后自动退出或接收信号后安全关闭。
///
/// \param argc 命令行参数个数（未使用）。
/// \param argv 命令行参数数组（未使用）。
/// \return 程序退出码（0 正常；1 初始化失败）。
int main(int argc, char *argv[]) {
    double startTime, endTime, deltaTime;
    double first_start_time;
    int num_latency_warnings = 0;
    int latency_excursion_factor = 10;
    double minTime, maxTime;
    int cycle_counter = 0;
    int motion_was_active = 0;

    (void)argc;
    (void)argv;

    done = 0;
    atexit(emctask_atexit);

    {
        struct sigaction sa = {};
        sa.sa_handler = emctask_quit;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sigaction(SIGINT, &sa, nullptr);
        sigaction(SIGTERM, &sa, nullptr);
        sigaction(SIGHUP, &sa, nullptr);
    }

    {
        struct sigaction fa = {};
        fa.sa_handler = emctask_fatal;
        sigemptyset(&fa.sa_mask);
        fa.sa_flags = SA_RESETHAND;
        sigaction(SIGSEGV, &fa, nullptr);
        sigaction(SIGABRT, &fa, nullptr);
        sigaction(SIGBUS, &fa, nullptr);
        sigaction(SIGFPE, &fa, nullptr);
    }

    if (0 != emctask_startup()) {
        exit(1);
    }

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

    startTime = etime();
    first_start_time = startTime;
    endTime = startTime;
    minTime = DBL_MAX;
    maxTime = 0.0;

    while (!done) {
        endTime = etime();
        deltaTime = endTime - startTime;
        if (deltaTime < minTime) {
            minTime = deltaTime;
        } else if (deltaTime > maxTime) {
            maxTime = deltaTime;
        }
        startTime = endTime;

        if (!getenv((char *)"QUIET_TASK")) {
            if (deltaTime >
                (latency_excursion_factor * DEFAULT_EMC_TASK_CYCLE_TIME)) {
                if (num_latency_warnings < 10) {
                    rtapi_print_msg(
                        RTAPI_MSG_INFO,
                        "main: iteration took %.6f s (limit = %.6f s)\n",
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

    rtapi_print_msg(RTAPI_MSG_INFO,
                    "main: shutting down (cycles=%d, "
                    "min_latency=%.6f s, max_latency=%.6f s, "
                    "warnings=%d)\n",
                    cycle_counter, minTime, maxTime, num_latency_warnings);

    if (threads_started) {
        hal_stop_threads();
        threads_started = 0;
    }

    exit(0);
}
