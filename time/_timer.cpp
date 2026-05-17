/// \file _timer.cpp
///
/// 实时系统计时接口实现。

#include "_timer.h"
#include "rtapi/rtapi.h"
#include <errno.h>
#include <sys/time.h>
#include <sched.h>
#include <unistd.h>

/// \brief 获取系统时钟滴答周期（秒）。
///
/// \return 每秒时钟滴答数的倒数，即单个滴答的时长（秒）。
double clk_tck() {
    return 1.0 / (double) sysconf(_SC_CLK_TCK);
}

/// \brief 获取自任意基准时刻起的秒数。
///
/// 调用 gettimeofday，精度为微秒级。
///
/// \return 自基准时刻起的秒数；获取失败时返回 0.0。
double etime() {
    struct timeval tp;
    double retval;

    if (0 != gettimeofday(&tp, NULL)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "etime: can't get time\n");
        return 0.0;
    }

    retval = ((double)tp.tv_sec) + ((double)tp.tv_usec) / 1000000.0;
    return retval;
}

/// 若为真且剩余睡眠时间小于一个时钟滴答，则调用 sched_yield() 让出 CPU
///（适用于高优先级实时任务）。全局可通过外部赋值调整行为。
int esleep_use_yield = 0;

/// \brief 进程级休眠。
///
/// 基于 select() 实现的精度休眠，粒度受系统时钟滴答限制。
/// 若 esleep_use_yield 为真且剩余时间小于一个时钟滴答，
/// 则调用 sched_yield() 让出 CPU 而非继续睡眠。
///
/// \param secs 期望休眠时长（秒），小于等于 0 时直接返回。
void esleep(double secs) {
    struct timeval tval;
    static double clk_tck_val = 0;
    double total = secs;
    double started = etime();
    double left = total;

    if (secs <= 0.0) return;

    if (clk_tck_val <= 0) {
        clk_tck_val = clk_tck();
    }

    do {
        if (left < clk_tck_val && esleep_use_yield) {
            sched_yield();
        } else {
            tval.tv_sec = (long)left;
            tval.tv_usec = (long)((left - (double)tval.tv_sec) * 1000000.0);
            if (tval.tv_sec == 0 && tval.tv_usec == 0) {
                tval.tv_usec = 1;
            }
            if (select(0, NULL, NULL, NULL, &tval) < 0) {
                if (errno != EINTR) {
                    break;
                }
            }
        }
        left = total - etime() + started;
    } while (left > 0 && (left > clk_tck_val && esleep_use_yield));
}
