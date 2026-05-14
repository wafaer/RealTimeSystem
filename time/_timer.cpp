//
// Created by Administrator on 2025/8/16.
//

#include "_timer.h"
#include "rtapi/rtapi.h"
#include <errno.h>
#include <sys/time.h>
#include <sched.h>
#include <unistd.h>

double clk_tck()
{
    return 1.0 / (double) sysconf(_SC_CLK_TCK);
}

double etime()
{
    struct timeval tp;
    double retval;

    if (0 != gettimeofday(&tp, NULL)) {
        rtapi_print_msg(RTAPI_MSG_ERR,"etime: can't get time\n");
        return 0.0;
    }

    retval = ((double) tp.tv_sec) + ((double) tp.tv_usec) / 1000000.0;
    return retval;
}

int esleep_use_yield = 0;

void esleep(double seconds_to_sleep)
{
    struct timeval tval;
    static double clk_tck_val = 0;
    double total = seconds_to_sleep;	/* total sleep asked for */
    double started = etime();	/* time when called */
    double left = total;
    if (seconds_to_sleep <= 0.0)
        return;
    if (clk_tck_val <= 0) {
        clk_tck_val = clk_tck();
    }
    do {
        if (left < clk_tck_val && esleep_use_yield) {
            sched_yield();
        } else {
            tval.tv_sec = (long) left;	/* double->long truncates, ANSI */
            tval.tv_usec = (long) ((left - (double) tval.tv_sec) * 1000000.0);
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
    }
    while (left > 0 && (left > clk_tck_val && esleep_use_yield));
    return;
}
