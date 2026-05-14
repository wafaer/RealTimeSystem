#ifndef REALTIMESYSTEM__TIMER_H
#define REALTIMESYSTEM__TIMER_H

//
// Created by Administrator on 2025/8/16.
//

#ifndef _TIMER_H
#define _TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

    /* sleeps # of seconds, to clock tick resolution */
    extern void esleep(double secs);
    extern double etime();
    extern double clk_tck();

#ifdef __cplusplus
}
#endif

#endif //_TIMER_H


#endif //REALTIMESYSTEM__TIMER_H