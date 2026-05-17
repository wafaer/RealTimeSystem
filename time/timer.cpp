/// \file timer.cpp
///
/// RCS_TIMER 周期定时器类实现。

#include "timer.h"
#include "_timer.h"

RCS_TIMER::RCS_TIMER(const char * /*process_name*/,
                     const char * /*config_file*/) {
    zero_timer();
    set_timeout(0);
}

RCS_TIMER::RCS_TIMER(double _timeout, const char * /*process_name*/,
                     const char * /*config_file*/) {
    zero_timer();
    set_timeout(_timeout);
}

RCS_TIMER::RCS_TIMER(double _timeout, RCS_TIMERFUNC _function, void *_arg) {
    zero_timer();
    counts_per_real_sleep = 0;
    counts_since_real_sleep = 0;

    if (_timeout < clk_tck_val) {
        counts_per_real_sleep = (int)(clk_tck_val / _timeout);
        timeout = clk_tck_val;
    } else {
        timeout = _timeout;
    }
    function = _function;
    arg = _arg;
    time_since_real_sleep = start_time;
}

RCS_TIMER::~RCS_TIMER() {}

void RCS_TIMER::set_timeout(double _timeout) {
    timeout = _timeout;
    if (timeout < clk_tck()) {
        counts_per_real_sleep = (int)(clk_tck() / _timeout) + 1;
    } else {
        counts_per_real_sleep = 0;
    }
}

void RCS_TIMER::init(double _timeout, int _id) {
    zero_timer();
    id = _id;
    set_timeout(_timeout);
}

void RCS_TIMER::zero_timer() {
    num_sems = 0;
#if USE_SEMS_FOR_TIMER
    sems = NULL;
#endif
    id = 0;
    function = NULL;
    arg = NULL;
    last_time = etime();
    idle = 0.0;
    counts = 0;
    start_time = etime();
    time_since_real_sleep = start_time;
    counts_per_real_sleep = 0;
    counts_since_real_sleep = 0;
    clk_tck_val = clk_tck();
    timeout = clk_tck_val;
}

void RCS_TIMER::sync() {
    last_time = etime();
}

int RCS_TIMER::wait() {
    double interval;
    double numcycles;
    int missed = 0;
    double remaining = 0.0;
    double time_in = 0.0;
    double time_done = 0.0;

    if (function != NULL) {
        time_in = etime();
        if ((*function)(arg) == -1) {
            return -1;
        }
        time_done = etime();
    } else {
        time_in = etime();
    }

    interval = time_in - last_time;
    numcycles = interval / timeout;

    counts++;
    if (function != NULL) {
        missed = (int)(numcycles - (clk_tck_val / timeout));
        idle += interval;
        last_time = time_done;
        remaining = 0.0;
    } else {
        missed = (int)(numcycles);
        remaining = timeout * (1.0 - (numcycles - (int)numcycles));
        idle += interval;
    }
    esleep(remaining);
    last_time = etime();
    return missed;
}

double RCS_TIMER::load() {
    if (counts * timeout != 0.0) return idle / (counts * timeout);
    return -1.0;
}
