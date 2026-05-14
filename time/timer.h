//
// Created by wg on 5/14/26.
//

#ifndef REALTIMESYSTEM_TIMER_H
#define REALTIMESYSTEM_TIMER_H

extern "C" {
#include <stdio.h>		/* NULL */
} extern "C" {
    /* number of seconds from standard epoch, to clock tick resolution */
    extern double etime(void);
    /* Return the number of seconds from some event. The time will be rounded
       up to the resolution of the system clock or the most precise time
       measuring function available for the given platform. For the value
       returned to mean anything you need to be able to compare it with a
       value stored from a previous call to etime(). */

    /* sleeps # of seconds, to clock tick resolution */
    extern void esleep(double secs);
    /* Go to sleep for _secs seconds. The time will be rounded up to the
       resolution of the system clock or the most precise sleep or delay
       function available for the given platform. */
}
class RCS_SEMAPHORE;

/* prototype for user-defined timing function */
typedef int (*RCS_TIMERFUNC) (void *_arg);

class RCS_TIMER {
  public:
/* Getting rid of this stuff which no one uses and makes porting more
difficult */
    RCS_TIMER(double timeout, RCS_TIMERFUNC function =
	(RCS_TIMERFUNC) NULL, void *arg = NULL);

      RCS_TIMER(const char *process_name, const char *timer_config_file);
      RCS_TIMER(double _timeout, const char *process_name, const char *timer_config_file);
    /* Initialize a new RCS_TIMER object. _interval is the cycle period.
       _function is an optional function for synchronizing to an event other
       than the system clock. _arg is a parameter that will be passed to
       function. */

    /* timeout is wait interval, rounded up to clock tick resolution;
       function is external time base, if provided */
     ~RCS_TIMER();
    int wait();			/* wait on synch; returns # of cycles missed */
    /* Wait until the end of interval or until a user function returns.

       Returns: 0 for success, the number of cycles missed if it missed some
       cycles, or -1 if some other error occurred. */

    double load();		/* returns % loading on timer, 0.0 means all
				   waits, 1.0 means no time in wait */
    /* Returns the percentage of loading by the cyclic process. If the
       process spends all of its time waiting for the synchronizing event
       then it returns 0.0. If it spends all of its time doing something else
       before calling wait then it returns 1.0. The load percentage is the
       average load over all of the previous cycles. */
    void sync();		/* restart the wait interval. */
    /* Restart the wait interval now. */
    double timeout;		/* copy of timeout */

  private:
    void init(double _timeout, int _id);

    void zero_timer();
    void set_timeout(double _timeout);
/*! \todo Another #if 0 */
#if 0
    void read_config_file(char *process, char *config_file);
#endif
    RCS_TIMERFUNC function;	/* copy of function */
    void *arg;			/* arg for function */
    double last_time;		/* last wakeup, in ticks from epoch */
    double start_time;		/* epoch time at creation of timer */
    double idle;		/* accumulated idle time */
    int counts;			/* accumulated waits */
    int counts_since_real_sleep;
    int counts_per_real_sleep;
    double time_since_real_sleep;
#ifdef USE_SEMS_FOR_TIMER
    RCS_SEMAPHORE **sems;
#endif
    int num_sems;
    int id;
    double clk_tck_val;
};

#endif //REALTIMESYSTEM_TIMER_H