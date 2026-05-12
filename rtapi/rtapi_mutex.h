//
// Created by Administrator on 2025/8/15.
//

#ifndef RTAPI_MUTEX_H
#define RTAPI_MUTEX_H

#if defined(__KERNEL__)
#include <linux/sched.h>	/* for blocking when needed */
#else
#include <sched.h>
#endif

#include "rtapi_bitops.h"

typedef unsigned long rtapi_mutex_t;


/**
 * @brief Releases the mutex.
 *
 * The release is unconditional, even if the caller doesn't have the mutex, it
 * will be released.
 * @param mutex Pointer to the @c mutex.
 */
static __inline__ void rtapi_mutex_give(unsigned long *mutex) {
    test_and_clear_bit(0, mutex);
}
/**
 * @brief Non-blocking attempt to get the mutex.
 *
 * The programmer is responsible for "doing the right thing" when it returns
 * non-zero. "Doing the right thing" almost certainly means doing something
 * that will yield the CPU, so that whatever other process has the mutex gets a
 * chance to release it.
 * @param mutex Pointer to the @c mutex.
 * @return If the mutex is available, it returns 0, and the mutex is no longer
 *         available. Otherwise, it returns a nonzero value indicating that
 *         someone else has the mutex.
 */
static __inline__ int rtapi_mutex_try(unsigned long *mutex) {
    return test_and_set_bit(0, mutex);
}

/**
 * @brief Blocking attempt to gGet the mutex.
 *
 * This function will block if the mutex is not available. Because of this,
 * calling it from a realtime main is a "very bad" thing to do.
 * @param mutex Pointer to the @c mutex.
 * @note Can not be used in realtime code.
 */
static __inline__ void rtapi_mutex_get(unsigned long *mutex)
{
    while (test_and_set_bit(0, mutex)) {
#if defined(__KERNEL__)
        schedule();
#else
        sched_yield();
#endif
    }
}

#endif //RTAPI_MUTEX_H
