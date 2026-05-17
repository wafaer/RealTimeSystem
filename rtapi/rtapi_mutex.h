/// \file rtapi_mutex.h
///
/// 基于原子位操作的简易互斥锁实现，供内核与用户空间共用。
///
/// 互斥锁仅用一位（bit 0）表示状态：0 表示未持有，1 表示已持有。
/// 所有操作均依赖 rtapi_bitops.h 中的原子位指令，无需额外系统调用。

#ifndef RTAPI_MUTEX_H
#define RTAPI_MUTEX_H

#if defined(__KERNEL__)
#include <linux/sched.h>
#else
#include <sched.h>
#endif

#include "rtapi_bitops.h"

/// 互斥锁的底层表示，仅使用 bit 0。
typedef unsigned long rtapi_mutex_t;

/// \brief 释放互斥锁。
///
/// 无论调用方是否真正持有锁，都会将 bit 0 清零。
/// 用于确保临界区结束后锁被释放。
///
/// \param mutex 指向互斥锁的指针。
static __inline__ void rtapi_mutex_give(unsigned long *mutex) {
    test_and_clear_bit(0, mutex);
}

/// \brief 非阻塞地尝试获取互斥锁。
///
/// 原子地测试并置位 bit 0，若位原本为 0（锁空闲）则成功获取并返回 0；
/// 若位原本为 1（锁已被持有）则立即返回非零值，锁状态不变。
///
/// \param mutex 指向互斥锁的指针。
/// \return 锁空闲返回 0（成功获取）；锁已被持有返回非零值（未获取）。
static __inline__ int rtapi_mutex_try(unsigned long *mutex) {
    return test_and_set_bit(0, mutex);
}

/// \brief 阻塞地等待获取互斥锁。
///
/// 循环调用 rtapi_mutex_try，直到成功获取锁后返回。
/// 每次重试失败后会主动让出 CPU，使持有锁的其他线程有机会释放。
///
/// \param mutex 指向互斥锁的指针。
/// \warning 不可在实时线程上下文中调用，可能导致实时性下降。
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
