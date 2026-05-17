/// \file rtapi_bitops.h
///
/// 原子位图操作函数集，提供用户空间的原子位操作实现。
///
/// 内核空间直接引用 <asm/bitops.h>，用户空间则使用 GCC 内置原子操作
///（__sync_* 系列）保证并发安全。
///
/// \note 所有位编号均从 0 开始。

#ifndef RTAPI_BITOPS_H
#define RTAPI_BITOPS_H

#if defined(__KERNEL__)
#include <asm/bitops.h>
#else
#include <limits.h>

/// \brief 计算 unsigned long 中的位数。
#define RTAPI_LONG_BIT (CHAR_BIT * sizeof(unsigned long))

/// \brief 原子地将指定位置 1。
///
/// 使用 __sync_fetch_and_or 实现原子"或"操作，具备内存屏障。
///
/// \param nr 要设置的位的零起始索引。
/// \param addr 指向位图（unsigned long 数组）的指针。
static __inline__ void set_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    __sync_fetch_and_or(laddr + loff, 1lu << boff);
}

/// \brief 原子地读取指定位的值。
///
/// 读取操作，不修改位图状态。
///
/// \param nr 要读取的位的零起始索引。
/// \param addr 指向位图的只读指针。
/// \return 该位被设置返回非零值，否则返回零。
static __inline__ int test_bit(int nr, const volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    return (laddr[loff] & (1lu << boff)) != 0;
}

/// \brief 原子地将指定位置零。
///
/// 使用 __sync_fetch_and_and 实现原子"与"操作，具备内存屏障。
///
/// \param nr 要清零的位的零起始索引。
/// \param addr 指向位图的指针。
static __inline__ void clear_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    __sync_fetch_and_and(laddr + loff, ~(1lu << boff));
}

/// \brief 原子地先测试再置位。
///
/// 先返回操作前的位值，再将该位置 1，保证测试与置位的原子性。
///
/// \param nr 要测试并置位的位的零起始索引。
/// \param addr 指向位图的指针。
/// \return 操作前该位为 1 返回非零值，为 0 返回零。
static __inline__ long test_and_set_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    unsigned long oldval = __sync_fetch_and_or(laddr + loff, 1lu << boff);
    return (oldval & (1lu << boff)) != 0;
}

/// \brief 原子地先测试再清零。
///
/// 先返回操作前的位值，再将该位置零，保证测试与清零的原子性。
///
/// \param nr 要测试并清零的位的零起始索引。
/// \param addr 指向位图的指针。
/// \return 操作前该位为 1 返回非零值，为 0 返回零。
static __inline__ long test_and_clear_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    unsigned long oldval = __sync_fetch_and_and(laddr + loff, ~(1lu << boff));
    return (oldval & (1lu << boff)) != 0;
}
#endif

#endif //RTAPI_BITOPS_H
