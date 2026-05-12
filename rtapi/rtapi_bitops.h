//
// Created by Administrator on 2025/8/15.
//

#ifndef RTAPI_BITOPS_H
#define RTAPI_BITOPS_H

#if defined(__KERNEL__)
#include <asm/bitops.h>
#else
#include <limits.h>
#define RTAPI_LONG_BIT (CHAR_BIT * sizeof(unsigned long))
static __inline__ void set_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    __sync_fetch_and_or(laddr + loff, 1lu << boff);
}

static __inline__ int test_bit(int nr, const volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    return (laddr[loff] & (1lu << boff)) != 0;
}

static __inline__ void clear_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    __sync_fetch_and_and(laddr + loff, ~(1lu << boff));
}

static __inline__ long test_and_set_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    unsigned long oldval = __sync_fetch_and_or(laddr + loff, 1lu << boff);
    return (oldval & (1lu << boff)) != 0;
}

static __inline__ long test_and_clear_bit(int nr, volatile void *addr) {
    size_t loff = nr / RTAPI_LONG_BIT;
    size_t boff = nr % RTAPI_LONG_BIT;
    unsigned long *laddr = (unsigned long*)addr;
    unsigned long oldval = __sync_fetch_and_and(laddr + loff, ~(1lu << boff));
    return (oldval & (1lu << boff)) != 0;
}
#endif

#endif //RTAPI_BITOPS_H
