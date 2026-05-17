/// \file rtapi_stdint.h
///
/// RTAPI 层标准整数类型别名，将 HAL 层使用的 rtapi_u32 / rtapi_s32 /
/// rtapi_u64 / rtapi_s64 桥接至 C99 stdint 类型，保证跨平台字长一致。

#ifndef RTAPI_STDINT_H
#define RTAPI_STDINT_H

#include <stdint.h>

/// 有符号 32 位整数，等价于 int32_t。
typedef int32_t  rtapi_s32;
/// 无符号 32 位整数，等价于 uint32_t。
typedef uint32_t rtapi_u32;
/// 有符号 64 位整数，等价于 int64_t。
typedef int64_t  rtapi_s64;
/// 无符号 64 位整数，等价于 uint64_t。
typedef uint64_t rtapi_u64;

#include <stddef.h>

/// 与指针等宽的有符号整数，用于共享内存偏移计算（hal_priv.h 中 SHMOFF / SHMPTR 宏）。
typedef ptrdiff_t rtapi_intptr_t;

#endif // RTAPI_STDINT_H
