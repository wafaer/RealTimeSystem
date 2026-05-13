//
// rtapi/rtapi_stdint.h — standard integer type aliases for the RTAPI layer.
//
// These typedefs bridge the HAL's use of rtapi_u32 / rtapi_s32 / rtapi_u64 /
// rtapi_s64 back to C99 stdint types.
//

#ifndef RTAPI_STDINT_H
#define RTAPI_STDINT_H

#include <stdint.h>

typedef uint32_t rtapi_u32;
typedef int32_t  rtapi_s32;
typedef uint64_t rtapi_u64;
typedef int64_t  rtapi_s64;

// rtapi_intptr_t — pointer-sized signed integer, used for shared-memory
// offset arithmetic in hal_priv.h (SHMOFF / SHMPTR macros).
#include <stddef.h>
typedef ptrdiff_t rtapi_intptr_t;

#endif // RTAPI_STDINT_H
