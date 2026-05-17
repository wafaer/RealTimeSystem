/// \file uspace_common.h
///
/// RTAPI 用户空间共享内存与 UUID 管理的公共数据结构定义。
///
/// 定义共享内存段的生命周期状态机、rtapi_shmem_handle 结构体、
/// uuid_data_t 以及相关的全局数组和宏。

#ifndef USPACE_COMMON_H
#define USPACE_COMMON_H

#include <errno.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <rtapi/rtapi_mutex.h>
#include "rtapi/rtapi.h"

// -----------------------------------------------------------------------------
// 共享内存生命周期状态机。
//
// 每个 rtapi_shmem_handle 在以下状态间转换：
//
//   UNBORN -> CREATED -> ATTACHED -> ACTIVE
//                               |            |
//                               v            v
//                           DETACHED     DESTROYED
//
// UNBORN:     槽位空闲，未持有任何 OS 资源。
// CREATED:    shmget() 成功，内核中存在该共享内存段。
// ATTACHED:   shmat() 成功，映射到进程虚拟地址空间，但尚未初始化。
// ACTIVE:     完全初始化，可正常读写。
// DETACHED:   shmdt() 已调用，本地映射已释放。
// DESTROYED:  shmctl(IPC_RMID) 已调用（或所有引用已释放）。
// ERROR:      创建或操作失败，不可恢复。
// -----------------------------------------------------------------------------

/// \brief 共享内存段的生命周期状态枚举。
enum {
    SHMEM_S_UNBORN    = 0,   ///< 槽位空闲，未持有任何 OS 资源。
    SHMEM_S_CREATED   = 1,   ///< shmget() 成功，内核中存在该段。
    SHMEM_S_ATTACHED  = 2,   ///< shmat() 成功，已映射但尚未初始化。
    SHMEM_S_ACTIVE    = 3,   ///< 完全初始化，可正常读写。
    SHMEM_S_DETACHED  = 4,   ///< shmdt() 已调用，本地映射已释放。
    SHMEM_S_DESTROYED = 5,   ///< shmctl(IPC_RMID) 已调用或引用已完全释放。
    SHMEM_S_ERROR     = -1   ///< 创建或操作失败，不可恢复。
};

/// \brief 共享内存段的句柄结构。
///
/// 保存 System V 共享内存的 IPC key、OS 级句柄、引用计数、
/// 生命周期状态以及最近一次错误码。
typedef struct {
    int magic;                      ///< 幻数，用于句柄有效性校验（值为 SHMEM_MAGIC）。
    int key;                        ///< System V IPC key，标识该共享内存区域。
    int id;                         ///< shmget() 返回的 OS 级共享内存标识符。
    int count;                      ///< 本进程对此段的映射次数（引用计数）。
    unsigned long int size;         ///< 共享内存区域的大小（字节）。
    void *mem;                      ///< 本进程中该段的映射虚拟地址，未映射时为 nullptr。

    /// 当前所处生命周期阶段，取值为 SHMEM_S_* 枚举值之一。
    /// BSS 段零初始化将其默认设为 SHMEM_S_UNBORN。
    int state;
    /// 创建该共享内存段的进程 PID，用于跨进程调试。
    pid_t owner_pid;
    /// 最近一次 OS 操作失败时捕获的 errno 值，正常时为 0。
    int error_code;
} rtapi_shmem_handle;

/// 共享内存句柄数组的最大槽位数。
#define MAX_SHM 64

/// 共享内存句柄的幻数签名，用于校验句柄有效性。
#define SHMEM_MAGIC   25453

/// 全局共享内存句柄数组，按句柄 ID 索引。
static rtapi_shmem_handle shmem_array[MAX_SHM];

/// \brief UUID 分配器的互斥锁与计数数据结构。
typedef struct {
    rtapi_mutex_t mutex;   ///< 保护 UUID 分配过程的原子性。
    int uuid;              ///< 下一个待分配的 UUID 值。
} uuid_data_t;

/// UUID 共享内存的 System V IPC key。
#define UUID_KEY  0x48484C34

/// UUID 共享内存在全局句柄数组中的索引（BSS 段初始化为 0）。
static int uuid_mem_id = 0;

/// \brief 获取高精度时间戳的宏，等价于读取 rtapi_get_time() 的返回值。
///
/// \param val 存放时间戳结果的变量（类型须为 long long）。
#define rdtscll(val) ((val) = rtapi_get_time())

#endif //USPACE_COMMON_H
