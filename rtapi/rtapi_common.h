/// \file rtapi_common.h
///
/// RTAPI 核心数据结构的共享定义，供内核模块与用户空间进程共同使用。
///
/// 包含所有资源（模块、任务、共享内存、信号量、FIFO、中断）的数量上限、
/// 状态枚举以及主数据结构 rtapi_data_t 的定义。
///
/// \note 此文件被包含在共享内存映射区域，必须保证布局在编译单元间一致。

#ifndef RTAPI_COMMON_H
#define RTAPI_COMMON_H

/// 打开 RTAPI 共享内存段所用的 key。
#define RTAPI_KEY   0x90280A48
/// 验证共享内存块有效性的幻数，初次初始化时写入，完成后其他人不得重复初始化。
#define RTAPI_MAGIC 0x12601409

#include "rtapi_mutex.h"

/// 模块数量上限（含索引 0，故数组大小为 RTAPI_MAX_MODULES + 1）。
#define RTAPI_MAX_MODULES 64
/// 任务数量上限。
#define RTAPI_MAX_TASKS   64
/// 共享内存块数量上限。
#define RTAPI_MAX_SHMEMS  32
/// 信号量数量上限。
#define RTAPI_MAX_SEMS    64
/// FIFO 数量上限。
#define RTAPI_MAX_FIFOS   32
/// 中断处理数量上限。
#define RTAPI_MAX_IRQS    16
/// 模块/任务名称字符串的最大长度（不含终止符）。
#define RTAPI_NAME_LEN   31

/// RTAPI 互斥锁的底层表示，具体实现由 rtapi_mutex.h 提供。
typedef unsigned long rtapi_mutex_t;

/// 不透明的任务句柄类型，隐藏底层实现细节。
typedef struct rt_task_struct {
    int opaque;
} RT_TASK;

/// 不透明的信号量类型，隐藏底层实现细节。
typedef struct rt_semaphore {
    int opaque;
} SEM;

/// \brief 模块运行模式枚举。
typedef enum {
    NO_MODULE = 0,    ///< 未使用。
    REALTIME,         ///< 实时内核模块。
    USERSPACE         ///< 用户空间进程。
} mod_type_t;

/// \brief 任务调度状态枚举。
typedef enum {
    EMPTY = 0,        ///< 空闲，未分配。
    PAUSED,           ///< 已暂停。
    PERIODIC,        ///< 周期运行模式。
    FREERUN,         ///< 自由运行模式。
    ENDED             ///< 已结束。
} task_state_t;

/// \brief 模块在共享内存中的元数据。
typedef struct {
    mod_type_t state;                    ///< 当前模块的运行模式。
    char name[RTAPI_NAME_LEN + 1];       ///< 模块名称字符串。
} module_data;

/// \brief 任务的运行时数据。
typedef struct {
    task_state_t state;                  ///< 任务主状态。
    int prio;                            ///< 任务优先级（数值越小优先级越高）。
    int owner;                           ///< 拥有该任务的模块 ID。
    void (*taskcode)(void *);            ///< 任务入口函数。
    void *arg;                           ///< 传递给 taskcode 的用户参数。
} task_data;

/// \brief 共享内存块的元数据。
typedef struct {
    int key;                             ///< 共享内存区域关联的 key。
    int rtusers;                         ///< 正在使用该块的实时模块数量。
    int ulusers;                         ///< 正在使用该块的用户进程数量。
    unsigned long size;                  ///< 共享内存区域大小（字节）。
    /// 占用该共享内存的模块位图，每位对应一个模块索引。
    unsigned long bitmap[(RTAPI_MAX_SHMEMS / 8) + 1];
} shmem_data;

/// \brief FIFO 状态枚举，作为位掩码使用。
typedef enum {
    UNUSED = 0,      ///< 未被使用。
    HAS_READER = 1,  ///< 有读取方。
    HAS_WRITER = 2,  ///< 有写入方。
    HAS_BOTH = 3     ///< 读写双方均已连接。
} fifo_state_t;

/// \brief FIFO 的元数据。
typedef struct {
    fifo_state_t state;                 ///< FIFO 的读写双方状态。
    int key;                             ///< FIFO 的 key。
    int reader;                          ///< 读取方模块 ID。
    int writer;                          ///< 写入方模块 ID。
    unsigned long int size;             ///< FIFO 数据区域大小（字节）。
} fifo_data;

/// \brief 信号量的元数据。
typedef struct {
    int users;                           ///< 当前持有或等待该信号量的模块数量。
    int key;                             ///< 信号量的 key。
    /// 占用该信号量的模块位图，每位对应一个模块索引。
    unsigned long bitmap[(RTAPI_MAX_SEMS / 8) + 1];
} sem_data;

/// \brief 已注册中断的元数据。
typedef struct {
    int irq_num;                         ///< IRQ 硬件编号。
    int owner;                           ///< 拥有该中断的模块 ID。
    void (*handler)(void);               ///< 中断处理函数。
} irq_data;

/// \brief RTAPI 全局主数据结构，所有资源表均嵌入其中。
///
/// 该结构体通过共享内存在内核模块与用户空间进程间共享，
/// 因此必须保证二进制布局在所有编译单元中完全一致。
typedef struct {
    int magic;                            ///< 幻数，验证本结构体是否已初始化。
    int rev_code;                         ///< 版本号，模块间用来检测数据布局是否匹配。
    rtapi_mutex_t mutex;                  ///< 访问本结构体的全局互斥锁。

    int rt_module_count;                   ///< 已加载的实时模块数量。
    int ul_module_count;                 ///< 运行中的用户空间进程数量。
    int task_count;                       ///< 当前已分配的任务 ID 数量。
    int shmem_count;                      ///< 已创建的共享内存块数量。
    int sem_count;                        ///< 已创建的信号量数量。
    int fifo_count;                        ///< 已创建的 FIFO 数量。
    int irq_count;                         ///< 已注册的中断处理数量。
    int timer_running;                    ///< 硬件定时器是否正在运行（0/1）。
    int rt_cpu;                           ///< 实时任务绑定的 CPU 编号。
    long int timer_period;                ///< 硬件定时器周期（纳秒）。

    module_data module_array[RTAPI_MAX_MODULES + 1];  ///< 模块数据表。
    task_data task_array[RTAPI_MAX_TASKS + 1];       ///< 任务数据表。
    shmem_data shmem_array[RTAPI_MAX_SHMEMS + 1];    ///< 共享内存数据表。
    sem_data sem_array[RTAPI_MAX_SEMS + 1];          ///< 信号量数据表。
    fifo_data fifo_array[RTAPI_MAX_FIFOS + 1];       ///< FIFO 数据表。
    irq_data irq_array[RTAPI_MAX_IRQS + 1];          ///< 中断数据表。
} rtapi_data_t;

/// 全局版本号，供各模块在初始化时写入 rtapi_data_t::rev_code。
static unsigned int rev_code = 1;

/// \brief 初始化 rtapi_data_t 结构体（幂等操作）。
///
/// 检查 magic 字段，若未初始化则获取互斥锁并清零所有字段，
/// 完成后释放锁。若已初始化则直接返回。
///
/// \param data 指向 rtapi_data_t 的指针，必须指向一块有效的共享内存区域。
static void init_rtapi_data(rtapi_data_t * data)
{
    int n, m;

    if (data->magic == RTAPI_MAGIC) {
        return;
    }

    rtapi_mutex_try(&(data->mutex));
    data->magic = RTAPI_MAGIC;
    data->rev_code = rev_code;
    data->rt_module_count = 0;
    data->ul_module_count = 0;
    data->task_count = 0;
    data->shmem_count = 0;
    data->sem_count = 0;
    data->fifo_count = 0;
    data->irq_count = 0;
    data->timer_running = 0;
    data->timer_period = 0;

    for (n = 0; n <= RTAPI_MAX_MODULES; n++) {
        data->module_array[n].state = NO_MODULE;
        data->module_array[n].name[0] = '\0';
    }
    for (n = 0; n <= RTAPI_MAX_TASKS; n++) {
        data->task_array[n].state = EMPTY;
        data->task_array[n].prio = 0;
        data->task_array[n].owner = 0;
        data->task_array[n].taskcode = NULL;
    }
    for (n = 0; n <= RTAPI_MAX_SHMEMS; n++) {
        data->shmem_array[n].key = 0;
        data->shmem_array[n].rtusers = 0;
        data->shmem_array[n].ulusers = 0;
        data->shmem_array[n].size = 0;
        for (m = 0; m < (RTAPI_MAX_SHMEMS / 8) + 1; m++) {
            data->shmem_array[n].bitmap[m] = 0;
        }
    }
    for (n = 0; n <= RTAPI_MAX_SEMS; n++) {
        data->sem_array[n].users = 0;
        data->sem_array[n].key = 0;
        for (m = 0; m < (RTAPI_MAX_SEMS / 8) + 1; m++) {
            data->sem_array[n].bitmap[m] = 0;
        }
    }
    for (n = 0; n <= RTAPI_MAX_FIFOS; n++) {
        data->fifo_array[n].state = UNUSED;
        data->fifo_array[n].key = 0;
        data->fifo_array[n].size = 0;
        data->fifo_array[n].reader = 0;
        data->fifo_array[n].writer = 0;
    }
    for (n = 0; n <= RTAPI_MAX_IRQS; n++) {
        data->irq_array[n].irq_num = 0;
        data->irq_array[n].owner = 0;
        data->irq_array[n].handler = NULL;
    }

    rtapi_mutex_give(&(data->mutex));
}

#endif //RTAPI_COMMON_H
