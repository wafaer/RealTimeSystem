/// \file hal_core.c
///
/// HAL 核心管理实现。
///
/// 负责 HAL 共享内存子系统的启动与关闭、组件注册与生命周期状态管理、
/// 全局锁级别控制。组件分配器（halpr_alloc_comp_struct / free_comp_struct）
/// 也在此处实现，因为组件是引脚、参数和函数的"拥有者"，销毁组件时需要
/// 级联清理其关联的子注册表。

#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/shm.h>

#include "hal.h"
#include "hal_priv.h"
#include "rtapi/rtapi_common.h"
#include "rtapi/rtapi.h"

/// HAL 共享内存在当前进程中的基地址。
char *hal_shmem_base = 0;
/// 指向 HAL 共享内存数据区（hal_data_t）的指针。
hal_data_t *hal_data = 0;
/// 当前 HAL 库的 RTAPI 模块 ID。
int lib_module_id = -1;

/// 当前进程持有 HAL 组件的数量（用于追踪是否还有未退出的组件）。
static int ref_cnt = 0;
/// HAL 共享内存的 RTAPI 句柄 ID。
static int lib_mem_id = 0;

/// 前向声明：孤儿共享内存回收函数。
static void reclaim_orphan_hal_shm(void);

// -----------------------------------------------------------------------------
// hal_init — 注册 HAL 组件
// -----------------------------------------------------------------------------

/// \brief 注册一个新的 HAL 组件，首次调用时还会初始化 HAL 共享内存子系统。
///
/// 首次调用时（lib_mem_id == 0）完成以下初始化步骤：
/// 1. 校验 HAL 未被销毁（state != HAL_S_DESTROYED）
/// 2. 通过 rtapi_init 创建 HAL_LIB 模块
/// 3. 回收孤儿共享内存段
/// 4. 向 RTAPI 申请 HAL 共享内存
/// 5. 调用 init_hal_data() 完成数据区初始化
///
/// 之后每次调用均在已初始化的共享内存中注册新组件（需唯一名称）。
///
/// \param name 组件名称（不超过 HAL_NAME_LEN 字符）。
/// \return 组件 ID（>= 0），失败返回负值 errno。
int hal_init(const char *name) {
    int comp_id;
    int retval;
    void *mem;
    char rtapi_name[RTAPI_NAME_LEN + 1];
    char hal_name[HAL_NAME_LEN + 1];
    hal_comp_t *comp;

    if (name == 0) {
        return -EINVAL;
    }
    if (strlen(name) > HAL_NAME_LEN) {
        return -EINVAL;
    }

    if (!lib_mem_id) {
        // 防护：HAL 已销毁后禁止重新初始化。
        if (hal_data && hal_data->state == HAL_S_DESTROYED) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: hal_init called after destruction\n");
            return -EINVAL;
        }
        rt_snprintf(rtapi_name, RTAPI_NAME_LEN, "HAL_LIB_%d", (int)getpid());
        lib_module_id = rtapi_init(rtapi_name);
        if (lib_module_id < 0) {
            return -EINVAL;
        }

        // 若前一个进程崩溃遗留了 HAL 共享内存段，先销毁再申请新的。
        reclaim_orphan_hal_shm();

        lib_mem_id = rtapi_shmem_new(HAL_KEY, lib_module_id, HAL_SIZE);
        if (lib_mem_id < 0) {
            rtapi_exit(lib_module_id);
            return -EINVAL;
        }
        retval = rtapi_shmem_getptr(lib_mem_id, &mem);
        if (retval < 0) {
            rtapi_exit(lib_module_id);
            return -EINVAL;
        }
        hal_shmem_base = (char *)mem;
        hal_data = (hal_data_t *)mem;
        retval = init_hal_data();
        if (retval) {
            rtapi_exit(lib_module_id);
            return -EINVAL;
        }
    }

    rt_snprintf(rtapi_name, RTAPI_NAME_LEN, "HAL_%s", name);
    rt_snprintf(hal_name, sizeof(hal_name), "%s", name);

    comp_id = rtapi_init(rtapi_name);
    if (comp_id < 0) {
        return -EINVAL;
    }

    rtapi_mutex_get(&(hal_data->mutex));

    // 检查名称唯一性。
    if (halpr_find_comp_by_name(hal_name) != 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_exit(comp_id);
        return -EINVAL;
    }

    comp = halpr_alloc_comp_struct();
    if (comp == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_exit(comp_id);
        return -ENOMEM;
    }

    comp->comp_id = comp_id;
    comp->type = COMPONENT_TYPE_REALTIME;
    comp->pid = 0;
    comp->ready = 0;
    comp->shmem_base = hal_shmem_base;
    comp->insmod_args = 0;
    comp->next_ptr = hal_data->comp_list_ptr;
    hal_data->comp_list_ptr = SHMOFF(comp);

    rtapi_mutex_give(&(hal_data->mutex));
    ref_cnt++;
    return comp_id;
}

// -----------------------------------------------------------------------------
// hal_exit — 注销 HAL 组件
// -----------------------------------------------------------------------------

/// \brief 注销指定的 HAL 组件，从链表中摘除并释放其结构体。
///
/// 同时调用 free_comp_struct 级联清理该组件拥有的所有函数和参数结构体。
///
/// \param comp_id 要注销的组件 ID。
/// \return 成功返回 0；HAL 未初始化、组件不存在或已销毁返回负值。
int hal_exit(int comp_id) {
    rtapi_intptr_t *prev, next;
    hal_comp_t *comp;
    char name[HAL_NAME_LEN + 1];

    if (hal_data == 0) {
        return -EINVAL;
    }

    // 防护：禁止操作已销毁的 HAL。
    if (hal_data->state == HAL_S_DESTROYED) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: hal_exit called after destruction\n");
        return -EALREADY;
    }

    rtapi_mutex_get(&(hal_data->mutex));

    prev = &(hal_data->comp_list_ptr);
    next = *prev;
    if (next == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    comp = SHMPTR(next);
    while (comp->comp_id != comp_id) {
        prev = &(comp->next_ptr);
        next = *prev;
        if (next == 0) {
            rtapi_mutex_give(&(hal_data->mutex));
            return -EINVAL;
        }
        comp = SHMPTR(next);
    }

    // 从链表中摘除并释放。
    *prev = comp->next_ptr;
    rt_snprintf(name, sizeof(name), "%s", comp->name);
    free_comp_struct(comp);

    rtapi_mutex_give(&(hal_data->mutex));
    --ref_cnt;
    rtapi_exit(comp_id);

    return 0;
}

// -----------------------------------------------------------------------------
// hal_malloc — HAL 共享内存分配
// -----------------------------------------------------------------------------

/// \brief 在 HAL 共享内存池中分配内存块。
///
/// 分配的内存可被多个 HAL 组件共享，适用于组件间通过引脚传递数据。
///
/// \param size 要分配的字节数。
/// \return 分配到的内存指针；HAL 未初始化或分配失败返回 nullptr。
void *hal_malloc(long int size) {
    void *retval;

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: hal_malloc called before init\n");
        return 0;
    }
    rtapi_mutex_get(&(hal_data->mutex));
    retval = shmalloc_up(size);
    rtapi_mutex_give(&(hal_data->mutex));
    return retval;
}

// -----------------------------------------------------------------------------
// hal_set_constructor — 注册组件构造函数
// -----------------------------------------------------------------------------

/// \brief 为指定组件注册延迟构造函数。
///
/// 构造函数将在组件首次就绪时被调用，用于动态实例化。
///
/// \param comp_id 组件 ID。
/// \param make    构造函数函数指针。
/// \return 成功返回 0；组件未找到返回 -EINVAL。
int hal_set_constructor(int comp_id, constructor make) {
    int next;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));

    next = hal_data->comp_list_ptr;
    if (next == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: component %d not found\n", comp_id);
        return -EINVAL;
    }

    comp = SHMPTR(next);
    while (comp->comp_id != comp_id) {
        next = comp->next_ptr;
        if (next == 0) {
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: component %d not found\n", comp_id);
            return -EINVAL;
        }
        comp = SHMPTR(next);
    }

    comp->make = make;

    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

// -----------------------------------------------------------------------------
// 组件就绪状态管理
// -----------------------------------------------------------------------------

/// \brief 将指定组件标记为未就绪状态（初始化中途失败时使用）。
/// \param comp_id 组件 ID。
/// \return 成功返回 0；组件未找到返回 -EINVAL。
int hal_set_unready(int comp_id) {
    hal_comp_t *comp;
    rtapi_mutex_get(&(hal_data->mutex));
    comp = halpr_find_comp_by_id(comp_id);
    if (comp) {
        comp->ready = 0;
    }
    rtapi_mutex_give(&(hal_data->mutex));
    if (comp) {
        return 0;
    }
    rtapi_print_msg(RTAPI_MSG_ERR,
        "HAL: ERROR: hal_set_unready(): component %d not found\n", comp_id);
    return -EINVAL;
}

/// \brief 将指定组件标记为就绪状态，允许其引脚和参数被其他组件使用。
/// \param comp_id 组件 ID。
/// \return 成功返回 0；组件不存在或已就绪返回 -EINVAL。
int hal_ready(int comp_id) {
    int next;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));

    next = hal_data->comp_list_ptr;
    if (next == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: component %d not found\n", comp_id);
        return -EINVAL;
    }

    comp = SHMPTR(next);
    while (comp->comp_id != comp_id) {
        next = comp->next_ptr;
        if (next == 0) {
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: component %d not found\n", comp_id);
            return -EINVAL;
        }
        comp = SHMPTR(next);
    }
    if (comp->ready > 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: Component '%s' already ready\n", comp->name);
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    comp->ready = 1;
    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

/// \brief 取消指定组件的就绪状态。
/// \param comp_id 组件 ID。
/// \return 成功返回 0；组件不存在或已未就绪返回 -EINVAL。
int hal_unready(int comp_id) {
    int next;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));

    next = hal_data->comp_list_ptr;
    if (next == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: component %d not found\n", comp_id);
        return -EINVAL;
    }

    comp = SHMPTR(next);
    while (comp->comp_id != comp_id) {
        next = comp->next_ptr;
        if (next == 0) {
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: component %d not found\n", comp_id);
            return -EINVAL;
        }
        comp = SHMPTR(next);
    }
    if (comp->ready < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: Component '%s' already unready\n", comp->name);
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    comp->ready = 0;
    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

// -----------------------------------------------------------------------------
// 辅助查询
// -----------------------------------------------------------------------------

/// \brief 根据组件 ID 查询其名称字符串。
/// \param comp_id 组件 ID。
/// \return 组件名称字符串；未找到返回 nullptr。
char *hal_comp_name(int comp_id) {
    hal_comp_t *comp;
    char *result = NULL;
    rtapi_mutex_get(&(hal_data->mutex));
    comp = halpr_find_comp_by_id(comp_id);
    if (comp) result = comp->name;
    rtapi_mutex_give(&(hal_data->mutex));
    return result;
}

// -----------------------------------------------------------------------------
// 全局锁级别
// -----------------------------------------------------------------------------

/// \brief 设置 HAL 全局锁级别，控制哪些 HAL 操作在运行时被禁止。
/// \param lock_type 锁级别（HAL_LOCK_* 的按位 OR 组合）。
/// \return 成功返回 0；HAL 未初始化返回 -EINVAL。
int hal_set_lock(unsigned char lock_type) {
    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: set_lock called before init\n");
        return -EINVAL;
    }
    hal_data->lock = lock_type;
    return 0;
}

/// \brief 获取当前 HAL 全局锁级别。
/// \return 当前锁级别；HAL 未初始化返回 -EINVAL（按 unsigned char 解释为 255）。
unsigned char hal_get_lock() {
    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: get_lock called before init\n");
        return -EINVAL;
    }
    return hal_data->lock;
}

// -----------------------------------------------------------------------------
// hal_app_main — HAL 子系统全量初始化
// -----------------------------------------------------------------------------

/// \brief 完成 HAL 共享内存子系统的完整初始化。
///
/// 生命周期转换：UNINIT -> INITIALIZING -> ACTIVE。
/// 任何步骤失败均会回滚已获取的 OS 资源。
///
/// \return 成功返回 0，失败返回负值 errno。
int hal_app_main(void) {
    int retval;
    void *mem;

    lib_module_id = rtapi_init("HAL_LIB");
    if (lib_module_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LIB: ERROR: rtapi init failed\n");
        return -EINVAL;
    }

    // 回收孤儿共享内存段（上一个进程崩溃遗留）。
    reclaim_orphan_hal_shm();

    lib_mem_id = rtapi_shmem_new(HAL_KEY, lib_module_id, HAL_SIZE);
    if (lib_mem_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL_LIB: ERROR: could not open shared memory\n");
        rtapi_exit(lib_module_id);
        return -EINVAL;
    }

    // 状态已转为 INITIALIZING（共享内存已获取但数据区尚未初始化）。

    retval = rtapi_shmem_getptr(lib_mem_id, &mem);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL_LIB: ERROR: could not access shared memory\n");
        rtapi_shmem_delete(lib_mem_id, lib_module_id);
        rtapi_exit(lib_module_id);
        return -EINVAL;
    }
    hal_shmem_base = (char *)mem;
    hal_data = (hal_data_t *)mem;

    // 在调用 init_hal_data() 之前先将状态标记为 INITIALIZING。
    hal_data->state = HAL_S_INITIALIZING;
    hal_data->error_code = 0;

    retval = init_hal_data();
    if (retval) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL_LIB: ERROR: could not init shared memory\n");
        rtapi_shmem_delete(lib_mem_id, lib_module_id);
        rtapi_exit(lib_module_id);
        return -EINVAL;
    }

    rtapi_print_msg(RTAPI_MSG_DBG,
        "HAL_LIB: kernel lib installed successfully\n");
    return 0;
}

// -----------------------------------------------------------------------------
// hal_app_exit — HAL 子系统关闭
// -----------------------------------------------------------------------------

/// \brief 关闭整个 HAL 子系统，释放所有资源。
///
/// 生命周期转换：ACTIVE/RUNNING -> SHUTTING_DOWN -> DESTROYED。
/// 幂等：若已处于 DESTROYED 状态则直接返回。
void hal_app_exit(void) {
    hal_thread_t *thread;

    if (hal_data == NULL) {
        return;
    }
    if (hal_data->state == HAL_S_DESTROYED) {
        return;
    }

    hal_data->state = HAL_S_SHUTTING_DOWN;

    rtapi_mutex_get(&(hal_data->mutex));
    // 必须在卸载模块前移除所有线程。
    while (hal_data->thread_list_ptr != 0) {
        thread = SHMPTR(hal_data->thread_list_ptr);
        hal_data->thread_list_ptr = thread->next_ptr;
        free_thread_struct(thread);
    }
    rtapi_mutex_give(&(hal_data->mutex));

    rtapi_shmem_delete(lib_mem_id, lib_module_id);
    rtapi_exit(lib_module_id);

    hal_data->state = HAL_S_DESTROYED;
}

// -----------------------------------------------------------------------------
// reclaim_orphan_hal_shm — 回收孤儿共享内存段
// -----------------------------------------------------------------------------

/// \brief 尽力回收上一个崩溃进程遗留的 HAL 共享内存段。
///
/// 背景：HAL 共享内存段以 HAL_KEY 为 key，其生命周期由内核管理。
/// 若上一个进程崩溃（SIGKILL / SIGSEGV / 断电），内核保留该段，
/// 新进程重新 attach 会遭遇陈旧的链表结构和可能卡住的互斥锁。
///
/// 本函数通过 shmget/shmat 探测该段，读取 init_pid 字段，
/// 若该 PID 已不存在（kill(pid,0) 返回 ESRCH），则用 IPC_RMID
/// 标记销毁，使后续 shmget(IPC_CREAT) 创建全新的段。
/// 若 PID 仍存活则保留（合法 attach 场景）。
static void reclaim_orphan_hal_shm(void) {
    int id = shmget((key_t)HAL_KEY, 0, 0);
    if (id < 0) return;   // 无遗留段。

    struct shmid_ds info;
    if (shmctl(id, IPC_STAT, &info) < 0) return;
    // 若段大小小于 hal_data_t，视为无效，直接销毁。
    if (info.shm_segsz < sizeof(hal_data_t)) {
        shmctl(id, IPC_RMID, NULL);
        return;
    }

    void *addr = shmat(id, NULL, 0);
    if (addr == (void *)-1) {
        shmctl(id, IPC_RMID, NULL);
        return;
    }

    hal_data_t *old = (hal_data_t *)addr;
    int orphan = 0;
    if (old->init_pid <= 0) {
        orphan = 1;
    } else if (kill(old->init_pid, 0) == -1 && errno == ESRCH) {
        orphan = 1;
    }
    shmdt(addr);

    if (orphan) {
        if (shmctl(id, IPC_RMID, NULL) == 0) {
            rtapi_print_msg(RTAPI_MSG_INFO,
                "HAL: reclaimed orphan HAL shm (id=%d)\n", id);
        }
    }
}

// -----------------------------------------------------------------------------
// 状态查询与统计
// -----------------------------------------------------------------------------

/// \brief 获取当前 HAL 子系统的生命周期状态。
/// \return HAL 状态（HAL_S_* 枚举值），未初始化返回 HAL_S_UNINIT。
int hal_get_state(void) {
    if (hal_data == NULL)
        return HAL_S_UNINIT;
    return hal_data->state;
}

/// \brief 填充 HAL 统计信息快照。
///
/// 在持有 hal_data->mutex 的情况下遍历所有链表并计数。
///
/// \param stats 调用方分配的统计结构体指针。
/// \return 成功返回 0；HAL 未初始化或 stats 为空返回 -EINVAL。
int hal_get_stats(hal_stats_t *stats) {
    if (hal_data == NULL || stats == NULL)
        return -EINVAL;

    rtapi_mutex_get(&(hal_data->mutex));

    stats->state = hal_data->state;
    stats->init_pid = hal_data->init_pid;
    stats->error_code = hal_data->error_code;
    stats->threads_running = hal_data->threads_running;
    stats->shmem_avail = hal_data->shmem_avail;
    stats->shmem_total = HAL_SIZE;

    stats->comp_count = 0;
    {
        int next = hal_data->comp_list_ptr;
        while (next != 0) {
            hal_comp_t *comp = SHMPTR(next);
            stats->comp_count++;
            next = comp->next_ptr;
        }
    }

    stats->pin_count = 0;
    {
        int next = hal_data->pin_list_ptr;
        while (next != 0) {
            hal_pin_t *pin = SHMPTR(next);
            stats->pin_count++;
            next = pin->next_ptr;
        }
    }

    stats->param_count = 0;
    {
        int next = hal_data->param_list_ptr;
        while (next != 0) {
            hal_param_t *param = SHMPTR(next);
            stats->param_count++;
            next = param->next_ptr;
        }
    }

    stats->funct_count = 0;
    {
        int next = hal_data->funct_list_ptr;
        while (next != 0) {
            hal_funct_t *funct = SHMPTR(next);
            stats->funct_count++;
            next = funct->next_ptr;
        }
    }

    stats->thread_count = 0;
    {
        int next = hal_data->thread_list_ptr;
        while (next != 0) {
            hal_thread_t *thread = SHMPTR(next);
            stats->thread_count++;
            next = thread->next_ptr;
        }
    }

    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

// -----------------------------------------------------------------------------
// init_hal_data — 初始化 HAL 共享内存数据区
// -----------------------------------------------------------------------------

/// \brief 初始化 HAL 共享内存数据区（hal_data_t 及所有链表）。
///
/// 在持有互斥锁的前提下：
/// 1. 若互斥锁被卡住且状态仍为 UNINIT 或 INITIALIZING，
///    视为上一个崩溃进程的残留，强制清除锁。
/// 2. 将所有链表头和空闲链表头置零。
/// 3. 初始化共享内存池边界（shmem_bot 从 hal_data_t 末端开始）。
/// 4. 设置状态为 ACTIVE，记录当前 PID。
///
/// \return 始终返回 0。
int init_hal_data(void) {
    // 尝试获取互斥锁。若已被占用且状态仍为 UNINIT/INITIALIZING，
    // 说明是上一个崩溃进程遗留的锁，通过 give+get 序列强制清除。
    if (rtapi_mutex_try(&(hal_data->mutex)) != 0) {
        hal_state_t s = hal_data->state;
        if (s != HAL_S_UNINIT && s != HAL_S_INITIALIZING) {
            rtapi_mutex_get(&(hal_data->mutex));
        } else {
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_mutex_get(&(hal_data->mutex));
        }
    }

    hal_data->comp_list_ptr = 0;
    hal_data->param_list_ptr = 0;
    hal_data->funct_list_ptr = 0;
    hal_data->thread_list_ptr = 0;
    hal_data->base_period = 0;
    hal_data->threads_running = 0;
    hal_data->oldname_free_ptr = 0;
    hal_data->comp_free_ptr = 0;
    hal_data->param_free_ptr = 0;
    hal_data->funct_free_ptr = 0;
    hal_data->pending_constructor = 0;
    hal_data->constructor_prefix[0] = 0;
    list_init_entry(&(hal_data->funct_entry_free));
    hal_data->thread_free_ptr = 0;
    hal_data->exact_base_period = 0;
    hal_data->shmem_bot = sizeof(hal_data_t);
    hal_data->shmem_top = HAL_SIZE;
    hal_data->lock = HAL_LOCK_NONE;

    hal_data->state = HAL_S_ACTIVE;
    hal_data->init_pid = getpid();
    hal_data->error_code = 0;
    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

// -----------------------------------------------------------------------------
// halpr_alloc_comp_struct — 分配组件结构体
// -----------------------------------------------------------------------------

/// \brief 从空闲链表分配一个组件结构体，若空闲链表为空则从共享内存池底部分配。
///
/// \return 分配到的组件指针；共享内存耗尽返回 nullptr。
hal_comp_t *halpr_alloc_comp_struct(void) {
    hal_comp_t *p;

    if (hal_data->comp_free_ptr != 0) {
        p = SHMPTR(hal_data->comp_free_ptr);
        hal_data->comp_free_ptr = p->next_ptr;
        p->next_ptr = 0;
    } else {
        p = shmalloc_dn(sizeof(hal_comp_t));
    }
    if (p) {
        p->next_ptr = 0;
        p->comp_id = 0;
        p->mem_id = 0;
        p->type = COMPONENT_TYPE_USER;
        p->shmem_base = 0;
        p->name[0] = '\0';
    }
    return p;
}

// -----------------------------------------------------------------------------
// free_comp_struct — 释放组件结构体
// -----------------------------------------------------------------------------

/// \brief 释放组件结构体至空闲链表，同时级联释放该组件拥有的所有函数和参数结构体。
///
/// 遍历函数链表和参数链表，将属于此组件的条目摘除并释放，
/// 最后将组件自身归还空闲链表。
///
/// \param comp 要释放的组件指针。
void free_comp_struct(hal_comp_t *comp) {
    rtapi_intptr_t *prev, next;
    hal_funct_t *funct;
    hal_param_t *param;

    // 从函数链表中摘除属于此组件的函数条目并释放。
    prev = &(hal_data->funct_list_ptr);
    next = *prev;
    while (next != 0) {
        funct = SHMPTR(next);
        if (SHMPTR(funct->owner_ptr) == comp) {
            *prev = funct->next_ptr;
            free_funct_struct(funct);
        } else {
            prev = &(funct->next_ptr);
        }
        next = *prev;
    }

    // 从参数链表中摘除属于此组件的参数条目并释放。
    prev = &(hal_data->param_list_ptr);
    next = *prev;
    while (next != 0) {
        param = SHMPTR(next);
        if (SHMPTR(param->owner_ptr) == comp) {
            *prev = param->next_ptr;
            free_param_struct(param);
        } else {
            prev = &(param->next_ptr);
        }
        next = *prev;
    }

    // 重置组件字段并归还空闲链表。
    comp->comp_id = 0;
    comp->mem_id = 0;
    comp->type = COMPONENT_TYPE_USER;
    comp->shmem_base = 0;
    comp->name[0] = '\0';
    comp->next_ptr = hal_data->comp_free_ptr;
    hal_data->comp_free_ptr = SHMOFF(comp);
}

// -----------------------------------------------------------------------------
// 内部查找函数
// -----------------------------------------------------------------------------

/// \brief 按名称查找组件。
/// \param name 组件名称。
/// \return 匹配的组件指针；未找到返回 nullptr。
hal_comp_t *halpr_find_comp_by_name(const char *name) {
    int next;
    hal_comp_t *comp;

    next = hal_data->comp_list_ptr;
    while (next != 0) {
        comp = SHMPTR(next);
        if (strcmp(comp->name, name) == 0) {
            return comp;
        }
        next = comp->next_ptr;
    }
    return 0;
}

/// \brief 按 ID 查找组件。
/// \param id 组件 ID。
/// \return 匹配的组件指针；未找到返回 nullptr。
hal_comp_t *halpr_find_comp_by_id(int id) {
    int next;
    hal_comp_t *comp;

    next = hal_data->comp_list_ptr;
    while (next != 0) {
        comp = SHMPTR(next);
        if (comp->comp_id == id) {
            return comp;
        }
        next = comp->next_ptr;
    }
    return 0;
}
