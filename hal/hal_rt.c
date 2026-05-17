/// \file hal_rt.c
///
/// HAL 实时执行层 — 导出函数与线程管理。
///
/// 负责：
/// - 函数注册：hal_export_funct / hal_export_functf — 按名称升序插入 funct_list_ptr，
///   自动创建 .time 引脚以及 .tmax / .tmax-increased 参数。
/// - 线程生命周期：hal_create_thread / hal_thread_delete — 包装一个在周期性时钟上
///   唤醒的实时任务。
/// - 函数-线程归属：hal_add_funct_to_thread / hal_del_funct_from_thread。
/// - 全局启停：hal_start_threads / hal_stop_threads。
/// - thread_task：每个线程的主循环，每周期遍历函数链表并更新计时统计。
///
/// funct / funct_entry / thread 结构体的分配器也在本模块，因为它们的生命周期
/// 与本模块的调度原语紧密绑定。

#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "hal.h"
#include "hal_priv.h"
#include "rtapi/rtapi_common.h"
#include "rtapi/rtapi.h"

/// 前向声明：线程主循环。
static void thread_task(void *arg);

// -----------------------------------------------------------------------------
// 函数导出 — va_list 封装
// -----------------------------------------------------------------------------

/// \brief hal_export_functf 的内部 va_list 实现。
///
/// 将格式字符串解包为名称后调用 hal_export_funct。
///
/// \param funct      函数入口。
/// \param arg        用户参数。
/// \param uses_fp    是否使用浮点单元。
/// \param reentrant  是否可重入。
/// \param comp_id    所属组件 ID。
/// \param fmt        printf 风格格式字符串。
/// \param ap         已解包的可变参数列表。
/// \return 成功返回 0，失败返回负值。
static int hal_export_functfv(void (*funct)(void *, long), void *arg,
                               int uses_fp, int reentrant, int comp_id,
                               const char *fmt, va_list ap) {
    char name[HAL_NAME_LEN + 1];
    int sz = rt_vsnprintf(name, sizeof(name), fmt, ap);
    if (sz == -1 || sz > HAL_NAME_LEN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "hal_export_functfv: length %d too long for name starting '%s'\n",
            sz, name);
        return -ENOMEM;
    }
    return hal_export_funct(name, funct, arg, uses_fp, reentrant, comp_id);
}

// -----------------------------------------------------------------------------
// 函数导出 — variadic 封装
// -----------------------------------------------------------------------------

/// \brief 导出函数至 HAL，支持格式化名称构造。
/// \param funct      函数入口。
/// \param arg        用户参数。
/// \param uses_fp    是否使用浮点单元。
/// \param reentrant  是否可重入。
/// \param comp_id    所属组件 ID。
/// \param fmt        printf 风格格式字符串。
/// \param ...        格式参数。
/// \return 成功返回 0，失败返回负值。
int hal_export_functf(void (*funct)(void *, long), void *arg,
                      int uses_fp, int reentrant, int comp_id,
                      const char *fmt, ...) {
    va_list ap;
    int ret;
    va_start(ap, fmt);
    ret = hal_export_functfv(funct, arg, uses_fp, reentrant, comp_id, fmt, ap);
    va_end(ap);
    return ret;
}

// -----------------------------------------------------------------------------
// hal_export_funct — 函数注册核心实现
// -----------------------------------------------------------------------------

/// \brief 将函数注册到 HAL 函数链表中，同时自动创建计时用引脚和参数。
///
/// 检查项：HAL 未初始化、名称过长、锁禁止、组件不存在、
/// 组件非实时、组件已就绪、重名冲突。
/// 注册成功后自动创建以下隐藏成员：
/// - `<name>.time`（S32 引脚）：最近一次执行耗时。
/// - `<name>.tmax`（S32 参数）：最长执行耗时。
/// - `<name>.tmax-increased`（BIT 参数，只读）：上次执行是否刷新了 tmax。
///
/// \param name      函数名称。
/// \param funct     函数入口。
/// \param arg       用户参数。
/// \param uses_fp   是否使用浮点单元。
/// \param reentrant 是否可重入。
/// \param comp_id   所属组件 ID。
/// \return 成功返回 0，失败返回负值。
int hal_export_funct(const char *name, void (*funct)(void *, long),
                     void *arg, int uses_fp, int reentrant, int comp_id) {
    rtapi_intptr_t *prev, next;
    int cmp;
    hal_funct_t *new_hal_funct_t, *fptr;
    hal_comp_t *comp;
    char buf[HAL_NAME_LEN + 1];

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: export_funct called before init\n");
        return -EINVAL;
    }
    if (strlen(name) > HAL_NAME_LEN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: function name '%s' is too long\n", name);
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_LOAD) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: export_funct called while HAL locked\n");
        return -EPERM;
    }

    rtapi_mutex_get(&(hal_data->mutex));
    comp = halpr_find_comp_by_id(comp_id);
    if (comp == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    if (comp->type == COMPONENT_TYPE_USER) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: component %d is not realtime\n", comp_id);
        return -EINVAL;
    }
    if (comp->ready) {
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }

    new_hal_funct_t = alloc_funct_struct();
    if (new_hal_funct_t == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: insufficient memory for function '%s'\n", name);
        rtapi_mutex_give(&(hal_data->mutex));
        return -ENOMEM;
    }

    new_hal_funct_t->uses_fp = uses_fp;
    new_hal_funct_t->owner_ptr = SHMOFF(comp);
    new_hal_funct_t->reentrant = reentrant;
    new_hal_funct_t->users = 0;
    new_hal_funct_t->arg = arg;
    new_hal_funct_t->funct = funct;
    rt_snprintf(new_hal_funct_t->name,
                sizeof(new_hal_funct_t->name), "%s", name);

    // 按名称升序遍历链表，找到插入位置后跳出。
    prev = &(hal_data->funct_list_ptr);
    next = *prev;
    while (1) {
        if (next == 0) {
            new_hal_funct_t->next_ptr = next;
            *prev = SHMOFF(new_hal_funct_t);
            break;
        }
        fptr = SHMPTR(next);
        cmp = strcmp(fptr->name, new_hal_funct_t->name);
        if (cmp > 0) {
            new_hal_funct_t->next_ptr = next;
            *prev = SHMOFF(new_hal_funct_t);
            break;
        }
        if (cmp == 0) {
            free_funct_struct(new_hal_funct_t);
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: duplicate function '%s'\n", name);
            return -EINVAL;
        }
        prev = &(fptr->next_ptr);
        next = *prev;
    }
    rtapi_mutex_give(&(hal_data->mutex));

    // 自动创建 .time 引脚（最近执行耗时）。
    if (hal_pin_newf(HAL_S32, &(new_hal_funct_t->runtime), comp_id,
                     "%s.time", name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: fail to create pin '%s.time'\n", name);
        return -EINVAL;
    }
    *(new_hal_funct_t->runtime) = 0;

    // 自动创建 .tmax 参数（最长执行耗时）。
    rt_snprintf(buf, sizeof(buf), "%s.tmax", name);
    new_hal_funct_t->maxtime = 0;
    hal_param_s32_new(buf, HAL_RW, HAL_S32,
                      &(new_hal_funct_t->maxtime), comp_id);

    // 自动创建 .tmax-increased 参数（只读，标识 tmax 是否被刷新）。
    rt_snprintf(buf, sizeof(buf), "%s.tmax-increased", name);
    new_hal_funct_t->maxtime_increased = 0;
    hal_param_bit_new(buf, HAL_RO, HAL_BIT,
                      &(new_hal_funct_t->maxtime_increased), comp_id);

    return 0;
}

// -----------------------------------------------------------------------------
// hal_create_thread — 创建线程
// -----------------------------------------------------------------------------

/// \brief 创建 HAL 实时线程，同时创建配套的计时引脚和参数。
///
/// 首次调用时自动初始化全局时钟周期（若已有线程则继承其周期）。
/// 线程以 `__<name>` 为名称注册为伪组件以纳入 HAL 管理。
///
/// 自动创建的隐藏成员：
/// - `<name>.time`（S32 引脚）：最近一次周期耗时。
/// - `<name>.tmax`（S32 参数）：最长周期耗时。
///
/// \param name           线程名称。
/// \param period_nsec    调度周期（纳秒），不能为零。
/// \param uses_fp        是否使用浮点单元。
/// \param priority       线程优先级。
/// \return 创建的伪组件 ID（>= 0），失败返回负值。
int hal_create_thread(const char *name, unsigned long period_nsec,
                      int uses_fp, int priority) {
    int next, cmp, prev_priority;
    int retval;
    hal_thread_t *new_hal_thread_t, *tptr;
    long prev_period, curr_period;
    char buf[HAL_NAME_LEN + 1];

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: create_thread called before init\n");
        return -EINVAL;
    }
    if (period_nsec == 0) {
        return -EINVAL;
    }
    if (strlen(name) > HAL_NAME_LEN) {
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_CONFIG) {
        return -EPERM;
    }

    rtapi_mutex_get(&(hal_data->mutex));

    // 检查线程名称唯一性。
    next = hal_data->thread_list_ptr;
    while (next != 0) {
        tptr = SHMPTR(next);
        cmp = strcmp(tptr->name, name);
        if (cmp == 0) {
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: duplicate thread name %s\n", name);
            return -EINVAL;
        }
        next = tptr->next_ptr;
    }

    new_hal_thread_t = alloc_thread_struct();
    if (new_hal_thread_t == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: insufficient memory to create thread\n");
        return -ENOMEM;
    }

    new_hal_thread_t->uses_fp = uses_fp;
    rt_snprintf(new_hal_thread_t->name,
                sizeof(new_hal_thread_t->name), "%s", name);

    // 首次创建线程时初始化时钟周期，后续线程继承已有周期。
    if (hal_data->thread_list_ptr == 0) {
        curr_period = rtapi_clock_set_period(0);
        if (curr_period == 0) {
            curr_period = rtapi_clock_set_period(period_nsec);
            if (curr_period < 0) {
                free_thread_struct(new_hal_thread_t);
                rtapi_mutex_give(&(hal_data->mutex));
                return -EINVAL;
            }
        }
        // 允许 1% 的时钟误差。
        if (curr_period > (long)(period_nsec + (period_nsec / 100))) {
            free_thread_struct(new_hal_thread_t);
            rtapi_mutex_give(&(hal_data->mutex));
            return -EINVAL;
        }
        prev_priority = 0;
        prev_period = 0;
    } else {
        tptr = SHMPTR(hal_data->thread_list_ptr);
        prev_period = tptr->period;
        prev_priority = tptr->priority;
    }

    new_hal_thread_t->period = period_nsec;
    new_hal_thread_t->priority = priority;

    // 创建由 HAL 库持有的实时任务（非调用方组件）。
    retval = rtapi_task_new(thread_task, new_hal_thread_t,
                            new_hal_thread_t->priority,
                            lib_module_id, HAL_STACKSIZE, uses_fp);
    if (retval < 0) {
        free_thread_struct(new_hal_thread_t);
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    new_hal_thread_t->task_id = retval;

    retval = rtapi_task_start(new_hal_thread_t->task_id,
                               new_hal_thread_t->period);
    if (retval < 0) {
        rtapi_task_delete(new_hal_thread_t->task_id);
        free_thread_struct(new_hal_thread_t);
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }

    new_hal_thread_t->next_ptr = hal_data->thread_list_ptr;
    hal_data->thread_list_ptr = SHMOFF(new_hal_thread_t);
    rtapi_mutex_give(&(hal_data->mutex));

    // 以伪组件方式注册线程（以 __ 为前缀，不暴露给用户）。
    rt_snprintf(buf, sizeof(buf),
                HAL_PSEUDO_COMP_PREFIX "%s", new_hal_thread_t->name);
    new_hal_thread_t->comp_id = hal_init(buf);
    if (new_hal_thread_t->comp_id < 0) {
        return -EINVAL;
    }

    // 创建 .tmax 参数（最长周期耗时，可读写调优）。
    rt_snprintf(buf, sizeof(buf), "%s.tmax", new_hal_thread_t->name);
    new_hal_thread_t->maxtime = 0;
    if (hal_param_s32_new(buf, HAL_RW, HAL_S32,
                          &(new_hal_thread_t->maxtime),
                          new_hal_thread_t->comp_id)) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: fail to create param '%s.tmax'\n",
            new_hal_thread_t->name);
        return -EINVAL;
    }

    // 创建 .time 引脚（最近一次周期耗时）。
    if (hal_pin_newf(HAL_S32, &(new_hal_thread_t->runtime),
                     new_hal_thread_t->comp_id,
                     "%s.time", new_hal_thread_t->name)) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: fail to create pin '%s.time'\n",
            new_hal_thread_t->name);
        return -EINVAL;
    }

    *(new_hal_thread_t->runtime) = 0;
    hal_ready(new_hal_thread_t->comp_id);

    return new_hal_thread_t->comp_id;
}

// -----------------------------------------------------------------------------
// hal_thread_delete — 删除线程
// -----------------------------------------------------------------------------

/// \brief 删除指定的 HAL 线程。
///
/// 须在 ACTIVE 或 RUNNING 状态下调用（HAL_S_ACTIVE / HAL_S_RUNNING）。
/// 会先通过 hal_exit 注销线程对应的伪组件。
///
/// \param name 线程名称。
/// \return 成功返回 0；HAL 未初始化、锁禁止、状态非法、线程不存在返回负值。
int hal_thread_delete(const char *name) {
    hal_thread_t *thread;
    rtapi_intptr_t *prev, next;

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: thread_delete called before init\n");
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_CONFIG) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: thread_delete called while HAL is locked\n");
        return -EPERM;
    }
    // 状态检查：仅允许在 ACTIVE 或 RUNNING 状态下修改线程。
    if (hal_data->state != HAL_S_ACTIVE &&
        hal_data->state != HAL_S_RUNNING) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: thread_delete in invalid state %d\n",
            hal_data->state);
        return -EINVAL;
    }

    rtapi_mutex_get(&(hal_data->mutex));
    prev = &(hal_data->thread_list_ptr);
    next = *prev;
    while (next != 0) {
        thread = SHMPTR(next);
        if (strcmp(thread->name, name) == 0) {
            if (thread->comp_id != 0) {
                hal_exit(thread->comp_id);
                thread->comp_id = 0;
            }
            *prev = thread->next_ptr;
            free_thread_struct(thread);
            rtapi_mutex_give(&(hal_data->mutex));
            return 0;
        }
        prev = &(thread->next_ptr);
        next = *prev;
    }
    rtapi_mutex_give(&(hal_data->mutex));
    rtapi_print_msg(RTAPI_MSG_ERR,
        "HAL: ERROR: thread '%s' not found\n", name);
    return -EINVAL;
}

// -----------------------------------------------------------------------------
// hal_add_funct_to_thread — 将函数加入线程调度队列
// -----------------------------------------------------------------------------

/// \brief 将函数追加到指定线程的调度序列中。
///
/// position > 0：从链表头部开始计数，插入到第 position 个位置。
/// position < 0：从链表尾部开始计数，插入到倒数第 |position| 个位置。
/// 链表头部（list_root）不计入位置。
///
/// 检查项：HAL 未初始化、锁禁止、position 为零、名称为空、
/// 函数不存在、函数不可重入但已加入其他线程、
/// 函数使用浮点但线程不使用浮点。
///
/// \param funct_name   函数名称。
/// \param thread_name  目标线程名称。
/// \param position     调度序列中的位置（正数从头数，负数从尾数）。
/// \return 成功返回 0，失败返回负值。
int hal_add_funct_to_thread(const char *funct_name,
                             const char *thread_name, int position) {
    hal_thread_t *thread;
    hal_funct_t *funct;
    hal_list_t *list_root, *list_entry;
    int n;
    hal_funct_entry_t *funct_entry;

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: add_funct called before init\n");
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_CONFIG) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: add_funct_to_thread called while HAL is locked\n");
        return -EPERM;
    }

    rtapi_mutex_get(&(hal_data->mutex));
    if (position == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: bad position: 0\n");
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    if (funct_name == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: missing function name\n");
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    if (thread_name == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: missing thread name\n");
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }

    funct = halpr_find_funct_by_name(funct_name);
    if (funct == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: function '%s' not found\n", funct_name);
        return -EINVAL;
    }
    if ((funct->users > 0) && (funct->reentrant == 0)) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: function '%s' may only be added to one thread\n",
            funct_name);
        return -EINVAL;
    }

    thread = halpr_find_thread_by_name(thread_name);
    if (thread == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: thread '%s' not found\n", thread_name);
        return -EINVAL;
    }

    // 浮点函数不可加入非浮点线程。
    if ((funct->uses_fp) && (!thread->uses_fp)) {
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }

    list_root = &(thread->funct_list);
    list_entry = list_root;
    n = 0;

    // 正数位置：从链表头部往后数 position-1 步。
    if (position > 0) {
        while (++n < position) {
            list_entry = list_next(list_entry);
            if (list_entry == list_root) {
                rtapi_mutex_give(&(hal_data->mutex));
                rtapi_print_msg(RTAPI_MSG_ERR,
                    "HAL: ERROR: position '%d' is too high\n", position);
                return -EINVAL;
            }
        }
    } else {
        // 负数位置：从链表尾部往前数 |position| 步。
        while (--n > position) {
            list_entry = list_prev(list_entry);
            if (list_entry == list_root) {
                rtapi_mutex_give(&(hal_data->mutex));
                rtapi_print_msg(RTAPI_MSG_ERR,
                    "HAL: ERROR: position '%d' is too low\n", position);
                return -EINVAL;
            }
        }
        list_entry = list_prev(list_entry);   // 插入到 list_entry 之前。
    }

    funct_entry = alloc_funct_entry_struct();
    if (funct_entry == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        return -ENOMEM;
    }

    funct_entry->funct_ptr = SHMOFF(funct);
    funct_entry->arg = funct->arg;
    funct_entry->funct = funct->funct;
    list_add_after((hal_list_t *)funct_entry, list_entry);
    funct->users++;

    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

// -----------------------------------------------------------------------------
// hal_del_funct_from_thread — 从线程调度队列移除函数
// -----------------------------------------------------------------------------

/// \brief 将函数从指定线程的调度序列中移除。
///
/// \param funct_name   函数名称。
/// \param thread_name  目标线程名称。
/// \return 成功返回 0，失败返回负值。
int hal_del_funct_from_thread(const char *funct_name,
                               const char *thread_name) {
    hal_thread_t *thread;
    hal_funct_t *funct;
    hal_list_t *list_root, *list_entry;
    hal_funct_entry_t *funct_entry;

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: del_funct called before init\n");
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_CONFIG) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: del_funct_from_thread called while HAL is locked\n");
        return -EPERM;
    }

    rtapi_mutex_get(&(hal_data->mutex));
    if (funct_name == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: missing function name\n");
        return -EINVAL;
    }
    if (thread_name == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: missing thread name\n");
        return -EINVAL;
    }

    funct = halpr_find_funct_by_name(funct_name);
    if (funct == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: function '%s' not found\n", funct_name);
        return -EINVAL;
    }
    if (funct->users == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: function '%s' is not in use\n", funct_name);
        return -EINVAL;
    }

    thread = halpr_find_thread_by_name(thread_name);
    if (thread == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: thread '%s' not found\n", thread_name);
        return -EINVAL;
    }

    list_root = &(thread->funct_list);
    list_entry = list_next(list_root);
    while (1) {
        if (list_entry == list_root) {
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: thread '%s' doesn't use %s\n",
                thread_name, funct_name);
            return -EINVAL;
        }
        funct_entry = (hal_funct_entry_t *)list_entry;
        if (SHMPTR(funct_entry->funct_ptr) == funct) {
            list_remove_entry(list_entry);
            free_funct_entry_struct(funct_entry);
            rtapi_mutex_give(&(hal_data->mutex));
            return 0;
        }
        list_entry = list_next(list_entry);
    }
}

// -----------------------------------------------------------------------------
// 全局启停
// -----------------------------------------------------------------------------

/// \brief 启动所有 HAL 线程，将 HAL 状态由 ACTIVE 转换为 RUNNING。
///
/// 幂等：若已处于 RUNNING 状态返回 -EALREADY。
///
/// \return 成功返回 0；HAL 未初始化、锁禁止、状态非法、已运行返回负值。
int hal_start_threads(void) {
    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: start_threads called before init\n");
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_RUN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: start_threads called while HAL is locked\n");
        return -EPERM;
    }
    if (hal_data->state == HAL_S_RUNNING) {
        return -EALREADY;
    }
    if (hal_data->state != HAL_S_ACTIVE) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: start_threads in invalid state %d\n",
            hal_data->state);
        return -EINVAL;
    }

    hal_data->threads_running = 1;
    hal_data->state = HAL_S_RUNNING;
    return 0;
}

/// \brief 停止所有 HAL 线程，将 HAL 状态由 RUNNING 转换回 ACTIVE。
///
/// 幂等：若已处于 ACTIVE 状态返回 -EALREADY。
///
/// \return 成功返回 0；HAL 未初始化、锁禁止、状态非法、已停止返回负值。
int hal_stop_threads(void) {
    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: stop_threads called before init\n");
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_RUN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: stop_threads called while HAL is locked\n");
        return -EPERM;
    }
    if (hal_data->state == HAL_S_ACTIVE) {
        return -EALREADY;
    }
    if (hal_data->state != HAL_S_RUNNING) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: stop_threads in invalid state %d\n",
            hal_data->state);
        return -EINVAL;
    }

    hal_data->threads_running = 0;
    hal_data->state = HAL_S_ACTIVE;
    rtapi_print_msg(RTAPI_MSG_DBG, "HAL: threads stopped\n");
    return 0;
}

// -----------------------------------------------------------------------------
// thread_task — 线程主循环
// -----------------------------------------------------------------------------

/// \brief HAL 线程的实时主循环（每线程一个）。
///
/// 当 hal_data->threads_running > 0 时，每周期执行以下步骤：
/// 1. 遍历线程的函数链表，逐个调用 funct_entry->funct(funct_entry->arg, period)。
/// 2. 调用完成后更新各函数的 .time 引脚和 .tmax / .tmax-increased 参数。
/// 3. 更新线程自身的 .time 引脚和 .tmax 参数。
/// 4. 调用 rtapi_wait() 等待下一周期。
///
/// 若函数链表为空，打印警告后继续等待而不执行任何函数。
///
/// \param arg 指向 hal_thread_t 结构体的指针。
static void thread_task(void *arg) {
    hal_thread_t *thread;
    hal_funct_t *funct;
    hal_funct_entry_t *funct_root, *funct_entry;
    long long int start_time, end_time;
    long long int thread_start_time;

    thread = arg;

    while (1) {
        if (hal_data->threads_running > 0) {
            funct_root = (hal_funct_entry_t *)&(thread->funct_list);
            funct_entry = SHMPTR(funct_root->links.next);

            // 空函数链表：打印警告后等待下一周期。
            if (funct_entry == funct_root) {
                rtapi_print_msg(RTAPI_MSG_WARN,
                    "Thread %s: function list is empty\n", thread->name);
                rtapi_wait();
                continue;
            }

            start_time = rtapi_get_clocks();
            end_time = start_time;
            thread_start_time = start_time;
            int func_count = 0;

            // 遍历函数链表，顺序执行每个函数。
            while (funct_entry != funct_root) {
                func_count++;
                funct_entry->funct(funct_entry->arg, thread->period);
                end_time = rtapi_get_clocks();

                if (funct_entry->funct_ptr == 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        "Thread %s: funct_entry %d has NULL funct_ptr\n",
                        thread->name, func_count);
                    break;
                }
                funct = SHMPTR(funct_entry->funct_ptr);
                if (funct == NULL) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        "Thread %s: failed to convert funct_ptr %lu to pointer\n",
                        thread->name, (unsigned long)funct_entry->funct_ptr);
                    break;
                }
                if (funct->runtime == NULL) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        "Thread %s: Function '%s' has NULL runtime pointer!\n",
                        thread->name,
                        funct->name ? funct->name : "unknown");
                    funct_entry = SHMPTR(funct_entry->links.next);
                    continue;
                }

                // 更新函数执行耗时统计。
                if (*(funct->runtime) > funct->maxtime) {
                    funct->maxtime = *(funct->runtime);
                    funct->maxtime_increased = 1;
                } else {
                    funct->maxtime_increased = 0;
                }

                funct_entry = SHMPTR(funct_entry->links.next);
                start_time = end_time;
            }

            // 更新线程周期耗时统计。
            *(thread->runtime) = (hal_s32_t)(end_time - thread_start_time);
            if (*(thread->runtime) > thread->maxtime) {
                thread->maxtime = *(thread->runtime);
            }
        }
        rtapi_wait();   // 等待下一个调度周期。
    }
}

// -----------------------------------------------------------------------------
// 结构体分配 — 函数
// -----------------------------------------------------------------------------

/// \brief 从空闲链表分配一个函数结构体；若空闲链表为空则从共享内存池底部分配。
/// \return 分配到的函数指针；共享内存耗尽返回 nullptr。
hal_funct_t *alloc_funct_struct(void) {
    hal_funct_t *p;

    if (hal_data->funct_free_ptr != 0) {
        p = SHMPTR(hal_data->funct_free_ptr);
        hal_data->funct_free_ptr = p->next_ptr;
        p->next_ptr = 0;
    } else {
        p = shmalloc_dn(sizeof(hal_funct_t));
    }
    if (p) {
        p->next_ptr = 0;
        p->uses_fp = 0;
        p->owner_ptr = 0;
        p->reentrant = 0;
        p->users = 0;
        p->arg = 0;
        p->funct = 0;
        p->name[0] = '\0';
    }
    return p;
}

/// \brief 从 funct_entry 空闲链表分配一个函数条目结构体。
///
/// 若空闲链表非空则复用链表节点，否则从共享内存池分配。
///
/// \return 分配到的函数条目指针；共享内存耗尽返回 nullptr。
hal_funct_entry_t *alloc_funct_entry_struct(void) {
    hal_list_t *freelist, *l;
    hal_funct_entry_t *p;

    freelist = &(hal_data->funct_entry_free);
    l = list_next(freelist);
    if (l != freelist) {
        list_remove_entry(l);
        p = (hal_funct_entry_t *)l;
    } else {
        p = shmalloc_dn(sizeof(hal_funct_entry_t));
        l = (hal_list_t *)p;
        list_init_entry(l);
    }
    if (p) {
        p->funct_ptr = 0;
        p->arg = 0;
        p->funct = 0;
    }
    return p;
}

// -----------------------------------------------------------------------------
// 结构体分配 — 线程
// -----------------------------------------------------------------------------

/// \brief 从空闲链表分配一个线程结构体；若空闲链表为空则从共享内存池底部分配。
/// \return 分配到的线程指针；共享内存耗尽返回 nullptr。
hal_thread_t *alloc_thread_struct(void) {
    hal_thread_t *p;

    if (hal_data->thread_free_ptr != 0) {
        p = SHMPTR(hal_data->thread_free_ptr);
        hal_data->thread_free_ptr = p->next_ptr;
        p->next_ptr = 0;
    } else {
        p = shmalloc_dn(sizeof(hal_thread_t));
    }
    if (p) {
        p->next_ptr = 0;
        p->uses_fp = 0;
        p->period = 0;
        p->priority = 0;
        p->task_id = 0;
        list_init_entry(&(p->funct_list));
        p->name[0] = '\0';
    }
    return p;
}

// -----------------------------------------------------------------------------
// 结构体释放 — 函数
// -----------------------------------------------------------------------------

/// \brief 释放函数结构体至空闲链表。
///
/// 若函数仍在被某个线程引用（users > 0），则先从所有线程的函数链表中
/// 级联摘除并释放对应的 funct_entry 条目。
///
/// \param funct 要释放的函数指针。
void free_funct_struct(hal_funct_t *funct) {
    int next_thread;
    hal_thread_t *thread;
    hal_list_t *list_root, *list_entry;
    hal_funct_entry_t *funct_entry;

    if (funct->users > 0) {
        // 遍历所有线程，摘除引用此函数的条目。
        next_thread = hal_data->thread_list_ptr;
        while (next_thread != 0) {
            thread = SHMPTR(next_thread);
            list_root = &(thread->funct_list);
            list_entry = list_next(list_root);
            while (list_entry != list_root) {
                funct_entry = (hal_funct_entry_t *)list_entry;
                if (SHMPTR(funct_entry->funct_ptr) == funct) {
                    list_entry = list_remove_entry(list_entry);
                    free_funct_entry_struct(funct_entry);
                } else {
                    list_entry = list_next(list_entry);
                }
            }
            next_thread = thread->next_ptr;
        }
    }

    funct->uses_fp = 0;
    funct->owner_ptr = 0;
    funct->reentrant = 0;
    funct->users = 0;
    funct->arg = 0;
    funct->funct = 0;
    funct->runtime = 0;
    funct->name[0] = '\0';
    funct->next_ptr = hal_data->funct_free_ptr;
    hal_data->funct_free_ptr = SHMOFF(funct);
}

/// \brief 释放函数条目结构体至空闲链表，同时递减关联函数的引用计数。
/// \param funct_entry 要释放的函数条目指针。
void free_funct_entry_struct(hal_funct_entry_t *funct_entry) {
    hal_funct_t *funct;

    if (funct_entry->funct_ptr > 0) {
        funct = SHMPTR(funct_entry->funct_ptr);
        funct->users--;
    }
    funct_entry->funct_ptr = 0;
    funct_entry->arg = 0;
    funct_entry->funct = 0;
    list_add_after((hal_list_t *)funct_entry,
                   &(hal_data->funct_entry_free));
}

// -----------------------------------------------------------------------------
// 结构体释放 — 线程
// -----------------------------------------------------------------------------

/// \brief 释放线程结构体至空闲链表，同时暂停并删除关联的实时任务。
///
/// 删除线程时会停止所有线程（threads_running = 0）并释放线程函数链表
/// 中的所有 funct_entry 条目。
///
/// \param thread 要释放的线程指针。
void free_thread_struct(hal_thread_t *thread) {
    hal_funct_entry_t *funct_entry;
    hal_list_t *list_root, *list_entry;

    // 停止所有线程并删除实时任务。
    hal_data->threads_running = 0;
    rtapi_task_pause(thread->task_id);
    rtapi_task_delete(thread->task_id);

    thread->uses_fp = 0;
    thread->period = 0;
    thread->priority = 0;
    thread->task_id = 0;

    // 释放线程函数链表中的所有条目。
    list_root = &(thread->funct_list);
    list_entry = list_next(list_root);
    while (list_entry != list_root) {
        funct_entry = (hal_funct_entry_t *)list_entry;
        list_entry = list_remove_entry(list_entry);
        free_funct_entry_struct(funct_entry);
    }

    thread->name[0] = '\0';
    thread->next_ptr = hal_data->thread_free_ptr;
    hal_data->thread_free_ptr = SHMOFF(thread);
}

// -----------------------------------------------------------------------------
// 内部查找函数
// -----------------------------------------------------------------------------

/// \brief 按名称查找导出的函数。
/// \param name 函数名称。
/// \return 匹配的函数指针；未找到返回 nullptr。
hal_funct_t *halpr_find_funct_by_name(const char *name) {
    int next;
    hal_funct_t *funct;

    next = hal_data->funct_list_ptr;
    while (next != 0) {
        funct = SHMPTR(next);
        if (strcmp(funct->name, name) == 0) {
            return funct;
        }
        next = funct->next_ptr;
    }
    return 0;
}

/// \brief 按名称查找线程。
/// \param name 线程名称。
/// \return 匹配的线程指针；未找到返回 nullptr。
hal_thread_t *halpr_find_thread_by_name(const char *name) {
    int next;
    hal_thread_t *thread;

    next = hal_data->thread_list_ptr;
    while (next != 0) {
        thread = SHMPTR(next);
        if (strcmp(thread->name, name) == 0) {
            return thread;
        }
        next = thread->next_ptr;
    }
    return 0;
}
