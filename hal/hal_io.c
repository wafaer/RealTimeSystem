/// \file hal_io.c
///
/// HAL 数据 I/O — 引脚与参数注册表。
///
/// 引脚和参数是 HAL 组件对外可见的数据点：
/// - 引脚（Pin）在组件之间传递信号并可相互连接；每个引脚持有一个隐藏的哑信号，
///   使未连接的引脚始终有合法的存储位置。
/// - 参数（Param）暴露内部状态供外部查看（RO）或运行时调优（RW）。
///
/// 本模块负责：
/// - hal_pin_new / hal_pin_newf / hal_pin_newfv：创建引脚，按名称升序插入 pin_list_ptr。
/// - hal_param_new 及各类型封装（bit / s32 / float）：创建参数，按名称升序插入 param_list_ptr。
/// - alloc_pin_struct / free_pin_struct / alloc_param_struct /
///   free_param_struct / free_oldname_struct：结构体后备存储管理。

#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "hal.h"
#include "hal_priv.h"
#include "rtapi/rtapi_common.h"
#include "rtapi/rtapi.h"

/// 前向声明：hal_pin_newf 的 va_list 版本。
static int hal_pin_newfv(hal_type_t type, void **data_ptr_addr,
                         int comp_id, const char *fmt, va_list ap);

// -----------------------------------------------------------------------------
// 参数注册 — 类型封装函数
// -----------------------------------------------------------------------------

/// \brief 注册布尔型 HAL 参数的便捷封装。
/// \param name   参数名称。
/// \param dir    读写方向。
/// \param type   参数类型（必须为 HAL_BIT）。
/// \param data_addr 参数数据地址。
/// \param comp_id 所属组件 ID。
/// \return 成功返回 0，失败返回负值。
int hal_param_bit_new(const char *name, hal_param_dir_t dir,
                       hal_type_t type, hal_bit_t *data_addr,
                       int comp_id) {
    return hal_param_new(name, type, dir, (void *)data_addr, comp_id);
}

/// \brief 注册有符号 32 位整型 HAL 参数的便捷封装。
/// \param name   参数名称。
/// \param dir    读写方向。
/// \param type   参数类型（必须为 HAL_S32）。
/// \param data_addr 参数数据地址。
/// \param comp_id 所属组件 ID。
/// \return 成功返回 0，失败返回负值。
int hal_param_s32_new(const char *name, hal_param_dir_t dir,
                       hal_type_t type, hal_s32_t *data_addr,
                       int comp_id) {
    return hal_param_new(name, type, dir, (void *)data_addr, comp_id);
}

/// \brief 注册浮点型 HAL 参数，支持格式化名称构造。
/// \param dir       读写方向。
/// \param data_addr 参数数据地址。
/// \param comp_id   所属组件 ID。
/// \param name      printf 风格格式字符串。
/// \param ...       格式参数。
/// \return 成功返回 0，失败返回负值。
int hal_param_float_newf(hal_param_dir_t dir, hal_float_t *data_addr,
                         int comp_id, const char *name, ...) {
    char buf[HAL_NAME_LEN + 1];
    va_list args;
    va_start(args, name);
    vsnprintf(buf, sizeof(buf), name, args);
    va_end(args);
    return hal_param_new(buf, HAL_FLOAT, dir, (void *)data_addr, comp_id);
}

// -----------------------------------------------------------------------------
// 引脚注册 — va_list 封装
// -----------------------------------------------------------------------------

/// \brief 注册 HAL 引脚，支持格式化名称构造（va_list 版本）。
/// \param type          引脚类型。
/// \param data_ptr_addr 输出参数，指向存放数据指针地址的变量。
/// \param comp_id       所属组件 ID。
/// \param fmt           printf 风格格式字符串。
/// \param ...           格式参数。
/// \return 成功返回 0，失败返回负值。
int hal_pin_newf(hal_type_t type, hal_s32_t **data_ptr_addr,
                 int comp_id, const char *fmt, ...) {
    va_list ap;
    int ret;
    va_start(ap, fmt);
    ret = hal_pin_newfv(type, (void **)data_ptr_addr, comp_id, fmt, ap);
    va_end(ap);
    return ret;
}

/// \brief hal_pin_newf 的内部 va_list 实现。
///
/// 将格式字符串解包为名称后调用 hal_pin_new。
///
/// \param type          引脚类型。
/// \param data_ptr_addr 数据指针地址输出变量。
/// \param comp_id       所属组件 ID。
/// \param fmt           printf 风格格式字符串。
/// \param ap            已解包的可变参数列表。
/// \return 成功返回 0，失败返回负值。
static int hal_pin_newfv(hal_type_t type, void **data_ptr_addr,
                         int comp_id, const char *fmt, va_list ap) {
    char name[HAL_NAME_LEN + 1];
    int sz = rt_vsnprintf(name, sizeof(name), fmt, ap);
    if (sz == -1 || sz > HAL_NAME_LEN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "hal_pin_newfv: length %d too long for name starting '%s'\n",
            sz, name);
        return -ENOMEM;
    }
    return hal_pin_new(name, type, data_ptr_addr, comp_id);
}

// -----------------------------------------------------------------------------
// hal_param_new — 参数注册核心实现
// -----------------------------------------------------------------------------

/// \brief 注册一个 HAL 参数，按名称升序插入参数链表。
///
/// 参数须在组件就绪（hal_ready）之前注册。函数内完成以下检查：
/// HAL 未初始化、类型非法、方向非法、名称过长、锁禁止、组件不存在、
/// data_addr 不在共享内存中、组件已就绪、重名冲突。
///
/// \param name      参数名称。
/// \param type      参数类型（HAL_BIT / HAL_FLOAT / HAL_S32 / HAL_U32 / HAL_S64 / HAL_U64）。
/// \param dir       读写方向（HAL_RO / HAL_RW）。
/// \param data_addr 参数值在共享内存中的地址。
/// \param comp_id   所属组件 ID。
/// \return 成功返回 0，失败返回负值。
int hal_param_new(const char *name, hal_type_t type, hal_param_dir_t dir,
                   void *data_addr, int comp_id) {
    rtapi_intptr_t *prev, next;
    int cmp;
    hal_param_t *new_hal_param_t, *ptr;
    hal_comp_t *comp;

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: param_new called before init\n");
        return -EINVAL;
    }

    if (type != HAL_BIT && type != HAL_FLOAT && type != HAL_S32 &&
        type != HAL_U32 && type != HAL_S64 && type != HAL_U64) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: pin type not one of HAL_BIT, HAL_FLOAT, HAL_S32, "
            "HAL_U32, HAL_S64 or HAL_U64\n");
        return -EINVAL;
    }

    if (dir != HAL_RO && dir != HAL_RW) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: param direction not one of HAL_RO, or HAL_RW\n");
        return -EINVAL;
    }

    if (strlen(name) > HAL_NAME_LEN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: parameter name '%s' is too long\n", name);
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_LOAD) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: param_new called while HAL locked\n");
        return -EPERM;
    }

    rtapi_mutex_get(&(hal_data->mutex));
    comp = halpr_find_comp_by_id(comp_id);
    if (comp == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: component %d not found\n", comp_id);
        return -EINVAL;
    }
    if (!SHMCHK(data_addr)) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: data_addr not in shared memory\n");
        return -EINVAL;
    }
    if (comp->ready) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: param_new called after hal_ready\n");
        return -EINVAL;
    }

    new_hal_param_t = alloc_param_struct();
    if (new_hal_param_t == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: insufficient memory for parameter '%s'\n", name);
        return -ENOMEM;
    }

    new_hal_param_t->owner_ptr = SHMOFF(comp);
    new_hal_param_t->data_ptr = SHMOFF(data_addr);
    new_hal_param_t->type = type;
    new_hal_param_t->dir = dir;
    rt_snprintf(new_hal_param_t->name, sizeof(new_hal_param_t->name), "%s", name);

    // 按名称升序遍历链表，找到插入位置（首次 cmp > 0 处，或到达链表尾部）。
    prev = &(hal_data->param_list_ptr);
    next = *prev;
    while (1) {
        if (next == 0) {
            new_hal_param_t->next_ptr = next;
            *prev = SHMOFF(new_hal_param_t);
            rtapi_mutex_give(&(hal_data->mutex));
            return 0;
        }
        ptr = SHMPTR(next);
        cmp = strcmp(ptr->name, new_hal_param_t->name);
        if (cmp > 0) {
            new_hal_param_t->next_ptr = next;
            *prev = SHMOFF(new_hal_param_t);
            rtapi_mutex_give(&(hal_data->mutex));
            return 0;
        }
        if (cmp == 0) {
            free_param_struct(new_hal_param_t);
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: duplicate parameter '%s'\n", name);
            return -EINVAL;
        }
        prev = &(ptr->next_ptr);
        next = *prev;
    }
}

// -----------------------------------------------------------------------------
// hal_pin_new — 引脚注册核心实现
// -----------------------------------------------------------------------------

/// \brief 注册一个 HAL 引脚，按名称升序插入引脚链表。
///
/// 引脚须在组件就绪之前注册。与参数不同，引脚拥有 dummysig 机制：
/// 未连接的引脚其 data_ptr 被初始化指向 dummysig，保证始终有合法存储。
///
/// 函数内检查项：HAL 未初始化、内存已初始化、类型非法、名称过长、
/// 锁禁止、组件不存在、data_ptr_addr 不在共享内存中、组件已就绪、重名冲突。
///
/// \param name           引脚名称。
/// \param type           引脚类型。
/// \param data_ptr_addr  输出参数，函数将把数据指针写入该地址。
/// \param comp_id         所属组件 ID。
/// \return 成功返回 0，失败返回负值。
int hal_pin_new(const char *name, hal_type_t type, void **data_ptr_addr,
                 int comp_id) {
    rtapi_intptr_t *prev, next;
    int cmp;
    hal_pin_t *new, *ptr;
    hal_comp_t *comp;

    if (hal_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: pin_new called before init\n");
        return -EINVAL;
    }
    if (*data_ptr_addr) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: pin_new(%s) called with already-initialized memory\n",
            name);
    }

    if (type != HAL_BIT && type != HAL_FLOAT && type != HAL_S32 &&
        type != HAL_U32 && type != HAL_S64 && type != HAL_U64 &&
        type != HAL_PORT) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: pin type not one of HAL_BIT, HAL_FLOAT, HAL_S32, "
            "HAL_U32, HAL_S64, HAL_U64 or HAL_PORT\n");
        return -EINVAL;
    }

    if (strlen(name) > HAL_NAME_LEN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: pin name '%s' is too long\n", name);
        return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_LOAD) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: pin_new called while HAL locked\n");
        return -EPERM;
    }

    rtapi_mutex_get(&(hal_data->mutex));
    comp = halpr_find_comp_by_id(comp_id);
    if (comp == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: component %d not found\n", comp_id);
        return -EINVAL;
    }
    if (!SHMCHK(data_ptr_addr)) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: data_ptr_addr not in shared memory\n");
        return -EINVAL;
    }
    if (comp->ready) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: pin_new called after hal_ready\n");
        return -EINVAL;
    }

    new = alloc_pin_struct();
    if (new == 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        rtapi_print_msg(RTAPI_MSG_ERR,
            "HAL: ERROR: insufficient memory for pin '%s'\n", name);
        return -ENOMEM;
    }

    new->data_ptr_addr = SHMOFF(data_ptr_addr);
    new->owner_ptr = SHMOFF(comp);
    new->type = type;
    new->signal = 0;
    memset(&new->dummysig, 0, sizeof(hal_data_u));
    rt_snprintf(new->name, sizeof(new->name), "%s", name);

    // 将 data_ptr 指向 dummysig（未连接时的默认存储位置）。
    *data_ptr_addr = (char *)comp->shmem_base + SHMOFF(&(new->dummysig));

    // 按名称升序插入链表。
    prev = &(hal_data->pin_list_ptr);
    next = *prev;
    while (1) {
        if (next == 0) {
            new->next_ptr = next;
            *prev = SHMOFF(new);
            rtapi_mutex_give(&(hal_data->mutex));
            return 0;
        }
        ptr = SHMPTR(next);
        cmp = strcmp(ptr->name, new->name);
        if (cmp > 0) {
            new->next_ptr = next;
            *prev = SHMOFF(new);
            rtapi_mutex_give(&(hal_data->mutex));
            return 0;
        }
        if (cmp == 0) {
            free_pin_struct(new);
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: duplicate variable '%s'\n", name);
            return -EINVAL;
        }
        prev = &(ptr->next_ptr);
        next = *prev;
    }
}

// -----------------------------------------------------------------------------
// 结构体分配 — 引脚
// -----------------------------------------------------------------------------

/// \brief 从空闲链表分配一个引脚结构体；若空闲链表为空则从共享内存池底部分配。
/// \return 分配到的引脚指针；共享内存耗尽返回 nullptr。
hal_pin_t *alloc_pin_struct(void) {
    hal_pin_t *p;

    if (hal_data->pin_free_ptr != 0) {
        p = SHMPTR(hal_data->pin_free_ptr);
        hal_data->pin_free_ptr = p->next_ptr;
        p->next_ptr = 0;
    } else {
        p = shmalloc_dn(sizeof(hal_pin_t));
    }
    if (p) {
        p->next_ptr = 0;
        p->data_ptr_addr = 0;
        p->owner_ptr = 0;
        p->type = 0;
        p->signal = 0;
        memset(&p->dummysig, 0, sizeof(hal_data_u));
        p->name[0] = '\0';
        p->oldname = 0;
    }
    return p;
}

/// \brief 释放引脚结构体至空闲链表，若有旧名称记录则一并释放。
/// \param pin 要释放的引脚指针。
void free_pin_struct(hal_pin_t *pin) {
    if (pin->oldname != 0) free_oldname_struct(SHMPTR(pin->oldname));
    pin->data_ptr_addr = 0;
    pin->owner_ptr = 0;
    pin->type = 0;
    pin->signal = 0;
    memset(&pin->dummysig, 0, sizeof(hal_data_u));
    pin->name[0] = '\0';
    pin->next_ptr = hal_data->pin_free_ptr;
    hal_data->pin_free_ptr = SHMOFF(pin);
}

// -----------------------------------------------------------------------------
// 结构体分配 — 参数
// -----------------------------------------------------------------------------

/// \brief 从空闲链表分配一个参数结构体；若空闲链表为空则从共享内存池底部分配。
/// \return 分配到的参数指针；共享内存耗尽返回 nullptr。
hal_param_t *alloc_param_struct(void) {
    hal_param_t *p;

    if (hal_data->param_free_ptr != 0) {
        p = SHMPTR(hal_data->param_free_ptr);
        hal_data->param_free_ptr = p->next_ptr;
        p->next_ptr = 0;
    } else {
        p = shmalloc_dn(sizeof(hal_param_t));
    }
    if (p) {
        p->next_ptr = 0;
        p->data_ptr = 0;
        p->owner_ptr = 0;
        p->type = 0;
        p->name[0] = '\0';
    }
    return p;
}

/// \brief 释放参数结构体至空闲链表，若有旧名称记录则一并释放。
/// \param p 要释放的参数指针。
void free_param_struct(hal_param_t *p) {
    if (p->oldname != 0) free_oldname_struct(SHMPTR(p->oldname));
    p->data_ptr = 0;
    p->owner_ptr = 0;
    p->type = 0;
    p->name[0] = '\0';
    p->next_ptr = hal_data->param_free_ptr;
    hal_data->param_free_ptr = SHMOFF(p);
}

// -----------------------------------------------------------------------------
// 结构体分配 — 旧名称
// -----------------------------------------------------------------------------

/// \brief 释放旧名称别名结构体至空闲链表。
/// \param oldname 要释放的 oldname 指针。
void free_oldname_struct(hal_oldname_t *oldname) {
    oldname->name[0] = '\0';
    oldname->next_ptr = hal_data->oldname_free_ptr;
    hal_data->oldname_free_ptr = SHMOFF(oldname);
}
