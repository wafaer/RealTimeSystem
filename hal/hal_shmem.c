/// \file hal_shmem.c
///
/// HAL 共享内存基础设施。
///
/// 两大职责：
/// 1. 指针碰撞分配器（shmalloc_up / shmalloc_dn），从 HAL 共享内存段的
///    两端分别向中间分配对齐块。
/// 2. 双向链表操作原语（hal_list_t），所有操作均使用共享内存偏移量而非裸指针，
///    因为同一段内存在不同进程中可能映射到不同地址。

#include "hal_priv.h"

// -----------------------------------------------------------------------------
// 共享内存分配器
// -----------------------------------------------------------------------------

/// \brief 从共享内存池底部向上分配对齐块。
///
/// 按 16/8/4/2/1 字节边界对齐，将 shmem_bot 上边界推进。
///
/// \param size 要分配的字节数。
/// \return 分配到的内存指针；池空间不足返回 nullptr。
void *shmalloc_up(long int size) {
    long int tmp_bot;
    void *retval;

    tmp_bot = hal_data->shmem_bot;

    // 按请求大小进行对齐（16 / 8 / 4 / 2 / 1 字节）。
    if (size >= 16) {
        tmp_bot = (tmp_bot + 15) & (~15);
    } else if (size >= 8) {
        tmp_bot = (tmp_bot + 7) & (~7);
    } else if (size >= 4) {
        tmp_bot = (tmp_bot + 3) & (~3);
    } else if (size == 2) {
        tmp_bot = (tmp_bot + 1) & (~1);
    }

    if ((hal_data->shmem_top - tmp_bot) < size) {
        return 0;
    }

    retval = SHMPTR(tmp_bot);
    hal_data->shmem_bot = tmp_bot + size;
    hal_data->shmem_avail = hal_data->shmem_top - hal_data->shmem_bot;
    return retval;
}

/// \brief 从共享内存池顶部向下分配对齐块。
///
/// 从 shmem_top 向低地址推进，按 16/8/4/2/1 字节边界对齐。
/// 适用于大块一次性分配，与 shmalloc_up 互补。
///
/// \param size 要分配的字节数。
/// \return 分配到的内存指针；池空间不足返回 nullptr。
void *shmalloc_dn(long int size) {
    long int tmp_top;
    void *retval;

    tmp_top = hal_data->shmem_top - size;

    // 按请求大小进行对齐（16 / 8 / 4 / 2 / 1 字节）。
    if (size >= 16) {
        tmp_top &= (~15);
    } else if (size >= 8) {
        tmp_top &= (~7);
    } else if (size >= 4) {
        tmp_top &= (~3);
    } else if (size == 2) {
        tmp_top &= (~1);
    }

    if (tmp_top < hal_data->shmem_bot) {
        return 0;
    }

    retval = SHMPTR(tmp_top);
    hal_data->shmem_top = tmp_top;
    hal_data->shmem_avail = hal_data->shmem_top - hal_data->shmem_bot;
    return retval;
}

// -----------------------------------------------------------------------------
// 双向链表操作原语
// -----------------------------------------------------------------------------

/// \brief 获取链表中给定节点的前一个节点。
/// \param entry 当前节点。
/// \return 前一个节点的指针，到达链表尾部时为 nullptr。
hal_list_t *list_prev(hal_list_t *entry) {
    return SHMPTR(entry->prev);
}

/// \brief 获取链表中给定节点的下一个节点。
/// \param entry 当前节点。
/// \return 下一个节点的指针，到达链表尾部时为 nullptr。
hal_list_t *list_next(hal_list_t *entry) {
    return SHMPTR(entry->next);
}

/// \brief 在指定节点之后插入新节点（双向链表）。
///
/// 插入后顺序：prev -> entry -> 原 prev->next。
///
/// \param entry 要插入的节点。
/// \param prev  插入位置之后的节点。
void list_add_after(hal_list_t *entry, hal_list_t *prev) {
    int entry_n, prev_n, next_n;
    hal_list_t *next;

    entry_n = SHMOFF(entry);
    prev_n = SHMOFF(prev);
    next_n = prev->next;
    next = SHMPTR(next_n);

    entry->next = next_n;
    entry->prev = prev_n;
    prev->next = entry_n;
    next->prev = entry_n;
}

/// \brief 在指定节点之前插入新节点（双向链表）。
///
/// 插入后顺序：原 next->prev -> entry -> next。
///
/// \param entry 要插入的节点。
/// \param next  插入位置之前的节点。
void list_add_before(hal_list_t *entry, hal_list_t *next) {
    int entry_n, prev_n, next_n;
    hal_list_t *prev;

    entry_n = SHMOFF(entry);
    next_n = SHMOFF(next);
    prev_n = next->prev;
    prev = SHMPTR(prev_n);

    entry->next = next_n;
    entry->prev = prev_n;
    prev->next = entry_n;
    next->prev = entry_n;
}

/// \brief 从链表中摘除指定节点。
///
/// 摘除后 entry 的 next/prev 均自指（形成孤立节点），便于后续复用或释放。
///
/// \param entry 要摘除的节点。
/// \return 被摘除节点的下一个节点（用于调用者继续遍历）。
hal_list_t *list_remove_entry(hal_list_t *entry) {
    int entry_n;
    hal_list_t *prev, *next;

    entry_n = SHMOFF(entry);
    prev = SHMPTR(entry->prev);
    next = SHMPTR(entry->next);

    prev->next = entry->next;
    next->prev = entry->prev;

    // 自指，使节点孤立但不破坏其内部结构。
    entry->next = entry_n;
    entry->prev = entry_n;

    return next;
}

/// \brief 初始化链表节点为空节点（next 和 prev 均指向自身）。
///
/// 调用后节点成为一个仅含自身的循环链表，可作为链表头（list_root）使用。
///
/// \param entry 要初始化的节点。
void list_init_entry(hal_list_t *entry) {
    int entry_n = SHMOFF(entry);
    entry->next = entry_n;
    entry->prev = entry_n;
}
