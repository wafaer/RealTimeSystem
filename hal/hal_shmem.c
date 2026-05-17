/* HAL shared-memory infrastructure.
 *
 * Two responsibilities:
 *   1. Bump-pointer allocators (shmalloc_up / shmalloc_dn) that hand out
 *      aligned blocks from opposite ends of the HAL shmem segment.
 *   2. Doubly-linked list primitives that operate on hal_list_t — these
 *      use shmem offsets rather than raw pointers because the segment is
 *      mapped at potentially different addresses in different processes.
 */

#include "hal_priv.h"

void *shmalloc_up(long int size)
{
    long int tmp_bot;
    void *retval;

    /* deal with alignment requirements */
    tmp_bot = hal_data->shmem_bot;
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

void *shmalloc_dn(long int size)
{
    long int tmp_top;
    void *retval;

    tmp_top = hal_data->shmem_top - size;
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

hal_list_t *list_prev(hal_list_t *entry)
{
	return SHMPTR(entry->prev);
}

hal_list_t *list_next(hal_list_t *entry)
{
	return SHMPTR(entry->next);
}

void list_add_after(hal_list_t *entry, hal_list_t *prev)
{
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

void list_add_before(hal_list_t *entry, hal_list_t *next)
{
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

hal_list_t *list_remove_entry(hal_list_t *entry)
{
	int entry_n;
	hal_list_t *prev, *next;

	entry_n = SHMOFF(entry);
	prev = SHMPTR(entry->prev);
	next = SHMPTR(entry->next);
	prev->next = entry->next;
	next->prev = entry->prev;
	entry->next = entry_n;
	entry->prev = entry_n;
	return next;
}

void list_init_entry(hal_list_t *entry)
{
	int entry_n;

	entry_n = SHMOFF(entry);
	entry->next = entry_n;
	entry->prev = entry_n;
}
