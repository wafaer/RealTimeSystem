/* HAL real-time execution — exported functions and the threads that run
 * them.
 *
 * Responsibilities:
 *   - Function registry: hal_export_funct / hal_export_functf — sorted
 *     insertion into hal_data->funct_list_ptr, auto-creation of .time pin
 *     and .tmax / .tmax-increased params.
 *   - Thread lifecycle: hal_create_thread / hal_thread_delete — wraps a
 *     real-time task that wakes on a periodic clock.
 *   - Function-on-thread membership: hal_add_funct_to_thread /
 *     hal_del_funct_from_thread.
 *   - Global run/stop: hal_start_threads / hal_stop_threads.
 *   - thread_task: the per-thread main loop that walks the funct list each
 *     period and updates timing statistics.
 *
 * Allocators for funct / funct_entry / thread structures live here because
 * their lifetime is bound to this module's primitives.
 */

#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "hal.h"
#include "hal_priv.h"
#include "rtapi/rtapi_common.h"
#include "rtapi/rtapi.h"

static void thread_task(void *arg);

static int hal_export_functfv(void (*funct) (void *, long),
    void *arg, int uses_fp, int reentrant, int comp_id, const char *fmt, va_list ap)
{
    char name[HAL_NAME_LEN + 1];
    int sz;
    if(sz == -1 || sz > HAL_NAME_LEN) {
        rtapi_print_msg(RTAPI_MSG_ERR,
	    "hal_export_functfv: length %d too long for name starting '%s'\n",
	    sz, name);
        return -ENOMEM;
    }
    return hal_export_funct(name, funct, arg, uses_fp, reentrant, comp_id);
}

int hal_export_functf(void (*funct) (void *, long),
    void *arg, int uses_fp, int reentrant, int comp_id, const char *fmt, ...)
{
    va_list ap;
    int ret;
    va_start(ap, fmt);
    ret = hal_export_functfv(funct, arg, uses_fp, reentrant, comp_id, fmt, ap);
    va_end(ap);
    return ret;
}

int hal_export_funct(const char *name, void (*funct) (void *, long),
    void *arg, int uses_fp, int reentrant, int comp_id)
{
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
    if (hal_data->lock & HAL_LOCK_LOAD)  {
	return -EPERM;
    }
	if (hal_data->lock & HAL_LOCK_LOAD)  {
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
    if(comp->ready) {
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
	rt_snprintf(new_hal_funct_t->name, sizeof(new_hal_funct_t->name), "%s", name);
    /* search list for 'name' and insert new structure (alphabetical) */
    prev = &(hal_data->funct_list_ptr);
    next = *prev;
    while (1)
    {
		if (next == 0)
		{
		    new_hal_funct_t->next_ptr = next;
		    *prev = SHMOFF(new_hal_funct_t);
		    break;
		}
		fptr = SHMPTR(next);
		cmp = strcmp(fptr->name, new_hal_funct_t->name);
		if (cmp > 0)
		{
		    new_hal_funct_t->next_ptr = next;
		    *prev = SHMOFF(new_hal_funct_t);
		    break;
		}
		if (cmp == 0)
		{
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

	if (hal_pin_newf(HAL_S32, &(new_hal_funct_t->runtime), comp_id, "%s.time",name)) {
		rtapi_print_msg(RTAPI_MSG_ERR,
	   "HAL: ERROR: fail to create pin '%s.time'\n", name);
		return -EINVAL;
	}

	*(new_hal_funct_t->runtime) = 0;

	rt_snprintf(buf, sizeof(buf), "%s.tmax", name);
	new_hal_funct_t->maxtime = 0;
	hal_param_s32_new(buf, HAL_RW, HAL_S32, &(new_hal_funct_t->maxtime), comp_id);

	rt_snprintf(buf, sizeof(buf), "%s.tmax-increased", name);
	new_hal_funct_t->maxtime_increased = 0;
	hal_param_bit_new(buf, HAL_RO, HAL_BIT, &(new_hal_funct_t->maxtime_increased), comp_id);

    return 0;
}

int hal_create_thread(const char *name, unsigned long period_nsec, int uses_fp, int priority)
{
    int next, cmp, prev_priority;
    int retval, n;
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
    next = hal_data->thread_list_ptr;
    while (next != 0)
    {
		tptr = SHMPTR(next);
		cmp = strcmp(tptr->name, name);
		if (cmp == 0)
		{
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
	rt_snprintf(new_hal_thread_t->name, sizeof(new_hal_thread_t->name), "%s", name);
    if (hal_data->thread_list_ptr == 0)
    {
    	curr_period = rtapi_clock_set_period(0);
    	if (curr_period == 0)
    	{
    		curr_period = rtapi_clock_set_period(period_nsec);
    		if (curr_period < 0) {
    			free_thread_struct(new_hal_thread_t);
    			rtapi_mutex_give(&(hal_data->mutex));
    			return -EINVAL;
    		}
    	}
    	/* make sure period <= desired period (allow 1% roundoff error) */
    	if (curr_period > (long)(period_nsec + (period_nsec / 100))) {
    		free_thread_struct(new_hal_thread_t);
    		rtapi_mutex_give(&(hal_data->mutex));
    		return -EINVAL;
    	}

		prev_priority = 0;
		prev_period = 0;
    }
	else
    {
		tptr = SHMPTR(hal_data->thread_list_ptr);
		prev_period = tptr->period;
		prev_priority = tptr->priority;
    }

    new_hal_thread_t->period = period_nsec;
	new_hal_thread_t->priority = priority;
    /* create main - owned by library module, not caller */
    retval = rtapi_task_new(thread_task, new_hal_thread_t, new_hal_thread_t->priority,
	lib_module_id, HAL_STACKSIZE, uses_fp);
    if (retval < 0)
    {
		free_thread_struct(new_hal_thread_t);
		rtapi_mutex_give(&(hal_data->mutex));
		return -EINVAL;
    }
    new_hal_thread_t->task_id = retval;
    retval = rtapi_task_start(new_hal_thread_t->task_id, new_hal_thread_t->period);
    if (retval < 0)
    {
		rtapi_task_delete(new_hal_thread_t->task_id);
		free_thread_struct(new_hal_thread_t);
		rtapi_mutex_give(&(hal_data->mutex));
		return -EINVAL;
    }
    new_hal_thread_t->next_ptr = hal_data->thread_list_ptr;
    hal_data->thread_list_ptr = SHMOFF(new_hal_thread_t);
    rtapi_mutex_give(&(hal_data->mutex));

	rt_snprintf(buf,sizeof(buf), HAL_PSEUDO_COMP_PREFIX"%s",new_hal_thread_t->name);
    new_hal_thread_t->comp_id = hal_init(buf);
    if (new_hal_thread_t->comp_id < 0) {
        return -EINVAL;
    }

	rt_snprintf(buf, sizeof(buf), "%s.tmax", new_hal_thread_t->name);
    new_hal_thread_t->maxtime = 0;
    if (hal_param_s32_new(buf,HAL_RW, HAL_S32, &(new_hal_thread_t->maxtime), new_hal_thread_t->comp_id))
    {
    	rtapi_print_msg(RTAPI_MSG_ERR,
		   "HAL: ERROR: fail to create param '%s.tmax'\n", new_hal_thread_t->name);
        return -EINVAL;
    }
	if (hal_pin_newf(HAL_S32, &(new_hal_thread_t->runtime), new_hal_thread_t->comp_id, "%s.time", new_hal_thread_t->name))
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
		   "HAL: ERROR: fail to create pin '%s.time'\n", new_hal_thread_t->name);
		return -EINVAL;
	}

	*(new_hal_thread_t->runtime) = 0;
	hal_ready(new_hal_thread_t->comp_id);

	return new_hal_thread_t->comp_id;
}

int hal_thread_delete(const char *name)
{
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

    // Guard: state must be ACTIVE or RUNNING to modify threads.
    if (hal_data->state != HAL_S_ACTIVE && hal_data->state != HAL_S_RUNNING) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: thread_delete in invalid state %d\n", hal_data->state);
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
    rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: thread '%s' not found\n",
	name);
    return -EINVAL;
}

int hal_add_funct_to_thread(const char *funct_name, const char *thread_name, int position)
{
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
    	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing function name\n");
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }

    if (thread_name == 0) {
    	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing thread name\n");
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
		"HAL: ERROR: function '%s' may only be added to one thread\n", funct_name);
	return -EINVAL;
    }
    thread = halpr_find_thread_by_name(thread_name);
    if (thread == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
    	rtapi_print_msg(RTAPI_MSG_ERR,"HAL: ERROR: thread '%s' not found\n", thread_name);
	return -EINVAL;
    }
    /* fp-using functs may not be added to a non-fp thread */
    if ((funct->uses_fp) && (!thread->uses_fp)) {
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }
    list_root = &(thread->funct_list);
    list_entry = list_root;
    n = 0;
    if (position > 0)
    {
		/* insertion is relative to start of list */
		while (++n < position)
		{
		    list_entry = list_next(list_entry);
		    if (list_entry == list_root)
		    {
				rtapi_mutex_give(&(hal_data->mutex));
	    			rtapi_print_msg(RTAPI_MSG_ERR,
					"HAL: ERROR: position '%d' is too high\n", position);
				return -EINVAL;
		    }
		}
    } else
    {
		/* insertion is relative to end of list */
		while (--n > position)
		{
		    list_entry = list_prev(list_entry);
		    if (list_entry == list_root)
		    {
				rtapi_mutex_give(&(hal_data->mutex));
	    			rtapi_print_msg(RTAPI_MSG_ERR,
					"HAL: ERROR: position '%d' is too low\n", position);
				return -EINVAL;
		    }
		}
		/* want to insert before list_entry, so back up one more step */
		list_entry = list_prev(list_entry);
    }

    funct_entry = alloc_funct_entry_struct();
    if (funct_entry == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
	return -ENOMEM;
    }
    funct_entry->funct_ptr = SHMOFF(funct);
    funct_entry->arg = funct->arg;
    funct_entry->funct = funct->funct;
    list_add_after((hal_list_t *) funct_entry, list_entry);
    funct->users++;
    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

int hal_del_funct_from_thread(const char *funct_name, const char *thread_name)
{
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
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing function name\n");
	return -EINVAL;
    }
    if (thread_name == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing thread name\n");
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
		"HAL: ERROR: thread '%s' doesn't use %s\n", thread_name,
		funct_name);
	    return -EINVAL;
	}
	funct_entry = (hal_funct_entry_t *) list_entry;
	if (SHMPTR(funct_entry->funct_ptr) == funct) {
	    list_remove_entry(list_entry);
	    free_funct_entry_struct(funct_entry);
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 0;
	}
	list_entry = list_next(list_entry);
    }
}

// ============================================================================
// hal_start_threads — transitions the HAL into RUNNING state.
//
// Idempotent: if already RUNNING, returns -EALREADY.
// Only callable from ACTIVE state.
// ============================================================================
int hal_start_threads(void)
{
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

    // State gate: only transition from ACTIVE -> RUNNING.
    if (hal_data->state == HAL_S_RUNNING) {
	return -EALREADY;
    }
    if (hal_data->state != HAL_S_ACTIVE) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: start_threads in invalid state %d\n", hal_data->state);
	return -EINVAL;
    }

    hal_data->threads_running = 1;
    hal_data->state = HAL_S_RUNNING;
    return 0;
}

// ============================================================================
// hal_stop_threads — transitions the HAL back to ACTIVE state.
//
// Idempotent: if already ACTIVE, returns -EALREADY.
// Only callable from RUNNING state.
// ============================================================================
int hal_stop_threads(void)
{
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

    // State gate: only transition from RUNNING -> ACTIVE.
    if (hal_data->state == HAL_S_ACTIVE) {
	return -EALREADY;
    }
    if (hal_data->state != HAL_S_RUNNING) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: stop_threads in invalid state %d\n", hal_data->state);
	return -EINVAL;
    }

    hal_data->threads_running = 0;
    hal_data->state = HAL_S_ACTIVE;
    rtapi_print_msg(RTAPI_MSG_DBG, "HAL: threads stopped\n");
    return 0;
}

/* this is the main function that implements threads in realtime */
static void thread_task(void *arg)
{
    hal_thread_t *thread;
    hal_funct_t *funct;
    hal_funct_entry_t *funct_root, *funct_entry;
    long long int start_time, end_time;
    long long int thread_start_time;

    thread = arg;

    while (1)
    {
		if (hal_data->threads_running > 0)
		{
		    funct_root = (hal_funct_entry_t *) & (thread->funct_list);
		    funct_entry = SHMPTR(funct_root->links.next);

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
		    while (funct_entry != funct_root)
		    {
		    	func_count++;
				funct_entry->funct(funct_entry->arg, thread->period);
				end_time = rtapi_get_clocks();
		    	if (funct_entry->funct_ptr == 0) {
		    		rtapi_print_msg(RTAPI_MSG_ERR, "Thread %s: funct_entry %d has NULL funct_ptr\n", thread->name, func_count);
		    		break;
		    	}
				funct = SHMPTR(funct_entry->funct_ptr);
		    	if (funct == NULL) {
		    		rtapi_print_msg(RTAPI_MSG_ERR,
						"Thread %s: failed to convert funct_ptr %lu to pointer\n",
						thread->name, funct_entry->funct_ptr);
		    		break;
		    	}
		    	if (funct->runtime == NULL) {
		    		rtapi_print_msg(RTAPI_MSG_ERR,
						"Thread %s: Function '%s' has NULL runtime pointer!\n",
						thread->name, funct->name ? funct->name : "unknown");
		    		funct_entry = SHMPTR(funct_entry->links.next);
		    		continue;
		    	}

				if ( *(funct->runtime) > funct->maxtime) {
				    funct->maxtime = *(funct->runtime);
				    funct->maxtime_increased = 1;
				} else {
				    funct->maxtime_increased = 0;
				}
				funct_entry = SHMPTR(funct_entry->links.next);
				start_time = end_time;
		    }
		    *(thread->runtime) = (hal_s32_t)(end_time - thread_start_time);
		    if ( *(thread->runtime) > thread->maxtime) {
		        thread->maxtime = *(thread->runtime);
		    }
		}
		rtapi_wait();
    }
}

hal_funct_t *alloc_funct_struct(void)
{
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

hal_funct_entry_t *alloc_funct_entry_struct(void)
{
    hal_list_t *freelist, *l;
    hal_funct_entry_t *p;

    freelist = &(hal_data->funct_entry_free);
    l = list_next(freelist);
    if (l != freelist) {
	list_remove_entry(l);
	p = (hal_funct_entry_t *) l;
    } else {
	p = shmalloc_dn(sizeof(hal_funct_entry_t));
	l = (hal_list_t *) p;
	list_init_entry(l);
    }
    if (p) {
	p->funct_ptr = 0;
	p->arg = 0;
	p->funct = 0;
    }
    return p;
}

hal_thread_t *alloc_thread_struct(void)
{
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

void free_funct_struct(hal_funct_t *funct)
{
    int next_thread;
    hal_thread_t *thread;
    hal_list_t *list_root, *list_entry;
    hal_funct_entry_t *funct_entry;

    if (funct->users > 0) {
	/* threads still reference this funct — purge their entries first */
	next_thread = hal_data->thread_list_ptr;
	while (next_thread != 0) {
	    thread = SHMPTR(next_thread);
	    list_root = &(thread->funct_list);
	    list_entry = list_next(list_root);
	    while (list_entry != list_root) {
		funct_entry = (hal_funct_entry_t *) list_entry;
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

void free_funct_entry_struct(hal_funct_entry_t *funct_entry)
{
    hal_funct_t *funct;

    if (funct_entry->funct_ptr > 0) {
	funct = SHMPTR(funct_entry->funct_ptr);
	funct->users--;
    }
    funct_entry->funct_ptr = 0;
    funct_entry->arg = 0;
    funct_entry->funct = 0;
    list_add_after((hal_list_t *) funct_entry, &(hal_data->funct_entry_free));
}

void free_thread_struct(hal_thread_t *thread)
{
    hal_funct_entry_t *funct_entry;
    hal_list_t *list_root, *list_entry;

    /* if we're deleting a thread, we need to stop all threads */
    hal_data->threads_running = 0;
    rtapi_task_pause(thread->task_id);
    rtapi_task_delete(thread->task_id);
    thread->uses_fp = 0;
    thread->period = 0;
    thread->priority = 0;
    thread->task_id = 0;
    list_root = &(thread->funct_list);
    list_entry = list_next(list_root);
    while (list_entry != list_root) {
	funct_entry = (hal_funct_entry_t *) list_entry;
	list_entry = list_remove_entry(list_entry);
	free_funct_entry_struct(funct_entry);
    }

    thread->name[0] = '\0';
    thread->next_ptr = hal_data->thread_free_ptr;
    hal_data->thread_free_ptr = SHMOFF(thread);
}

hal_funct_t *halpr_find_funct_by_name(const char *name)
{
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

hal_thread_t *halpr_find_thread_by_name(const char *name)
{
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
