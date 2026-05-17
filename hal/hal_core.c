/* HAL core management — subsystem lifecycle, component registry, and the
 * global lock level.
 *
 * Responsibilities:
 *   - hal_app_main / hal_app_exit: bring the HAL shared-memory subsystem up
 *     and tear it down, including reclaiming orphan segments left by a
 *     crashed previous process.
 *   - hal_init / hal_exit / hal_ready / ...: per-component registration and
 *     state.
 *   - hal_set_lock / hal_get_lock: coarse-grained access control over which
 *     HAL operations are allowed at runtime.
 *
 * The component allocator (halpr_alloc_comp_struct / free_comp_struct) lives
 * here as well, since the component is the "owner" of pins, params and
 * functs and tearing one down must reach into those sub-registries.
 */

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

char *hal_shmem_base = 0;
hal_data_t *hal_data = 0;
int lib_module_id = -1;

static int ref_cnt = 0;
static int lib_mem_id = 0;

static void reclaim_orphan_hal_shm(void);

int hal_init(const char *name)
{
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

	if(!lib_mem_id)
	{
		// Guard: refuse to re-initialise after the HAL has been torn down.
		if (hal_data && hal_data->state == HAL_S_DESTROYED) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"HAL: ERROR: hal_init called after destruction\n");
			return -EINVAL;
		}
		rt_snprintf(rtapi_name, RTAPI_NAME_LEN, "HAL_LIB_%d", (int)getpid());
		lib_module_id = rtapi_init(rtapi_name);
		if (lib_module_id < 0)
		{
			return -EINVAL;
		}

		/* If a previous process crashed and left a HAL shm segment behind,
		 * destroy it before requesting a fresh one. */
		reclaim_orphan_hal_shm();

		/* get HAL shared memory block from RTAPI */
		lib_mem_id = rtapi_shmem_new(HAL_KEY, lib_module_id, HAL_SIZE);
		if (lib_mem_id < 0) {
			rtapi_exit(lib_module_id);
			return -EINVAL;
		}
		/* get address of shared memory area */
		retval = rtapi_shmem_getptr(lib_mem_id, &mem);
		if (retval < 0) {
			rtapi_exit(lib_module_id);
			return -EINVAL;
		}
		/* set up internal pointers to shared mem and data structure */
		hal_shmem_base = (char *) mem;
		hal_data = (hal_data_t *) mem;
		/* perform a global init if needed */
		retval = init_hal_data();
		if ( retval ) {
			rtapi_exit(lib_module_id);
			return -EINVAL;
		}
	}

	rt_snprintf(rtapi_name, RTAPI_NAME_LEN, "HAL_%s", name);
	rt_snprintf(hal_name, sizeof(hal_name), "%s", name);

    /* do RTAPI init */
    comp_id = rtapi_init(rtapi_name);
    if (comp_id < 0) {
	return -EINVAL;
    }
    /* get mutex before manipulating the shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* make sure name is unique in the system */
    if (halpr_find_comp_by_name(hal_name) != 0) {
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_exit(comp_id);
	return -EINVAL;
    }
    /* allocate a new component structure */
    comp = halpr_alloc_comp_struct();
    if (comp == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_exit(comp_id);
	return -ENOMEM;
    }
    /* initialize the structure */
    comp->comp_id = comp_id;
	comp->type = COMPONENT_TYPE_REALTIME;
	comp->pid = 0;
    comp->ready = 0;
    comp->shmem_base = hal_shmem_base;
    comp->insmod_args = 0;
    /* insert new structure at head of list */
    comp->next_ptr = hal_data->comp_list_ptr;
    hal_data->comp_list_ptr = SHMOFF(comp);
    rtapi_mutex_give(&(hal_data->mutex));
	comp->type = COMPONENT_TYPE_REALTIME;
	comp->pid = 0;
    ref_cnt ++;
    return comp_id;
}

int hal_exit(int comp_id)
{
    rtapi_intptr_t *prev, next;
    hal_comp_t *comp;
    char name[HAL_NAME_LEN + 1];

    if (hal_data == 0) {
	return -EINVAL;
    }

    // Guard: refuse to operate on a destroyed HAL.
    if (hal_data->state == HAL_S_DESTROYED) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: hal_exit called after destruction\n");
	return -EALREADY;
    }

    rtapi_mutex_get(&(hal_data->mutex));
    /* search component list for 'comp_id' */
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
    /* found our component, unlink it from the list */
    *prev = comp->next_ptr;
	rt_snprintf(name, sizeof(name), "%s", comp->name);
    free_comp_struct(comp);

    rtapi_mutex_give(&(hal_data->mutex));
    --ref_cnt;

    rtapi_exit(comp_id);

    return 0;
}

void *hal_malloc(long int size)
{
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

int hal_set_constructor(int comp_id, constructor make)
{
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

int hal_set_unready(int comp_id)
{
    hal_comp_t *comp;
    rtapi_mutex_get(&(hal_data->mutex));
    comp = halpr_find_comp_by_id(comp_id);
    if (comp) { comp->ready = 0; }
    rtapi_mutex_give(&(hal_data->mutex));
    if (comp) {return 0;}
    rtapi_print_msg(RTAPI_MSG_ERR,
         "HAL: ERROR: hal_set_unready(): component %d not found\n", comp_id);
    return -EINVAL;
}

int hal_ready(int comp_id)
{
    int next;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));

    next = hal_data->comp_list_ptr;
    if (next == 0)
    {
		rtapi_mutex_give(&(hal_data->mutex));
		rtapi_print_msg(RTAPI_MSG_ERR,
		    "HAL: ERROR: component %d not found\n", comp_id);
		return -EINVAL;
    }

    comp = SHMPTR(next);
    while (comp->comp_id != comp_id)
    {
		next = comp->next_ptr;
		if (next == 0)
		{
		    rtapi_mutex_give(&(hal_data->mutex));
		    rtapi_print_msg(RTAPI_MSG_ERR,
			"HAL: ERROR: component %d not found\n", comp_id);
		    return -EINVAL;
		}
		comp = SHMPTR(next);
    }
    if(comp->ready > 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: Component '%s' already ready\n", comp->name);
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    comp->ready = 1;
    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

int hal_unready(int comp_id)
{
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
    if(comp->ready < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL: ERROR: Component '%s' already unready\n", comp->name);
        rtapi_mutex_give(&(hal_data->mutex));
        return -EINVAL;
    }
    comp->ready = 0;
    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

char *hal_comp_name(int comp_id)
{
    hal_comp_t *comp;
    char *result = NULL;
    rtapi_mutex_get(&(hal_data->mutex));
    comp = halpr_find_comp_by_id(comp_id);
    if(comp) result = comp->name;
    rtapi_mutex_give(&(hal_data->mutex));
    return result;
}

/***********************************************************************
*                      "LOCKING" FUNCTIONS                             *
************************************************************************/

int hal_set_lock(unsigned char lock_type)
{
    if (hal_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: set_lock called before init\n");
	return -EINVAL;
    }
    hal_data->lock = lock_type;
    return 0;
}

unsigned char hal_get_lock()
{
    if (hal_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: get_lock called before init\n");
	return -EINVAL;
    }
    return hal_data->lock;
}

// ============================================================================
// hal_app_main — full HAL subsystem initialisation.
//
// Lifecycle: UNINIT -> INITIALIZING -> ACTIVE.
// On any failure, previously acquired OS resources are rolled back.
// ============================================================================
int hal_app_main(void)
{
    int retval;
    void *mem;

    lib_module_id = rtapi_init("HAL_LIB");
    if (lib_module_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LIB: ERROR: rtapi init failed\n");
	return -EINVAL;
    }

    /* If a previous process crashed and left a HAL shm segment behind,
     * destroy it before requesting a fresh one. */
    reclaim_orphan_hal_shm();

    lib_mem_id = rtapi_shmem_new(HAL_KEY, lib_module_id, HAL_SIZE);
    if (lib_mem_id < 0)
    {
		rtapi_print_msg(RTAPI_MSG_ERR,"HAL_LIB: ERROR: could not open shared memory\n");
		rtapi_exit(lib_module_id);
		return -EINVAL;
    }

    // State: now INITIALIZING (shared memory obtained, but not yet initialised).

    retval = rtapi_shmem_getptr(lib_mem_id, &mem);
    if (retval < 0)
    {
		rtapi_print_msg(RTAPI_MSG_ERR,"HAL_LIB: ERROR: could not access shared memory\n");
		// Rollback: release the shared memory segment.
		rtapi_shmem_delete(lib_mem_id, lib_module_id);
		rtapi_exit(lib_module_id);
		return -EINVAL;
    }
    hal_shmem_base = (char *) mem;
    hal_data = (hal_data_t *) mem;
    // Mark as INITIALIZING before calling init_hal_data().
    hal_data->state = HAL_S_INITIALIZING;
    hal_data->error_code = 0;

    retval = init_hal_data();
    if ( retval )
    {
		rtapi_print_msg(RTAPI_MSG_ERR,"HAL_LIB: ERROR: could not init shared memory\n");
		// Rollback: release the shared memory segment.
		rtapi_shmem_delete(lib_mem_id, lib_module_id);
		rtapi_exit(lib_module_id);
		return -EINVAL;
    }

    rtapi_print_msg(RTAPI_MSG_DBG, "HAL_LIB: kernel lib installed successfully\n");
    return 0;
}

// ============================================================================
// hal_app_exit — tears down the entire HAL subsystem.
//
// Lifecycle: ACTIVE/RUNNING -> SHUTTING_DOWN -> DESTROYED.
// Idempotent: if already DESTROYED, returns 0.
// ============================================================================
void hal_app_exit(void)
{
    hal_thread_t *thread;

    if (hal_data == NULL) {
	return;
    }
    if (hal_data->state == HAL_S_DESTROYED) {
	return;
    }

    hal_data->state = HAL_S_SHUTTING_DOWN;

    rtapi_mutex_get(&(hal_data->mutex));
    /* must remove all threads before unloading this module */
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

// ---------------------------------------------------------------------------
// reclaim_orphan_hal_shm — best-effort cleanup of a pre-existing HAL shared
// memory segment whose creator process is no longer alive.
//
// Background: the HAL shmem segment is keyed by HAL_KEY and its lifetime is
// owned by the kernel.  If a previous process crashed (SIGKILL / SIGSEGV /
// power loss), the kernel keeps the segment around and a new instance will
// re-attach to the stale layout — in particular, to a stale linked list of
// pins/params and possibly a stuck spinlock byte in hal_data->mutex.
//
// This helper probes the segment via plain shmget/shmat, looks at hal_data->
// init_pid, and if that PID is no longer running, marks the segment for
// destruction with IPC_RMID so that the subsequent shmget(IPC_CREAT) creates
// a fresh one.  If the PID is still alive, we leave the segment alone (a
// second instance attaching is the legitimate use case).
// ---------------------------------------------------------------------------
static void reclaim_orphan_hal_shm(void)
{
	int id = shmget((key_t)HAL_KEY, 0, 0);
	if (id < 0) return;          /* nothing to reclaim */

	struct shmid_ds info;
	if (shmctl(id, IPC_STAT, &info) < 0) return;
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

int hal_get_state(void)
{
    if (hal_data == NULL)
	return HAL_S_UNINIT;
    return hal_data->state;
}

int hal_get_stats(hal_stats_t *stats)
{
    if (hal_data == NULL || stats == NULL)
	return -EINVAL;

    rtapi_mutex_get(&(hal_data->mutex));

    stats->state    = hal_data->state;
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

int init_hal_data(void)
{
    /* Attempt to acquire the mutex.  If it is already held, the previous
     * owner may have crashed while holding it.  Use a simple heuristic:
     * if the HAL state is still UNINIT or INITIALIZING (never progressed to
     * a running state), we treat the mutex as a stale remnant of a crashed
     * process and forcibly clear it. */
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

hal_comp_t *halpr_alloc_comp_struct(void)
{
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

void free_comp_struct(hal_comp_t *comp)
{
    rtapi_intptr_t *prev, next;
    hal_funct_t *funct;
    hal_param_t *param;

    /* search the function list for this component's functs */
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

    /* search the parameter list for this component's parameters */
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
    /* now we can delete the component itself */
    comp->comp_id = 0;
    comp->mem_id = 0;
    comp->type = COMPONENT_TYPE_USER;
    comp->shmem_base = 0;
    comp->name[0] = '\0';
    comp->next_ptr = hal_data->comp_free_ptr;
    hal_data->comp_free_ptr = SHMOFF(comp);
}

hal_comp_t *halpr_find_comp_by_name(const char *name)
{
	int next;
	hal_comp_t *comp;

	next = hal_data->comp_list_ptr;
	while (next != 0) {
		comp = SHMPTR(next);
		if (strcmp(comp->name, name) == 0)
		{
			return comp;
		}
		next = comp->next_ptr;
	}
	return 0;
}

hal_comp_t *halpr_find_comp_by_id(int id)
{
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
