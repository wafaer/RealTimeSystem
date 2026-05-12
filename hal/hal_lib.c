//
// Created by Administrator on 2025/8/16.
//

#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <asm-generic/errno-base.h>

#include "hal.h"
#include "hal_priv.h"

#include "rtapi/rtapi_common.h"

#include "rtapi/rtapi.h"


char *hal_shmem_base = 0;
hal_data_t *hal_data = 0;
static int ref_cnt = 0;
static int lib_module_id = -1;
static int lib_mem_id = 0;

static int init_hal_data(void);
static void *shmalloc_up(long int size);
static void *shmalloc_dn(long int size);
hal_comp_t *halpr_alloc_comp_struct(void);
static void free_comp_struct(hal_comp_t * comp);
static hal_funct_t *alloc_funct_struct(void);
static void free_funct_struct(hal_funct_t * funct);
static void free_funct_entry_struct(hal_funct_entry_t * funct_entry);
static void thread_task(void *arg);
static void free_thread_struct(hal_thread_t * thread);
static hal_thread_t *alloc_thread_struct(void);
static hal_param_t *alloc_param_struct(void);
static hal_funct_entry_t *alloc_funct_entry_struct(void);
static void free_param_struct(hal_param_t * param);
static void free_oldname_struct(hal_oldname_t * oldname);
static hal_pin_t *alloc_pin_struct(void);
static void free_pin_struct(hal_pin_t * pin);
static int hal_pin_newfv(hal_type_t type, void ** data_ptr_addr, int comp_id, const char *fmt, va_list ap);

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
		rt_snprintf(rtapi_name, RTAPI_NAME_LEN, "HAL_LIB_%d", (int)getpid());
		lib_module_id = rtapi_init(rtapi_name);
		if (lib_module_id < 0)
		{
			return -EINVAL;
		}

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
	/* a component with this name already exists */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_exit(comp_id);
	return -EINVAL;
    }
    /* allocate a new component structure */
    comp = halpr_alloc_comp_struct();
    if (comp == 0) {
	/* couldn't allocate structure */
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
    /* done with list, release mutex */
    rtapi_mutex_give(&(hal_data->mutex));
    /* done */
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

    /* grab mutex before manipulating list */
    rtapi_mutex_get(&(hal_data->mutex));
    /* search component list for 'comp_id' */
    prev = &(hal_data->comp_list_ptr);
    next = *prev;
    if (next == 0) {
	/* list is empty - should never happen, but... */
	rtapi_mutex_give(&(hal_data->mutex));

	return -EINVAL;
    }
    comp = SHMPTR(next);
    while (comp->comp_id != comp_id) {
	/* not a match, try the next one */
	prev = &(comp->next_ptr);
	next = *prev;
	if (next == 0) {
	    /* reached end of list without finding component */
	    rtapi_mutex_give(&(hal_data->mutex));

	    return -EINVAL;
	}
	comp = SHMPTR(next);
    }
    /* found our component, unlink it from the list */
    *prev = comp->next_ptr;
	rt_snprintf(name, sizeof(name), "%s", comp->name);
    /* get rid of the component */
    free_comp_struct(comp);

    rtapi_mutex_give(&(hal_data->mutex));
    --ref_cnt;

    rtapi_exit(comp_id);

    return 0;
}

//共享内存分配机制
void *hal_malloc(long int size)
{
    void *retval;

    if (hal_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: hal_malloc called before init\n");
	return 0;
    }
    /* get the mutex */
    rtapi_mutex_get(&(hal_data->mutex));
    /* allocate memory */
    retval = shmalloc_up(size);
    /* release the mutex */
    rtapi_mutex_give(&(hal_data->mutex));
    /* check return value */
    if (retval == 0) {
	// rtapi_print_msg(RTAPI_MSG_DBG,"HAL: hal_malloc() can't allocate %ld bytes\n", size);
    }
    return retval;
}

//组件构造函数设置机制
int hal_set_constructor(int comp_id, constructor make) {
    int next;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));

    /* search component list for 'comp_id' */
    next = hal_data->comp_list_ptr;
    if (next == 0) {
	/* list is empty - should never happen, but... */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: component %d not found\n", comp_id);
	return -EINVAL;
    }

    comp = SHMPTR(next);
    while (comp->comp_id != comp_id) {
	/* not a match, try the next one */
	next = comp->next_ptr;
	if (next == 0) {
	    /* reached end of list without finding component */
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


int hal_set_unready(int comp_id) {
    hal_comp_t *comp;
    rtapi_mutex_get(&(hal_data->mutex));
	//通过ID查找组件
    comp = halpr_find_comp_by_id(comp_id);
	// 找到则设置ready=0
    if (comp) { comp->ready = 0; }
    rtapi_mutex_give(&(hal_data->mutex));
    if (comp) {return 0;}
    rtapi_print_msg(RTAPI_MSG_ERR,
         "HAL: ERROR: hal_set_unready(): component %d not found\n", comp_id);
    return -EINVAL;
}

//设置就绪态
int hal_ready(int comp_id) {
    int next;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));

    /* search component list for 'comp_id' */
    next = hal_data->comp_list_ptr;
    if (next == 0)
    {
		/* list is empty - should never happen, but... */
		rtapi_mutex_give(&(hal_data->mutex));
		rtapi_print_msg(RTAPI_MSG_ERR,
		    "HAL: ERROR: component %d not found\n", comp_id);
		return -EINVAL;
    }

    comp = SHMPTR(next);
    while (comp->comp_id != comp_id)
    {
		/* not a match, try the next one */
		next = comp->next_ptr;
		if (next == 0)
		{
		    /* reached end of list without finding component */
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

//设置未就绪态
int hal_unready(int comp_id) {
    int next;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));

    /* search component list for 'comp_id' */
    next = hal_data->comp_list_ptr;
    if (next == 0) {
	/* list is empty - should never happen, but... */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: component %d not found\n", comp_id);
	return -EINVAL;
    }

    comp = SHMPTR(next);
    while (comp->comp_id != comp_id) {
	/* not a match, try the next one */
	next = comp->next_ptr;
	if (next == 0) {
	    /* reached end of list without finding component */
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
/** The 'hal_set_lock()' function sets locking based on one of the
    locking types defined in hal.h
*/
int hal_set_lock(unsigned char lock_type) {
    if (hal_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: set_lock called before init\n");
	return -EINVAL;
    }
    hal_data->lock = lock_type;
    return 0;
}

/** The 'hal_get_lock()' function returns the current locking level
    locking types defined in hal.h
*/

unsigned char hal_get_lock() {
    if (hal_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: get_lock called before init\n");
	return -EINVAL;
    }
    return hal_data->lock;
}

//实现了HAL实时函数的动态注册机制
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

//用户接口函数
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
    hal_funct_t * new_hal_funct_t, *fptr;
    hal_comp_t *comp;
    char buf[HAL_NAME_LEN + 1];

    if (hal_data == 0) {
    	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: export_funct called before init\n");
	return -EINVAL;
    }
	// 名称长度验证
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

    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* validate comp_id */
	// 组件存在性
    comp = halpr_find_comp_by_id(comp_id);
    if (comp == 0) {
	/* bad comp_id */
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }
	// 实时性要求
    if (comp->type == COMPONENT_TYPE_USER) {
	/* not a realtime component */
	rtapi_mutex_give(&(hal_data->mutex));
    	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: component %d is not realtime\n", comp_id);
	return -EINVAL;
    }
	// ready状态检查
    if(comp->ready) {
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }

    /* allocate a new function structure */
    new_hal_funct_t = alloc_funct_struct();
    if (new_hal_funct_t == 0) {
	/* alloc failed */
    	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: insufficient memory for function '%s'\n", name);
	rtapi_mutex_give(&(hal_data->mutex));
	return -ENOMEM;
    }
    /* initialize the structure */
    new_hal_funct_t->uses_fp = uses_fp;
    new_hal_funct_t->owner_ptr = SHMOFF(comp);
    new_hal_funct_t->reentrant = reentrant;
    new_hal_funct_t->users = 0;
    new_hal_funct_t->arg = arg;
    new_hal_funct_t->funct = funct;
	rt_snprintf(new_hal_funct_t->name, sizeof(new_hal_funct_t->name), "%s", name);
    /* search list for 'name' and insert new structure */
    prev = &(hal_data->funct_list_ptr);
    next = *prev;
    while (1)
    {
		if (next == 0)
		{
			// 插入到链表末尾
		    /* reached end of list, insert here */
		    new_hal_funct_t->next_ptr = next;
		    *prev = SHMOFF(new_hal_funct_t);
		    /* break out of loop and init the new function */
		    break;
		}
		fptr = SHMPTR(next);
	    // 按字母序插入
		cmp = strcmp(fptr->name, new_hal_funct_t->name);
		if (cmp > 0)
		{
		    /* found the right place for it, insert here */
		    new_hal_funct_t->next_ptr = next;
		    *prev = SHMOFF(new_hal_funct_t);
		    /* break out of loop and init the new function */
		    break;
		}
	    // 处理重复名称
		if (cmp == 0)
		{
		    /* name already in list, can't insert */
		    free_funct_struct(new_hal_funct_t);
		    rtapi_mutex_give(&(hal_data->mutex));
			rtapi_print_msg(RTAPI_MSG_ERR,
			"HAL: ERROR: duplicate function '%s'\n", name);
		    return -EINVAL;
		}
		/* didn't find it yet, look at next one */
		prev = &(fptr->next_ptr);
		next = *prev;
    }
    /* at this point we have a new function and can yield the mutex */
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

//实时线程创建机制
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

    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* make sure name is unique on thread list */
	//get thread list
    next = hal_data->thread_list_ptr;
    while (next != 0)
    {
		tptr = SHMPTR(next);
		cmp = strcmp(tptr->name, name);
		if (cmp == 0)
		{
		    /* name already in list, can't insert */
		    rtapi_mutex_give(&(hal_data->mutex));
			rtapi_print_msg(RTAPI_MSG_ERR,
			"HAL: ERROR: duplicate thread name %s\n", name);
		    return -EINVAL;
		}
		/* didn't find it yet, look at next one */
		next = tptr->next_ptr;
    }
    /* allocate a new thread structure */
    new_hal_thread_t = alloc_thread_struct();
    if (new_hal_thread_t == 0) {
		/* alloc failed */
		rtapi_mutex_give(&(hal_data->mutex));
    	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: insufficient memory to create thread\n");
		return -ENOMEM;
    }

    /* initialize the structure */
    new_hal_thread_t->uses_fp = uses_fp;
	rt_snprintf(new_hal_thread_t->name, sizeof(new_hal_thread_t->name), "%s", name);
    /* have to create and start a main to run the thread */
    if (hal_data->thread_list_ptr == 0)
    {
    	curr_period = rtapi_clock_set_period(0);
    	if (curr_period == 0)
    	{
    		curr_period = rtapi_clock_set_period(period_nsec);
    		if (curr_period < 0) {
    			rtapi_mutex_give(&(hal_data->mutex));
    			return -EINVAL;
    		}
    	}
    	/* make sure period <= desired period (allow 1% roundoff error) */
    	if (curr_period > (long)(period_nsec + (period_nsec / 100))) {
    		rtapi_mutex_give(&(hal_data->mutex));
    		return -EINVAL;
    	}

		prev_priority = 0;
		/* no previous period to worry about */
		prev_period = 0;
    }
	else
    {
		tptr = SHMPTR(hal_data->thread_list_ptr);
		prev_period = tptr->period;
		prev_priority = tptr->priority;
    }

	//set period
    new_hal_thread_t->period = period_nsec;

	// 后续线程
	new_hal_thread_t->priority = priority;
    /* create main - owned by library module, not caller */
	//创建任务
    retval = rtapi_task_new(thread_task, new_hal_thread_t, new_hal_thread_t->priority,
	lib_module_id, HAL_STACKSIZE, uses_fp);
    if (retval < 0)
    {
		rtapi_mutex_give(&(hal_data->mutex));
		return -EINVAL;
    }
    new_hal_thread_t->task_id = retval;
    /* start main */
	// 启动任务
    retval = rtapi_task_start(new_hal_thread_t->task_id, new_hal_thread_t->period);
    if (retval < 0)
    {
		rtapi_mutex_give(&(hal_data->mutex));
		return -EINVAL;
    }
	// 插入线程链表
    /* insert new structure at head of list */
    new_hal_thread_t->next_ptr = hal_data->thread_list_ptr;
    hal_data->thread_list_ptr = SHMOFF(new_hal_thread_t);
    /* done, release mutex */
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
	//init hal_data_t
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

    // rtapi_print_msg(RTAPI_MSG_DBG, "HAL: deleting thread '%s'\n", name);
    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* search for the signal */
    prev = &(hal_data->thread_list_ptr);
    next = *prev;
    while (next != 0) {
	thread = SHMPTR(next);
	if (strcmp(thread->name, name) == 0) {
	    /* this is the right thread, unlink from list */
	    if (thread->comp_id != 0) {
	        hal_exit(thread->comp_id);
	        thread->comp_id = 0;
	    }
	    *prev = thread->next_ptr;
	    /* and delete it */
	    free_thread_struct(thread);
	    /* done */
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 0;
	}
	/* no match, try the next one */
	prev = &(thread->next_ptr);
	next = *prev;
    }
    /* if we get here, we didn't find a match */
    rtapi_mutex_give(&(hal_data->mutex));
    rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: thread '%s' not found\n",
	name);
    return -EINVAL;
}

//将HAL函数添加到实时线程的功能
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

	//验证函数和线程的合法性
    if (hal_data->lock & HAL_LOCK_CONFIG) {
    	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: add_funct_to_thread called while HAL is locked\n");
	return -EPERM;
    }

    /* get mutex before accessing data structures */
    rtapi_mutex_get(&(hal_data->mutex));
    /* make sure position is valid */
    if (position == 0) {
	/* zero is not allowed */
    	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: bad position: 0\n");
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }

    /* make sure we were given a function name */
    if (funct_name == 0) {
	/* no name supplied */
    	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing function name\n");
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }

    /* make sure we were given a thread name */
    if (thread_name == 0) {
	/* no name supplied */
    	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing thread name\n");
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }
	//通过函数名查找对应的 HAL 函数结构体
    /* search function list for the function */
    funct = halpr_find_funct_by_name(funct_name);
    if (funct == 0) {
	/* function not found */
	rtapi_mutex_give(&(hal_data->mutex));
    	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: function '%s' not found\n", funct_name);
	return -EINVAL;
    }
	//非重入函数只能添加到一个线程
    /* found the function, is it available? */
    if ((funct->users > 0) && (funct->reentrant == 0)) {
	rtapi_mutex_give(&(hal_data->mutex));
    	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: function '%s' may only be added to one thread\n", funct_name);
	return -EINVAL;
    }
	//通过线程名查找对应的 HAL 线程结构体
    /* search thread list for thread_name */
    thread = halpr_find_thread_by_name(thread_name);
    if (thread == 0) {
	/* thread not found */
	rtapi_mutex_give(&(hal_data->mutex));
    	rtapi_print_msg(RTAPI_MSG_ERR,"HAL: ERROR: thread '%s' not found\n", thread_name);
	return -EINVAL;
    }
	//需要浮点的函数必须添加到支持浮点的线程
    /* ok, we have thread and function, are they compatible? */
    if ((funct->uses_fp) && (!thread->uses_fp)) {
	rtapi_mutex_give(&(hal_data->mutex));
	return -EINVAL;
    }
    /* find insertion point */
    list_root = &(thread->funct_list);
    list_entry = list_root;
    n = 0;
    if (position > 0)
    {
	    // 从链表头部开始正向查找
		/* insertion is relative to start of list */
		while (++n < position)
		{
		    /* move further into list */
		    list_entry = list_next(list_entry);
		    if (list_entry == list_root)
		    {
				/* reached end of list */
				rtapi_mutex_give(&(hal_data->mutex));
	    			rtapi_print_msg(RTAPI_MSG_ERR,
					"HAL: ERROR: position '%d' is too high\n", position);
				return -EINVAL;
		    }
		}
    } else
    {
    	// 从链表尾部开始反向查找
		/* insertion is relative to end of list */
		while (--n > position)
		{
		    /* move further into list */
		    list_entry = list_prev(list_entry);
		    if (list_entry == list_root)
		    {
				/* reached end of list */
				rtapi_mutex_give(&(hal_data->mutex));
	    			rtapi_print_msg(RTAPI_MSG_ERR,
					"HAL: ERROR: position '%d' is too low\n", position);
				return -EINVAL;
		    }
		}
		/* want to insert before list_entry, so back up one more step */
		list_entry = list_prev(list_entry);
    }

	//分配新的函数入口结构体
    /* allocate a funct entry structure */
    funct_entry = alloc_funct_entry_struct();
    if (funct_entry == 0) {
	/* alloc failed */
	rtapi_mutex_give(&(hal_data->mutex));
	return -ENOMEM;
    }
	//初始化入口内容
    /* init struct contents */
    funct_entry->funct_ptr = SHMOFF(funct);
    funct_entry->arg = funct->arg;
    funct_entry->funct = funct->funct;
    /* add the entry to the list */
	//将新函数入口插入到计算好的位置
    list_add_after((hal_list_t *) funct_entry, list_entry);
    /* update the function usage count */
	//增加函数的使用计数
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

    // rtapi_print_msg(RTAPI_MSG_DBG,"HAL: removing function '%s' from thread '%s'\n",funct_name, thread_name);
    /* get mutex before accessing data structures */
    rtapi_mutex_get(&(hal_data->mutex));
    /* make sure we were given a function name */
    if (funct_name == 0) {
	/* no name supplied */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing function name\n");
	return -EINVAL;
    }
    /* make sure we were given a thread name */
    if (thread_name == 0) {
	/* no name supplied */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: missing thread name\n");
	return -EINVAL;
    }
    /* search function list for the function */
    funct = halpr_find_funct_by_name(funct_name);
    if (funct == 0) {
	/* function not found */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: function '%s' not found\n", funct_name);
	return -EINVAL;
    }
    /* found the function, is it in use? */
    if (funct->users == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: function '%s' is not in use\n", funct_name);
	return -EINVAL;
    }
    /* search thread list for thread_name */
    thread = halpr_find_thread_by_name(thread_name);
    if (thread == 0) {
	/* thread not found */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: thread '%s' not found\n", thread_name);
	return -EINVAL;
    }
    /* ok, we have thread and function, does thread use funct? */
    list_root = &(thread->funct_list);
    list_entry = list_next(list_root);
    while (1) {
	if (list_entry == list_root) {
	    /* reached end of list, funct not found */
	    rtapi_mutex_give(&(hal_data->mutex));
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: thread '%s' doesn't use %s\n", thread_name,
		funct_name);
	    return -EINVAL;
	}
	funct_entry = (hal_funct_entry_t *) list_entry;
	if (SHMPTR(funct_entry->funct_ptr) == funct) {
	    /* this funct entry points to our funct, unlink */
	    list_remove_entry(list_entry);
	    /* and delete it */
	    free_funct_entry_struct(funct_entry);
	    /* done */
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 0;
	}
	/* try next one */
	list_entry = list_next(list_entry);
    }
}

int hal_start_threads(void)
{
    /* a trivial function for a change! */
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

    // rtapi_print_msg(RTAPI_MSG_DBG, "HAL: starting threads\n");
    hal_data->threads_running = 1;
    return 0;
}

int hal_stop_threads(void)
{
    /* wow, two in a row! */
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

    hal_data->threads_running = 0;
    // rtapi_print_msg(RTAPI_MSG_DBG, "HAL: threads stopped\n");
    return 0;
}

//RTAPI 实时环境初始化
int hal_app_main(void)
{
    int retval;
    void *mem;

    // rtapi_print_msg(RTAPI_MSG_DBG, "HAL_LIB: loading kernel lib\n");
    /* do RTAPI init */
	//创建实时模块并获取模块 ID
    lib_module_id = rtapi_init("HAL_LIB");
    if (lib_module_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL_LIB: ERROR: rtapi init failed\n");
	return -EINVAL;
    }
	//分配 HAL专用的共享内存区域
    /* get HAL shared memory block from RTAPI */
    lib_mem_id = rtapi_shmem_new(HAL_KEY, lib_module_id, HAL_SIZE);
    if (lib_mem_id < 0)
    {
		rtapi_print_msg(RTAPI_MSG_ERR,"HAL_LIB: ERROR: could not open shared memory\n");
		rtapi_exit(lib_module_id);
		return -EINVAL;
    }

	//内存映射
    /* get address of shared memory area */
    retval = rtapi_shmem_getptr(lib_mem_id, &mem);
    if (retval < 0)
    {
		rtapi_print_msg(RTAPI_MSG_ERR,"HAL_LIB: ERROR: could not access shared memory\n");
		rtapi_exit(lib_module_id);
		return -EINVAL;
    }
    /* set up internal pointers to shared mem and data structure */
    hal_shmem_base = (char *) mem;
    hal_data = (hal_data_t *) mem;
	//HAL 数据结构初始化
    /* perform a global init if needed */
    retval = init_hal_data();
    if ( retval )
    {
		rtapi_print_msg(RTAPI_MSG_ERR,"HAL_LIB: ERROR: could not init shared memory\n");
		rtapi_exit(lib_module_id);
		return -EINVAL;
    }

    /* done */
    rtapi_print_msg(RTAPI_MSG_DBG, "HAL_LIB: kernel lib installed successfully\n");
    return 0;
}

void hal_app_exit(void)
{
    hal_thread_t *thread;

    // rtapi_print_msg(RTAPI_MSG_DBG, "HAL_LIB: removing kernel lib\n");
    /* grab mutex before manipulating list */
    rtapi_mutex_get(&(hal_data->mutex));
    /* must remove all threads before unloading this module */
    while (hal_data->thread_list_ptr != 0) {
	/* point to a thread */
	thread = SHMPTR(hal_data->thread_list_ptr);
	/* unlink from list */
	hal_data->thread_list_ptr = thread->next_ptr;
	/* and delete it */
	free_thread_struct(thread);
    }
    /* release mutex */
    rtapi_mutex_give(&(hal_data->mutex));
    /* release RTAPI resources */
    rtapi_shmem_delete(lib_mem_id, lib_module_id);
    rtapi_exit(lib_module_id);
    /* done */
    // rtapi_print_msg(RTAPI_MSG_DBG,"HAL_LIB: kernel lib removed successfully\n");
}

/* this is the main function that implements threads in realtime */

static void thread_task(void *arg)
{
    hal_thread_t *thread;//指向当前线程结构的指针
    hal_funct_t *funct;//指向当前执行的函数结构的指针
    hal_funct_entry_t *funct_root, *funct_entry;//用于遍历函数链表的指针
    long long int start_time, end_time;//用于记录单个函数的开始和结束时间
    long long int thread_start_time;//记录整个线程周期的开始时间

    thread = arg;

    while (1)
    {
		if (hal_data->threads_running > 0)
		{
		    /* point at first function on function list */
		    funct_root = (hal_funct_entry_t *) & (thread->funct_list);//获取函数链表头节点
		    funct_entry = SHMPTR(funct_root->links.next);//定位到第一个实际函数节点

			// 检查链表是否为空
			if (funct_entry == funct_root) {
				rtapi_print_msg(RTAPI_MSG_WARN,
					"Thread %s: function list is empty\n", thread->name);
				rtapi_wait();
				continue;
			}
		    /* execution time logging */
		    start_time = rtapi_get_clocks();
		    end_time = start_time;
		    thread_start_time = start_time;
		    /* run thru function list */
			int func_count = 0;
		    while (funct_entry != funct_root)
		    {
		    	func_count++;
				/* call the function */
				funct_entry->funct(funct_entry->arg, thread->period);
				/* capture execution time */
				end_time = rtapi_get_clocks();
				/* point to function structure */
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
				/* update execution time data */
			    //测量函数实际执行时间
		    	if (funct->runtime == NULL) {
		    		rtapi_print_msg(RTAPI_MSG_ERR,
						"Thread %s: Function '%s' has NULL runtime pointer!\n",
						thread->name, funct->name ? funct->name : "unknown");
		    		// 继续处理下一个函数而不是崩溃
		    		funct_entry = SHMPTR(funct_entry->links.next);
		    		continue;
		    	}

				if ( *(funct->runtime) > funct->maxtime) {
				    funct->maxtime = *(funct->runtime);
				    funct->maxtime_increased = 1;
				} else {
				    funct->maxtime_increased = 0;
				}
				/* point to next next entry in list */
			    //移动到链表中的下一个函数
				funct_entry = SHMPTR(funct_entry->links.next);
				/* prepare to measure time for next funct */
				start_time = end_time;
		    }
		    /* update thread execution time */
		    *(thread->runtime) = (hal_s32_t)(end_time - thread_start_time);
		    if ( *(thread->runtime) > thread->maxtime) {
		        thread->maxtime = *(thread->runtime);
		    }
		}
		/* wait until next period */
		rtapi_wait();
    }
}

static int init_hal_data(void)
{
    rtapi_mutex_get(&(hal_data->mutex));

    /* initialize everything */
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
    /* set up for shmalloc_xx() */
    hal_data->shmem_bot = sizeof(hal_data_t);
    hal_data->shmem_top = HAL_SIZE;
    hal_data->lock = HAL_LOCK_NONE;
    /* done, release mutex */
    rtapi_mutex_give(&(hal_data->mutex));
    return 0;
}

static void *shmalloc_up(long int size)
{
    long int tmp_bot;
    void *retval;

    /* deal with alignment requirements */
    tmp_bot = hal_data->shmem_bot;
    if (size >= 16) {
	/* align on 16 byte boundary */
	tmp_bot = (tmp_bot + 15) & (~15);
    } else if (size >= 8) {
	/* align on 8 byte boundary */
	tmp_bot = (tmp_bot + 7) & (~7);
    } else if (size >= 4) {
	/* align on 4 byte boundary */
	tmp_bot = (tmp_bot + 3) & (~3);
    } else if (size == 2) {
	/* align on 2 byte boundary */
	tmp_bot = (tmp_bot + 1) & (~1);
    }
    /* is there enough memory available? */
    if ((hal_data->shmem_top - tmp_bot) < size) {
	/* no */
	return 0;
    }
    /* memory is available, allocate it */
    retval = SHMPTR(tmp_bot);
    hal_data->shmem_bot = tmp_bot + size;
    hal_data->shmem_avail = hal_data->shmem_top - hal_data->shmem_bot;
    // rtapi_print_msg(RTAPI_MSG_DBG, "smalloc_up: shmem available %d\n", hal_data->shmem_avail);
    return retval;
}

static void *shmalloc_dn(long int size)
{
    long int tmp_top;
    void *retval;

    /* tentatively allocate memory */
    tmp_top = hal_data->shmem_top - size;
    /* deal with alignment requirements */
    if (size >= 16) {
	/* align on 16 byte boundary */
	tmp_top &= (~15);
    } else if (size >= 8) {
	/* align on 8 byte boundary */
	tmp_top &= (~7);
    } else if (size >= 4) {
	/* align on 4 byte boundary */
	tmp_top &= (~3);
    } else if (size == 2) {
	/* align on 2 byte boundary */
	tmp_top &= (~1);
    }
    /* is there enough memory available? */
    if (tmp_top < hal_data->shmem_bot) {
	/* no */
	return 0;
    }
    /* memory is available, allocate it */
    retval = SHMPTR(tmp_top);
    hal_data->shmem_top = tmp_top;
    hal_data->shmem_avail = hal_data->shmem_top - hal_data->shmem_bot;
    // rtapi_print_msg(RTAPI_MSG_DBG, "smalloc_dn: shmem available %d\n", hal_data->shmem_avail);
    return retval;
}

hal_comp_t *halpr_alloc_comp_struct(void)
{
    hal_comp_t *p;

    /* check the free list */
    if (hal_data->comp_free_ptr != 0) {
	/* found a free structure, point to it */
	p = SHMPTR(hal_data->comp_free_ptr);
	/* unlink it from the free list */
	hal_data->comp_free_ptr = p->next_ptr;
	p->next_ptr = 0;
    } else {
	/* nothing on free list, allocate a brand new one */
	p = shmalloc_dn(sizeof(hal_comp_t));
    }
    if (p) {
	/* make sure it's empty */
	p->next_ptr = 0;
	p->comp_id = 0;
	p->mem_id = 0;
	p->type = COMPONENT_TYPE_USER;
	p->shmem_base = 0;
	p->name[0] = '\0';
    }
    return p;
}
static hal_funct_t *alloc_funct_struct(void)
{
    hal_funct_t *p;

    /* check the free list */
    if (hal_data->funct_free_ptr != 0) {
	/* found a free structure, point to it */
	p = SHMPTR(hal_data->funct_free_ptr);
	/* unlink it from the free list */
	hal_data->funct_free_ptr = p->next_ptr;
	p->next_ptr = 0;
    } else {
	/* nothing on free list, allocate a brand new one */
	p = shmalloc_dn(sizeof(hal_funct_t));
    }
    if (p) {
	/* make sure it's empty */
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


static hal_funct_entry_t *alloc_funct_entry_struct(void)
{
    hal_list_t *freelist, *l;
    hal_funct_entry_t *p;

    /* check the free list */
    freelist = &(hal_data->funct_entry_free);
    l = list_next(freelist);
    if (l != freelist) {
	/* found a free structure, unlink from the free list */
	list_remove_entry(l);
	p = (hal_funct_entry_t *) l;
    } else {
	/* nothing on free list, allocate a brand new one */
	p = shmalloc_dn(sizeof(hal_funct_entry_t));
	l = (hal_list_t *) p;
	list_init_entry(l);
    }
    if (p) {
	/* make sure it's empty */
	p->funct_ptr = 0;
	p->arg = 0;
	p->funct = 0;
    }
    return p;
}


static hal_thread_t *alloc_thread_struct(void)
{
    hal_thread_t *p;

    /* check the free list */
    if (hal_data->thread_free_ptr != 0) {
	/* found a free structure, point to it */
	p = SHMPTR(hal_data->thread_free_ptr);
	/* unlink it from the free list */
	hal_data->thread_free_ptr = p->next_ptr;
	p->next_ptr = 0;
    } else {
	/* nothing on free list, allocate a brand new one */
	p = shmalloc_dn(sizeof(hal_thread_t));
    }
    if (p) {
	/* make sure it's empty */
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


static void free_comp_struct(hal_comp_t * comp)
{
    rtapi_intptr_t *prev, next;

    hal_funct_t *funct;

    hal_param_t *param;

    /* can't delete the component until we delete its "stuff" */
    /* need to check for functs only if a realtime component */

    /* search the function list for this component's functs */
    prev = &(hal_data->funct_list_ptr);
    next = *prev;
    while (next != 0) {
	funct = SHMPTR(next);
	if (SHMPTR(funct->owner_ptr) == comp) {
	    /* this function belongs to our component, unlink from list */
	    *prev = funct->next_ptr;
	    /* and delete it */
	    free_funct_struct(funct);
	} else {
	    /* no match, try the next one */
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
	    /* this param belongs to our component, unlink from list */
	    *prev = param->next_ptr;
	    /* and delete it */
	    free_param_struct(param);
	} else {
	    /* no match, try the next one */
	    prev = &(param->next_ptr);
	}
	next = *prev;
    }
    /* now we can delete the component itself */
    /* clear contents of struct */
    comp->comp_id = 0;
    comp->mem_id = 0;
    comp->type = COMPONENT_TYPE_USER;
    comp->shmem_base = 0;
    comp->name[0] = '\0';
    /* add it to free list */
    comp->next_ptr = hal_data->comp_free_ptr;
    hal_data->comp_free_ptr = SHMOFF(comp);
}

static void free_param_struct(hal_param_t * p)
{
	/* clear contents of struct */
	if ( p->oldname != 0 ) free_oldname_struct(SHMPTR(p->oldname));
	p->data_ptr = 0;
	p->owner_ptr = 0;
	p->type = 0;
	p->name[0] = '\0';
	/* add it to free list (params use the same struct as src vars) */
	p->next_ptr = hal_data->param_free_ptr;
	hal_data->param_free_ptr = SHMOFF(p);
}

static void free_oldname_struct(hal_oldname_t * oldname)
{
	/* clear contents of struct */
	oldname->name[0] = '\0';
	/* add it to free list */
	oldname->next_ptr = hal_data->oldname_free_ptr;
	hal_data->oldname_free_ptr = SHMOFF(oldname);
}

static void free_funct_struct(hal_funct_t * funct)
{
    int next_thread;
    hal_thread_t *thread;
    hal_list_t *list_root, *list_entry;
    hal_funct_entry_t *funct_entry;

/*  int next_thread, next_entry;*/

    if (funct->users > 0) {
	/* We can't casually delete the function, there are thread(s) which
	   will call it.  So we must check all the threads and remove any
	   funct_entrys that call this function */
	/* start at root of thread list */
	next_thread = hal_data->thread_list_ptr;
	/* run through thread list */
	while (next_thread != 0) {
	    /* point to thread */
	    thread = SHMPTR(next_thread);
	    /* start at root of funct_entry list */
	    list_root = &(thread->funct_list);
	    list_entry = list_next(list_root);
	    /* run thru funct_entry list */
	    while (list_entry != list_root) {
		/* point to funct entry */
		funct_entry = (hal_funct_entry_t *) list_entry;
		/* test it */
		if (SHMPTR(funct_entry->funct_ptr) == funct) {
		    /* this funct entry points to our funct, unlink */
		    list_entry = list_remove_entry(list_entry);
		    /* and delete it */
		    free_funct_entry_struct(funct_entry);
		} else {
		    /* no match, try the next one */
		    list_entry = list_next(list_entry);
		}
	    }
	    /* move on to the next thread */
	    next_thread = thread->next_ptr;
	}
    }
    /* clear contents of struct */
    funct->uses_fp = 0;
    funct->owner_ptr = 0;
    funct->reentrant = 0;
    funct->users = 0;
    funct->arg = 0;
    funct->funct = 0;
    funct->runtime = 0;
    funct->name[0] = '\0';
    /* add it to free list */
    funct->next_ptr = hal_data->funct_free_ptr;
    hal_data->funct_free_ptr = SHMOFF(funct);
}


static void free_funct_entry_struct(hal_funct_entry_t * funct_entry)
{
    hal_funct_t *funct;

    if (funct_entry->funct_ptr > 0) {
	/* entry points to a function, update the function struct */
	funct = SHMPTR(funct_entry->funct_ptr);
	funct->users--;
    }
    /* clear contents of struct */
    funct_entry->funct_ptr = 0;
    funct_entry->arg = 0;
    funct_entry->funct = 0;
    /* add it to free list */
    list_add_after((hal_list_t *) funct_entry, &(hal_data->funct_entry_free));
}

static void free_thread_struct(hal_thread_t * thread)
{
    hal_funct_entry_t *funct_entry;
    hal_list_t *list_root, *list_entry;

    /* if we're deleting a thread, we need to stop all threads */
    hal_data->threads_running = 0;
    /* and stop the main associated with this thread */
    rtapi_task_pause(thread->task_id);
    rtapi_task_delete(thread->task_id);
    /* clear contents of struct */
    thread->uses_fp = 0;
    thread->period = 0;
    thread->priority = 0;
    thread->task_id = 0;
    /* clear the function entry list */
    list_root = &(thread->funct_list);
    list_entry = list_next(list_root);
    while (list_entry != list_root) {
	/* entry found, save pointer to it */
	funct_entry = (hal_funct_entry_t *) list_entry;
	/* unlink it, point to the next one */
	list_entry = list_remove_entry(list_entry);
	/* free the removed entry */
	free_funct_entry_struct(funct_entry);
    }

    thread->name[0] = '\0';
    /* add thread to free list */
    thread->next_ptr = hal_data->thread_free_ptr;
    hal_data->thread_free_ptr = SHMOFF(thread);
}

hal_comp_t *halpr_find_comp_by_name(const char *name)
{
	int next;
	hal_comp_t *comp;

	/* search component list for 'name' */
	next = hal_data->comp_list_ptr;
	while (next != 0) {
		comp = SHMPTR(next);
		if (strcmp(comp->name, name) == 0)
		{
			/* found a match */
			return comp;
		}
		/* didn't find it yet, look at next one */
		next = comp->next_ptr;
	}
	/* if loop terminates, we reached end of list with no match */
	return 0;
}

hal_comp_t *halpr_find_comp_by_id(int id) {
	int next;
	hal_comp_t *comp;

	/* search list for 'comp_id' */
	next = hal_data->comp_list_ptr;
	while (next != 0) {
		comp = SHMPTR(next);
		if (comp->comp_id == id) {
			/* found a match */
			return comp;
		}
		/* didn't find it yet, look at next one */
		next = comp->next_ptr;
	}
	/* if loop terminates, we reached end of list without finding a match */
	return 0;
}

hal_funct_t *halpr_find_funct_by_name(const char *name)
{
	int next;
	hal_funct_t *funct;

	/* search function list for 'name' */
	next = hal_data->funct_list_ptr;
	while (next != 0) {
		funct = SHMPTR(next);
		if (strcmp(funct->name, name) == 0) {
			/* found a match */
			return funct;
		}
		/* didn't find it yet, look at next one */
		next = funct->next_ptr;
	}
	/* if loop terminates, we reached end of list with no match */
	return 0;
}

int hal_param_bit_new(const char *name, hal_param_dir_t dir, hal_type_t type, hal_bit_t * data_addr, int comp_id)
{
	return hal_param_new(name, type, dir, (void *) data_addr, comp_id);
}

int hal_param_s32_new(const char *name, hal_param_dir_t dir, hal_type_t type, hal_s32_t * data_addr,int comp_id)
{
	return hal_param_new(name, type, dir, (void *) data_addr, comp_id);
}

int hal_pin_newf(hal_type_t type, hal_s32_t ** data_ptr_addr, int comp_id, const char *fmt, ...)
{
	va_list ap;
	int ret;
	va_start(ap, fmt);
	ret = hal_pin_newfv(type, (void**)data_ptr_addr, comp_id, fmt, ap);
	va_end(ap);
	return ret;
}

static int hal_pin_newfv(hal_type_t type, void ** data_ptr_addr, int comp_id, const char *fmt, va_list ap)
{
	char name[HAL_NAME_LEN + 1];
	int sz;
	sz = rt_vsnprintf(name, sizeof(name), fmt, ap);
	if(sz == -1 || sz > HAL_NAME_LEN) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		"hal_pin_newfv: length %d too long for name starting '%s'\n",
		sz, name);
		return -ENOMEM;
	}
	return hal_pin_new(name, type, data_ptr_addr, comp_id);
}

int hal_param_new(const char *name, hal_type_t type, hal_param_dir_t dir, void *data_addr, int comp_id)
{
    rtapi_intptr_t *prev, next;
    int cmp;
    hal_param_t *new_hal_param_t, *ptr;
    hal_comp_t *comp;
	// HAL 系统初始化检查
    if (hal_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: param_new called before init\n");
	return -EINVAL;
    }

    if (type != HAL_BIT && type != HAL_FLOAT && type != HAL_S32 && type != HAL_U32 && type != HAL_S64 && type != HAL_U64) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: pin type not one of HAL_BIT, HAL_FLOAT, HAL_S32, HAL_U32, Hal_S64 or HAL_U64\n");
	return -EINVAL;
    }

    if(dir != HAL_RO && dir != HAL_RW) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: param direction not one of HAL_RO, or HAL_RW\n");
	return -EINVAL;
    }

    if (strlen(name) > HAL_NAME_LEN) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: parameter name '%s' is too long\n", name);
	return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_LOAD)  {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: param_new called while HAL locked\n");
	return -EPERM;
    }

    // rtapi_print_msg(RTAPI_MSG_DBG, "HAL: creating parameter '%s'\n", name);
    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* validate comp_id */
	// 组件存在性验证
    comp = halpr_find_comp_by_id(comp_id);
    if (comp == 0) {
	/* bad comp_id */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: component %d not found\n", comp_id);
	return -EINVAL;
    }
    /* validate passed in pointer - must point to HAL shmem */
    if (! SHMCHK(data_addr)) {
	/* bad pointer */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: data_addr not in shared memory\n");
	return -EINVAL;
    }
    if(comp->ready) {
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: param_new called after hal_ready\n");
	return -EINVAL;
    }
    /* allocate a new parameter structure */
	// 分配参数结构体
    new_hal_param_t = alloc_param_struct();
    if (new_hal_param_t == 0) {
	/* alloc failed */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: insufficient memory for parameter '%s'\n", name);
	return -ENOMEM;
    }
    /* initialize the structure */
    new_hal_param_t->owner_ptr = SHMOFF(comp);
    new_hal_param_t->data_ptr = SHMOFF(data_addr);
    new_hal_param_t->type = type;
    new_hal_param_t->dir = dir;
	rt_snprintf(new_hal_param_t->name, sizeof(new_hal_param_t->name), "%s", name);
    /* search list for 'name' and insert new structure */
    prev = &(hal_data->param_list_ptr);
    next = *prev;
    while (1) {
	if (next == 0) {
	    /* reached end of list, insert here */
	    new_hal_param_t->next_ptr = next;
	    *prev = SHMOFF(new_hal_param_t);
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 0;
	}
	ptr = SHMPTR(next);
	cmp = strcmp(ptr->name, new_hal_param_t->name);
	if (cmp > 0) {
	    /* found the right place for it, insert here */
	    new_hal_param_t->next_ptr = next;
	    *prev = SHMOFF(new_hal_param_t);
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 0;
	}
	if (cmp == 0)
	{
	    /* name already in list, can't insert */
	    free_param_struct(new_hal_param_t);
	    rtapi_mutex_give(&(hal_data->mutex));
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL: ERROR: duplicate parameter '%s'\n", name);
	    return -EINVAL;
	}
	/* didn't find it yet, look at next one */
	prev = &(ptr->next_ptr);
	next = *prev;
    }
}

static hal_param_t *alloc_param_struct(void)
{
	hal_param_t *p;

	/* check the free list */
	if (hal_data->param_free_ptr != 0) {
		/* found a free structure, point to it */
		p = SHMPTR(hal_data->param_free_ptr);
		/* unlink it from the free list */
		hal_data->param_free_ptr = p->next_ptr;
		p->next_ptr = 0;
	} else {
		/* nothing on free list, allocate a brand new one */
		p = shmalloc_dn(sizeof(hal_param_t));
	}
	if (p) {
		/* make sure it's empty */
		p->next_ptr = 0;
		p->data_ptr = 0;
		p->owner_ptr = 0;
		p->type = 0;
		p->name[0] = '\0';
	}
	return p;
}

hal_thread_t *halpr_find_thread_by_name(const char *name)
{
	int next;
	hal_thread_t *thread;

	/* search thread list for 'name' */
	next = hal_data->thread_list_ptr;
	while (next != 0) {
		thread = SHMPTR(next);
		if (strcmp(thread->name, name) == 0) {
			/* found a match */
			return thread;
		}
		/* didn't find it yet, look at next one */
		next = thread->next_ptr;
	}
	/* if loop terminates, we reached end of list with no match */
	return 0;
}

hal_list_t *list_prev(hal_list_t * entry)
{
	/* this function is only needed because of memory mapping */
	return SHMPTR(entry->prev);
}

hal_list_t *list_next(hal_list_t * entry)
{
	/* this function is only needed because of memory mapping */
	return SHMPTR(entry->next);
}

void list_add_after(hal_list_t * entry, hal_list_t * prev)
{
	int entry_n, prev_n, next_n;
	hal_list_t *next;

	/* messiness needed because of memory mapping */
	entry_n = SHMOFF(entry);
	prev_n = SHMOFF(prev);
	next_n = prev->next;
	next = SHMPTR(next_n);
	/* insert the entry */
	entry->next = next_n;
	entry->prev = prev_n;
	prev->next = entry_n;
	next->prev = entry_n;
}

void list_add_before(hal_list_t * entry, hal_list_t * next) {
	int entry_n, prev_n, next_n;
	hal_list_t *prev;

	/* messiness needed because of memory mapping */
	entry_n = SHMOFF(entry);
	next_n = SHMOFF(next);
	prev_n = next->prev;
	prev = SHMPTR(prev_n);
	/* insert the entry */
	entry->next = next_n;
	entry->prev = prev_n;
	prev->next = entry_n;
	next->prev = entry_n;
}

hal_list_t *list_remove_entry(hal_list_t * entry)
{
	int entry_n;
	hal_list_t *prev, *next;

	/* messiness needed because of memory mapping */
	entry_n = SHMOFF(entry);
	prev = SHMPTR(entry->prev);
	next = SHMPTR(entry->next);
	/* remove the entry */
	prev->next = entry->next;
	next->prev = entry->prev;
	entry->next = entry_n;
	entry->prev = entry_n;
	return next;
}

void list_init_entry(hal_list_t * entry)
{
	int entry_n;

	entry_n = SHMOFF(entry);
	entry->next = entry_n;
	entry->prev = entry_n;
}


static int hal_param_newfv(hal_type_t type, hal_param_dir_t dir,
	void *data_addr, int comp_id, const char *fmt, va_list ap) {
	char name[HAL_NAME_LEN + 1];
	int sz;
	sz = rt_snprintf(name, sizeof(name), fmt, ap);
	if(sz == -1 || sz > HAL_NAME_LEN) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		"hal_param_newfv: length %d too long for name starting '%s'\n",
		sz, name);
		return -ENOMEM;
	}
	return hal_param_new(name, type, dir, (void *) data_addr, comp_id);
}

int hal_param_float_newf(hal_param_dir_t dir, hal_float_t * data_addr,
	int comp_id, const char *fmt, ...)
{
	va_list ap;
	int ret;
	va_start(ap, fmt);
	ret = hal_param_newfv(HAL_FLOAT, dir, (void*)data_addr, comp_id, fmt, ap);
	va_end(ap);
	return ret;
}




static hal_pin_t *alloc_pin_struct(void)
{
	hal_pin_t *p;

	/* check the free list */
	if (hal_data->pin_free_ptr != 0) {
		/* found a free structure, point to it */
		p = SHMPTR(hal_data->pin_free_ptr);
		/* unlink it from the free list */
		hal_data->pin_free_ptr = p->next_ptr;
		p->next_ptr = 0;
	} else {
		/* nothing on free list, allocate a brand new one */
		p = shmalloc_dn(sizeof(hal_pin_t));
	}
	if (p) {
		/* make sure it's empty */
		p->next_ptr = 0;
		p->data_ptr_addr = 0;
		p->owner_ptr = 0;
		p->type = 0;
		p->signal = 0;
		memset(&p->dummysig, 0, sizeof(hal_data_u));
		p->name[0] = '\0';
	}
	return p;
}

static void free_pin_struct(hal_pin_t * pin)
{
	/* clear contents of struct */
	if ( pin->oldname != 0 ) free_oldname_struct(SHMPTR(pin->oldname));
	pin->data_ptr_addr = 0;
	pin->owner_ptr = 0;
	pin->type = 0;
	pin->signal = 0;
	memset(&pin->dummysig, 0, sizeof(hal_data_u));
	pin->name[0] = '\0';
	/* add it to free list */
	pin->next_ptr = hal_data->pin_free_ptr;
	hal_data->pin_free_ptr = SHMOFF(pin);
}

int hal_pin_new(const char *name, hal_type_t type, void **data_ptr_addr, int comp_id)
{
    rtapi_intptr_t *prev, next;
    int cmp;
    hal_pin_t *new, *ptr;
    hal_comp_t *comp;

    if (hal_data == 0)
    {
		rtapi_print_msg(RTAPI_MSG_ERR,
		    "HAL: ERROR: pin_new called before init\n");
		return -EINVAL;
    }

    if(*data_ptr_addr)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: pin_new(%s) called with already-initialized memory\n", name);
		
    }

    if (type != HAL_BIT && type != HAL_FLOAT && type != HAL_S32 && type != HAL_U32 && type != HAL_S64 && type != HAL_U64 && type != HAL_PORT) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: pin type not one of HAL_BIT, HAL_FLOAT, HAL_S32, HAL_U32, HAL_S64, HAL_U64 or HAL_PORT\n");
	return -EINVAL;
    }

    if (strlen(name) > HAL_NAME_LEN) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: pin name '%s' is too long\n", name);
	return -EINVAL;
    }
    if (hal_data->lock & HAL_LOCK_LOAD)  {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: pin_new called while HAL locked\n");
	return -EPERM;
    }

    // rtapi_print_msg(RTAPI_MSG_DBG, "HAL: creating pin '%s'\n", name);
    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* validate comp_id */
    comp = halpr_find_comp_by_id(comp_id);
    if (comp == 0) {
	/* bad comp_id */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: component %d not found\n", comp_id);
	return -EINVAL;
    }
    /* validate passed in pointer - must point to HAL shmem */
    if (! SHMCHK(data_ptr_addr)) {
	/* bad pointer */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: data_ptr_addr not in shared memory\n");
	return -EINVAL;
    }
    if(comp->ready) {
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: pin_new called after hal_ready\n");
	return -EINVAL;
    }
    /* allocate a new variable structure */
    new = alloc_pin_struct();
    if (new == 0) {
	/* alloc failed */
	rtapi_mutex_give(&(hal_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL: ERROR: insufficient memory for pin '%s'\n", name);
	return -ENOMEM;
    }
    /* initialize the structure */
    new->data_ptr_addr = SHMOFF(data_ptr_addr);
    new->owner_ptr = SHMOFF(comp);
    new->type = type;
    new->signal = 0;
    memset(&new->dummysig, 0, sizeof(hal_data_u));
    rt_snprintf(new->name, sizeof(new->name), "%s", name);
    /* make 'data_ptr' point to dummy signal */
    *data_ptr_addr = (char *)comp->shmem_base + SHMOFF(&(new->dummysig));
    /* search list for 'name' and insert new structure */
    prev = &(hal_data->pin_list_ptr);
    next = *prev;
    while (1) {
	if (next == 0) {
	    /* reached end of list, insert here */
	    new->next_ptr = next;
	    *prev = SHMOFF(new);
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 0;
	}
	ptr = SHMPTR(next);
	cmp = strcmp(ptr->name, new->name);
	if (cmp > 0) {
	    /* found the right place for it, insert here */
	    new->next_ptr = next;
	    *prev = SHMOFF(new);
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 0;
	}
	if (cmp == 0) {
	    /* name already in list, can't insert */
	    free_pin_struct(new);
	    rtapi_mutex_give(&(hal_data->mutex));
	    rtapi_print_msg(RTAPI_MSG_ERR, "HAL: ERROR: duplicate variable '%s'\n", name);
		return -EINVAL;
	    // return 0;
	}
	/* didn't find it yet, look at next one */
	prev = &(ptr->next_ptr);
	next = *prev;
    }
}

