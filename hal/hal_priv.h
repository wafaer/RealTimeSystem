#ifndef HAL_PRIV_H
#define HAL_PRIV_H

#include <rtapi/rtapi_mutex.h>
#include <hal/hal.h>
#include <unistd.h>

#define HAL_NAME_LEN     47
#define HAL_STACKSIZE 16384
#define HAL_KEY   0x48414C32	/* key used to open HAL shared memory */
#define HAL_VER   0x00000010	/* version code */
#define HAL_SIZE  (256*4096)

#define HAL_PSEUDO_COMP_PREFIX "__"

extern char *hal_shmem_base;
extern struct hal_data_t *hal_data;

#ifdef __cplusplus
template<class T>
bool hal_shmchk(T *t) {
    char *c = (char*)t;
    return c > hal_shmem_base && c < hal_shmem_base + HAL_SIZE;
}

template<class T>
int hal_shmoff(T *t) { return t ? (char*)t - hal_shmem_base : 0; }

template<class T>
T *hal_shmptr(int p) { return p ? (T*)(hal_shmem_base + p) : nullptr; }

template<class T>
class hal_shmfield {
public:
    hal_shmfield() : off{} {}
    hal_shmfield(T *t) : off{hal_shmoff(t)} {}
    hal_shmfield &operator=(T *t) { off = hal_shmoff(t); return *this; }
    T *get() { return hal_shmptr<T>(off); }
    const T *get() const { return hal_shmptr<T>(off); }
    T *operator *() { return get(); }
    const T *operator *() const { return get(); }
    T *operator ->() { return get(); }
    operator bool() const { return off; }
private:
    rtapi_intptr_t off;
};

template<class T>
hal_shmfield<T> hal_make_shmfield(T *t) {
    return hal_shmfield<T>(t);
}

static_assert(sizeof(hal_shmfield<void>) == sizeof(rtapi_intptr_t), "hal_shmfield size matches");

#define SHMFIELD(type) hal_shmfield<type>
#define SHMPTR(arg) ((arg).get())
#define SHMOFF(ptr) (hal_make_shmfield(ptr))
#else
#define SHMFIELD(type) rtapi_intptr_t

/* SHMPTR(offset) converts 'offset' to a void pointer. */
#define SHMPTR(offset)  ( (void *)( hal_shmem_base + (offset) ) )

/* SHMOFF(ptr) converts 'ptr' to an offset from shared mem base.  */
#define SHMOFF(ptr)     ( ((char *)(ptr)) - hal_shmem_base )

#define SHMCHK(ptr)  ( ((char *)(ptr)) > (hal_shmem_base) && \
((char *)(ptr)) < (hal_shmem_base + HAL_SIZE) )
#endif

typedef struct hal_list_t {
    SHMFIELD(hal_list_t) next;			/* next element in list */
    SHMFIELD(hal_list_t) prev;			/* previous element in list */
} hal_list_t;

typedef struct hal_oldname_t {
    SHMFIELD(hal_oldname_t) next_ptr;		/* next struct (used for free list only) */
    char name[HAL_NAME_LEN + 1];	/* the original name */
} hal_oldname_t;

typedef struct hal_comp_t hal_comp_t;
typedef struct hal_thread_t hal_thread_t;
typedef struct hal_param_t hal_param_t;
typedef struct hal_funct_t hal_funct_t;
typedef struct hal_funct_entry_t hal_funct_entry_t;
typedef struct hal_pin_t hal_pin_t;

typedef enum
{
    COMPONENT_TYPE_UNKNOWN = -1,
    COMPONENT_TYPE_USER,
    COMPONENT_TYPE_REALTIME,
    COMPONENT_TYPE_OTHER
} component_type_t;

typedef enum {
    HAL_S_UNINIT        = 0,  // No shared memory allocated.
    HAL_S_INITIALIZING  = 1,  // shmget succeeded, init_hal_data in progress.
    HAL_S_ACTIVE        = 2,  // Fully initialised, ready for thread creation.
    HAL_S_RUNNING       = 3,  // Real-time threads are executing.
    HAL_S_SHUTTING_DOWN = 4,  // hal_app_exit() tearing down resources.
    HAL_S_DESTROYED     = 5,  // Shared memory released; no further use.
    HAL_S_ERROR         = -1  // Non-recoverable initialisation failure.
} hal_state_t;

typedef struct hal_data_t {
    rtapi_mutex_t mutex;	/* protection for linked lists, etc. */
    hal_s32_t shmem_avail;	/* amount of shmem left free */
    constructor pending_constructor;
    /* pointer to the pending constructor function */
    char constructor_prefix[HAL_NAME_LEN+1];
    /* prefix of name for new instance */
    char constructor_arg[HAL_NAME_LEN+1];
    /* prefix of name for new instance */
    int shmem_bot;		/* bottom of free shmem (first free byte) */
    int shmem_top;		/* top of free shmem (1 past last free) */
    SHMFIELD(hal_comp_t) comp_list_ptr;		/* root of linked list of components */
    SHMFIELD(hal_pin_t) pin_list_ptr;		/* root of linked list of pins */
    SHMFIELD(hal_param_t) param_list_ptr;		/* root of linked list of parameters */
    SHMFIELD(hal_funct_t) funct_list_ptr;		/* root of linked list of functions */
    SHMFIELD(hal_thread_t) thread_list_ptr;	/* root of linked list of threads */
    long base_period;		/* timer period for realtime tasks */
    int threads_running;	/* non-zero if threads are started */
    SHMFIELD(hal_oldname_t) oldname_free_ptr;	/* list of free oldname structs */
    SHMFIELD(hal_comp_t) comp_free_ptr;		/* list of free component structs */
    SHMFIELD(hal_pin_t) pin_free_ptr;		/* list of free pin structs */
    SHMFIELD(hal_param_t) param_free_ptr;		/* list of free parameter structs */
    SHMFIELD(hal_funct_t) funct_free_ptr;		/* list of free function structs */
    hal_list_t funct_entry_free;	/* list of free funct entry structs */
    SHMFIELD(hal_thread_t) thread_free_ptr;	/* list of free thread structs */
    int exact_base_period;      /* if set, pretend that rtapi satisfied our
                   period request exactly */
    unsigned char lock;         /* hal locking, can be one of the HAL_LOCK_* types */

    // --- Fields added for lifecycle management (Google-style) ---
    int state;             // Current HAL lifecycle state; one of HAL_S_*.
                          // Set by init_hal_data() and updated by
                          // hal_start_threads/hal_stop_threads/hal_app_exit.
    pid_t init_pid;        // PID of the process that performed hal_init()
                          // that created the shared memory.  0 until the
                          // first hal_init() call completes.
    int error_code;        // Last errno value from a failed OS operation
                          // within the HAL layer; 0 if no error.
} hal_data_t;

// ---------------------------------------------------------------------------
// hal_stats_t — statistics snapshot returned by hal_get_stats().
// Caller allocates; hal_get_stats() fills it under hal_data->mutex.
// ---------------------------------------------------------------------------
typedef struct {
    hal_state_t state;         // Current HAL lifecycle state.
    int comp_count;            // Number of registered components.
    int pin_count;             // Number of HAL pins.
    int param_count;           // Number of HAL parameters.
    int funct_count;           // Number of exported functions.
    int thread_count;          // Number of created threads.
    int threads_running;       // 1 if threads are started, 0 otherwise.
    long shmem_avail;          // Free bytes remaining in HAL shared memory.
    long shmem_total;          // Total HAL shared memory size (HAL_SIZE).
    pid_t init_pid;            // PID that initialised the HAL layer.
    int error_code;            // Last error captured by the HAL layer.
} hal_stats_t;


struct hal_comp_t {
    SHMFIELD(hal_comp_t) next_ptr;		/* next component in the list */
    int comp_id;		/* component ID (RTAPI module id) */
    int mem_id;			/* RTAPI shmem ID used by this comp */
    component_type_t type;
    int ready;                  /* nonzero if ready, 0 if not */
    int pid;			/* PID of component (user components only) */
    void *shmem_base;		/* base of shmem for this component */
    char name[HAL_NAME_LEN + 1];	/* component name */
    constructor make;
    SHMFIELD(char) insmod_args;		/* args passed to insmod when loaded */
};


struct hal_thread_t {
    SHMFIELD(hal_thread_t) next_ptr;		/* next thread in linked list */
    int uses_fp;		/* floating point flag */
    long int period;		/* period of the thread, in nsec */
    int priority;		/* priority of the thread */
    int task_id;		/* ID of the main that runs this thread */
    hal_s32_t* runtime;	/* (pin) duration of last run, in CPU cycles */
    hal_s32_t maxtime;	/* (param) duration of longest run, in CPU cycles */
    hal_list_t funct_list;	/* list of functions to run */
    char name[HAL_NAME_LEN + 1];	/* thread name */
    int comp_id;
};

struct hal_pin_t {
    SHMFIELD(hal_pin_t) next_ptr;		/* next pin in linked list */
    SHMFIELD(void*) data_ptr_addr;		/* address of pin data pointer */
    SHMFIELD(hal_comp_t) owner_ptr;		/* component that owns this pin */
    SHMFIELD(hal_sig_t) signal;			/* signal to which pin is linked */
    hal_data_u dummysig;	/* if unlinked, data_ptr points here */
    SHMFIELD(hal_oldname_t) oldname;		/* old name if aliased, else zero */
    hal_type_t type;		/* data type */
    char name[HAL_NAME_LEN + 1];	/* pin name */
};

struct hal_param_t {
    SHMFIELD(hal_param_t) next_ptr;		/* next parameter in linked list */
    SHMFIELD(void*) data_ptr;		/* offset of parameter value */
    SHMFIELD(hal_comp_t) owner_ptr;		/* component that owns this signal */
    SHMFIELD(hal_oldname_t) oldname;		/* old name if aliased, else zero */
    hal_type_t type;		/* data type */
    hal_param_dir_t dir;	/* data direction */
    char name[HAL_NAME_LEN + 1];	/* parameter name */
};

struct hal_funct_t {
    SHMFIELD(hal_funct_t) next_ptr;		/* next function in linked list */
    int uses_fp;		/* floating point flag */
    SHMFIELD(hal_comp_t) owner_ptr;		/* component that added this funct */
    int reentrant;		/* non-zero if function is re-entrant */
    int users;			/* number of threads using function */
    void *arg;			/* argument for function */
    void (*funct) (void *, long);	/* ptr to function code */
    hal_s32_t* runtime;	/* (pin) duration of last run, in CPU cycles */
    hal_s32_t maxtime;	/* (param) duration of longest run, in CPU cycles */
    hal_bit_t maxtime_increased;	/* on last call, maxtime increased */
    char name[HAL_NAME_LEN + 1];	/* function name */
};

struct hal_funct_entry_t {
    hal_list_t links;		/* linked list data */
    void *arg;			/* argument for function */
    void (*funct) (void *, long);	/* ptr to function code */
    SHMFIELD(hal_funct_t) funct_ptr;		/* pointer to function */
};

void list_init_entry(hal_list_t * entry);
hal_list_t *list_prev(hal_list_t * entry);
hal_list_t *list_next(hal_list_t * entry);
void list_add_after(hal_list_t * entry, hal_list_t * prev);
void list_add_before(hal_list_t * entry, hal_list_t * next);
hal_list_t *list_remove_entry(hal_list_t * entry);

extern hal_comp_t *halpr_find_comp_by_name(const char *name);
extern hal_thread_t *halpr_find_thread_by_name(const char *name);
extern hal_funct_t *halpr_find_funct_by_name(const char *name);
extern hal_comp_t *halpr_find_comp_by_id(int id);


#endif //HAL_PRIV_H
