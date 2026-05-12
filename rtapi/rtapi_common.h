//
// Created by Administrator on 2025/8/15.
//

#ifndef RTAPI_COMMON_H
#define RTAPI_COMMON_H

#define RTAPI_KEY   0x90280A48	/* key used to open RTAPI shared memory */
#define RTAPI_MAGIC 0x12601409	/* magic number used to verify shmem */
#include "rtapi_mutex.h"

/* maximum number of various resources */
#define RTAPI_MAX_MODULES 64
#define RTAPI_MAX_TASKS   64
#define RTAPI_MAX_SHMEMS  32
#define RTAPI_MAX_SEMS    64
#define RTAPI_MAX_FIFOS   32
#define RTAPI_MAX_IRQS    16
#define RTAPI_NAME_LEN   31

typedef unsigned long rtapi_mutex_t;

typedef struct rt_task_struct {
	int opaque;
} RT_TASK;

typedef struct rt_semaphore {
	int opaque;
} SEM;

typedef enum {
	NO_MODULE = 0,
	REALTIME,
	USERSPACE
} mod_type_t;


typedef enum {
	EMPTY = 0,
	PAUSED,
	PERIODIC,
	FREERUN,
	ENDED
} task_state_t;

typedef struct {
	mod_type_t state;
	char name[RTAPI_NAME_LEN + 1];
} module_data;

typedef struct {
	task_state_t state;		/* main state */
	int prio;			/* priority */
	int owner;			/* owning module */
	void (*taskcode) (void *);	/* main code */
	void *arg;			/* main argument */
} task_data;

typedef struct {
	int key;			/* key to shared memory area */
	int rtusers;		/* number of realtime modules using block */
	int ulusers;		/* number of user processes using block */
	unsigned long size;		/* size of shared memory area */
	unsigned long bitmap[(RTAPI_MAX_SHMEMS / 8) + 1];	/* which modules are
							   using block */
} shmem_data;

typedef enum {
	UNUSED = 0,
	HAS_READER = 1,
	HAS_WRITER = 2,
	HAS_BOTH = 3
} fifo_state_t;			/* used as bitmasks */

typedef struct {
	fifo_state_t state;		/* main state */
	int key;			/* key to fifo */
	int reader;			/* module ID of reader */
	int writer;			/* module ID of writer */
	unsigned long int size;	/* size of fifo area */
} fifo_data;

typedef struct {
	int users;			/* number of modules using the semaphore */
	int key;			/* key to semaphore */
	unsigned long bitmap[(RTAPI_MAX_SEMS / 8) + 1];	/* which modules are
							   using sem */
} sem_data;

typedef struct {
	int irq_num;		/* IRQ number */
	int owner;			/* owning module */
	void (*handler) (void);	/* interrupt handler function */
} irq_data;

typedef struct {
	int magic;			/* magic number to validate data */
	int rev_code;		/* revision code for matching */
	rtapi_mutex_t mutex;	/* mutex against simultaneous access */
	int rt_module_count;	/* loaded RT modules */
	int ul_module_count;	/* running UL processes */
	int task_count;		/* main IDs in use */
	int shmem_count;		/* shared memory blocks in use */
	int sem_count;		/* semaphores in use */
	int fifo_count;		/* fifos in use */
	int irq_count;		/* interrupts hooked */
	int timer_running;		/* state of HW timer */
	int rt_cpu;			/* CPU to use for RT tasks */
	long int timer_period;	/* HW timer period */
	module_data module_array[RTAPI_MAX_MODULES + 1];	/* data for modules */
	task_data task_array[RTAPI_MAX_TASKS + 1];	/* data for tasks */
	shmem_data shmem_array[RTAPI_MAX_SHMEMS + 1];	/* data for shared
							   memory */
	sem_data sem_array[RTAPI_MAX_SEMS + 1];	/* data for semaphores */
	fifo_data fifo_array[RTAPI_MAX_FIFOS + 1];	/* data for fifos */
	irq_data irq_array[RTAPI_MAX_IRQS + 1];	/* data for hooked irqs */
} rtapi_data_t;


static unsigned int rev_code = 1;

static void init_rtapi_data(rtapi_data_t * data)
{
    int n, m;

    /* has the block already been initialized? */
    if (data->magic == RTAPI_MAGIC) {
	/* yes, nothing to do */
	return;
    }
    /* no, we need to init it, grab mutex unconditionally */
    rtapi_mutex_try(&(data->mutex));
    /* set magic number so nobody else init's the block */
    data->magic = RTAPI_MAGIC;
    /* set version code so other modules can check it */
    data->rev_code = rev_code;
    /* and get busy */
    data->rt_module_count = 0;
    data->ul_module_count = 0;
    data->task_count = 0;
    data->shmem_count = 0;
    data->sem_count = 0;
    data->fifo_count = 0;
    data->irq_count = 0;
    data->timer_running = 0;
    data->timer_period = 0;
    /* init the arrays */
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
    /* done, release the mutex */
    rtapi_mutex_give(&(data->mutex));
    return;
}

#endif //RTAPI_COMMON_H
