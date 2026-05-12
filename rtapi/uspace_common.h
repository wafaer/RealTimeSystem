//
// Created by Administrator on 2025/8/20.
//

#ifndef USPACE_COMMON_H
#define USPACE_COMMON_H


#include <errno.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <rtapi/rtapi_mutex.h>
#include "rtapi/rtapi.h"

typedef struct {
  int magic;			/* to check for valid handle */
  int key;			/* key to shared memory area */
  int id;			/* OS identifier for shmem */
  int count;                    /* count of maps in this process */
  unsigned long int size;	/* size of shared memory area */
  void *mem;			/* pointer to the memory */
} rtapi_shmem_handle;

#define MAX_SHM 64

#define SHMEM_MAGIC   25453	/* random numbers used as signatures */

static rtapi_shmem_handle shmem_array[MAX_SHM];

typedef struct {
    rtapi_mutex_t mutex;
    int           uuid;
} uuid_data_t;

static int  uuid_mem_id = 0;
#define UUID_KEY  0x48484c34

#define rdtscll(val) ((val) = rtapi_get_time())

#endif //USPACE_COMMON_H
