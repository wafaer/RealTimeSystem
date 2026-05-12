//
// Created by skeqi on 25-8-30.
//

#include "rtapi/uspace_common.h"
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include "hal/hpsocket.h"
#include "rtapi/rtapi.h"

extern uid_t ruid;
extern RING_BUFF_t UsartTxRingBuff;
extern sem_t sem_count_usart_tx;

static msg_level_t msg_level = RTAPI_MSG_ALL;

void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  default_rtapi_msg_handler(level, fmt, args);
  va_end(args);
}

void print_msg(msg_level_t level, const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  default_msg_handler(level, fmt, args);
  va_end(args);
}

int rt_snprintf(char *buffer, unsigned long int size, const char *msg, ...)
{
  va_list args;
  int result;

  va_start(args, msg);
  /* call the normal library vnsprintf() */
  result = vsnprintf(buffer, size, msg, args);
  va_end(args);
  return result;
}

int rt_vsnprintf(char *buffer, unsigned long int size, const char *fmt,va_list args)
{
  return vsnprintf(buffer, size, fmt, args);
}


int rtapi_shmem_new(int key, int module_id, unsigned long int size)
{
  (void)module_id;
  rtapi_shmem_handle *shmem;
  int i;

  //shmem_array 存储所有共享内存句柄
  for (i=0,shmem=0 ; i < MAX_SHM; i++)
    {
      if(shmem_array[i].magic == SHMEM_MAGIC) {
        if (shmem_array[i].key == key) {
          shmem_array[i].count ++;
          return i;
        }
      }
      else if (!shmem) {
        shmem = &shmem_array[i];
      }
  }
  if (!shmem) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_shmem_new failed due to MAX_SHM\n");
    return -ENOMEM;
  }

  /* now get shared memory block from OS */
  int shmget_retries = 5;
  shmget_again:
  //获取共享内存
  shmem->id = shmget((key_t) key, (int) size, IPC_CREAT | 0600);
  if (shmem->id == -1) {
      // See below for explanation of why retry against -EPERM here
      if(shmget_retries-- && errno == -EPERM) {
          sched_yield();
          goto shmget_again;
      }
    rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_shmem_new failed due to shmget(key=0x%08x): %s\n", key, strerror(errno));
    return -errno;
  }

  struct shmid_ds stat;
  int res = shmctl(shmem->id, IPC_STAT, &stat);
  if(res < 0) perror("shmctl IPC_STAT");

  if(geteuid() == 0) {
    stat.shm_perm.uid = ruid;
    res = shmctl(shmem->id, IPC_SET, &stat);
    if(res < 0) perror("shmctl IPC_SET");
  }


  /* and map it into process space */
  //将共享内存映射到当前进程地址空间
  shmem->mem = shmat(shmem->id, 0, 0);
  if ((ssize_t) (shmem->mem) == -1) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_shmem_new failed due to shmat()\n");
    return -errno;
  }

  //触碰每一页内存，防止在映射时产生懒加载延迟，确保内存实际分配
  long pagesize = sysconf(_SC_PAGESIZE);
  /* touch every page */
  for(size_t off = 0; off < size; off += pagesize)
  {
      volatile char i = ((char*)shmem->mem)[off];
      (void)i;
  }

  /* label as a valid shmem structure */
  shmem->magic = SHMEM_MAGIC;
  /* fill in the other fields */
  shmem->size = size;
  shmem->key = key;
  shmem->count = 1;

  /* return handle to the caller */
  return shmem - shmem_array;
}


int rtapi_shmem_getptr(int handle, void **ptr)
{
  rtapi_shmem_handle *shmem;
  if(handle < 0 || handle >= MAX_SHM)
    return -EINVAL;

  shmem = &shmem_array[handle];

  /* validate shmem handle */
  if (shmem->magic != SHMEM_MAGIC)
    return -EINVAL;

  /* pass memory address back to caller */
  *ptr = shmem->mem;
  return 0;
}


int rtapi_shmem_delete(int handle, int module_id)
{
  struct shmid_ds d;
  int r1, r2;
  rtapi_shmem_handle *shmem;
  (void)module_id;

  if(handle < 0 || handle >= MAX_SHM)
    return -EINVAL;

  shmem = &shmem_array[handle];

  /* validate shmem handle */
  if (shmem->magic != SHMEM_MAGIC)
    return -EINVAL;

  shmem->count --;
  if(shmem->count) return 0;

  /* unmap the shared memory */
  r1 = shmdt(shmem->mem);

  /* destroy the shared memory */
  r2 = shmctl(shmem->id, IPC_STAT, &d);
  if (r2 != 0)
      rtapi_print_msg(RTAPI_MSG_ERR, "shmctl(%d, IPC_STAT, ...): %s\n", shmem->id, strerror(errno));

  if(r2 == 0 && d.shm_nattch == 0) {
      r2 = shmctl(shmem->id, IPC_RMID, &d);
      if (r2 != 0)
	      rtapi_print_msg(RTAPI_MSG_ERR, "shmctl(%d, IPC_RMID, ...): %s\n", shmem->id, strerror(errno));
  }

  /* free the shmem structure */
  shmem->magic = 0;

  if ((r1 != 0) || (r2 != 0))
    return -EINVAL;
  return 0;
}

long long rtapi_get_clocks(void)
{
  long long int retval;

  rdtscll(retval);
  return retval;
}

//初始化一个基于共享内存的 UUID生成器，并返回一个唯一的 ID 值
int rtapi_init(const char *modname)
{
  (void)modname;
  static uuid_data_t* uuid_data   = 0;
  static const   int  uuid_id     = 0;

  //共享内存的基地址
  static char* uuid_shmem_base = 0;
  int retval,id;
  void *uuid_mem;//用来存放共享内存的实际

  //创建一个共享内存段
  uuid_mem_id = rtapi_shmem_new(UUID_KEY,uuid_id,sizeof(uuid_data_t));
  if (uuid_mem_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
    "rtapi_init: could not open shared memory for uuid\n");
    rtapi_exit(uuid_id);
    return -EINVAL;
  }

  //获取共享内存的 实际指针地址，存放到 uuid_mem 中
  retval = rtapi_shmem_getptr(uuid_mem_id,&uuid_mem);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
    "rtapi_init: could not access shared memory for uuid\n");
    rtapi_exit(uuid_id);
    return -EINVAL;
  }

  if (uuid_shmem_base == 0) {
    uuid_shmem_base =        (char *) uuid_mem;
    uuid_data       = (uuid_data_t *) uuid_mem;
  }

  rtapi_mutex_get(&uuid_data->mutex);
  uuid_data->uuid++;
  id = uuid_data->uuid;
  rtapi_mutex_give(&uuid_data->mutex);

  return id;
}

int rtapi_exit(int module_id)
{
  rtapi_shmem_delete(uuid_mem_id, module_id);
  return 0;
}
