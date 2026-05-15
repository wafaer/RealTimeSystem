//
// Created by skeqi on 25-8-30.
//

#include "rtapi/uspace_common.h"
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include "rtapi/rtapi.h"

extern uid_t ruid;
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


// ============================================================================
// rtapi_shmem_new — allocates a System V shared memory segment and maps it
// into the calling process.
//
// Lifecycle transitions on success:
//   UNBORN -> CREATED (shmget) -> ATTACHED (shmat) -> ACTIVE (page-touch)
//
// On any failure, previously-acquired OS resources are rolled back via the
// unified cleanup label at the end of the function.  The handle state is
// restored to UNBORN and the slot is released.
//
// Parameters:
//   key:       System V IPC key.  Must be non-zero.
//   module_id: RTAPI module identifier (unused in userspace; preserved for
//              kernel compatibility).
//   size:      Desired segment size in bytes.  Must be > 0.
//
// Returns:
//   >= 0:  Handle index into shmem_array (the same handle used by
//          rtapi_shmem_getptr / rtapi_shmem_delete).
//   < 0:   Negated errno value (e.g. -ENOMEM, -EACCES, -ENOSPC).
// ============================================================================
int rtapi_shmem_new(int key, int module_id, unsigned long int size)
{
  (void)module_id;
  rtapi_shmem_handle *shmem = NULL;
  int i;
  int saved_errno = 0;
  long pagesize;

  // Phase 1 — key sanity check.  A zero key is indistinguishable from
  // IPC_PRIVATE and would create a new, unreachable segment on every call.
  if (key == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "rtapi_shmem_new: key must be non-zero\n");
    return -EINVAL;
  }
  if (size == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "rtapi_shmem_new: size must be > 0\n");
    return -EINVAL;
  }

  // Phase 2 — walk the handle array for an existing match or a free slot.
  // shmem_array is process-local (static storage), so no locking is needed.
  for (i = 0, shmem = NULL; i < MAX_SHM; i++) {
    if (shmem_array[i].magic == SHMEM_MAGIC) {
      if (shmem_array[i].key == key) {
        // Already mapped in this process — just bump the reference count.
        shmem_array[i].count++;
        return i;
      }
    } else if (!shmem) {
      shmem = &shmem_array[i];
    }
  }
  if (!shmem) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "rtapi_shmem_new failed due to MAX_SHM\n");
    return -ENOMEM;
  }

  // Record the PID of the creator for cross-process diagnostics.
  shmem->owner_pid = getpid();
  shmem->error_code = 0;

  // Phase 3 — obtain the shared memory segment from the kernel.
  // Retry on transient EPERM (can happen when the segment was previously
  // created by root and the uid change has not yet propagated).
  {
    int shmget_retries = 5;
  shmget_again:
    shmem->id = shmget((key_t) key, (int) size, IPC_CREAT | 0600);
    if (shmem->id == -1) {
      if (shmget_retries-- && errno == EPERM) {
        sched_yield();
        goto shmget_again;
      }
      saved_errno = errno;
      shmem->error_code = saved_errno;
      rtapi_print_msg(RTAPI_MSG_ERR,
          "rtapi_shmem_new failed due to shmget(key=0x%08x): %s\n",
          key, strerror(saved_errno));
      // No OS resources to release yet — just return the error.
      return -saved_errno;
    }
  }
  // State transition: UNBORN -> CREATED.
  shmem->state = SHMEM_S_CREATED;

  // If the caller has root privileges (euid == 0), reassign ownership of
  // the segment to the real uid so that non-root processes can access it.
  {
    struct shmid_ds stat;
    int res = shmctl(shmem->id, IPC_STAT, &stat);
    if (res < 0) {
      // Non-fatal — we can proceed even if stat fails.
      perror("shmctl IPC_STAT");
    }

    if (geteuid() == 0) {
      stat.shm_perm.uid = ruid;
      res = shmctl(shmem->id, IPC_SET, &stat);
      if (res < 0) {
        perror("shmctl IPC_SET");
      }
    }
  }

  // Phase 4 — map the segment into the process address space.
  shmem->mem = shmat(shmem->id, NULL, 0);
  if ((ssize_t)(shmem->mem) == -1) {
    saved_errno = errno;
    shmem->error_code = saved_errno;
    rtapi_print_msg(RTAPI_MSG_ERR,
        "rtapi_shmem_new failed due to shmat(): %s\n",
        strerror(saved_errno));
    // Rollback: the segment was created (CREATED state), but we cannot
    // attach it.  Mark it for removal immediately.
    goto cleanup_created;
  }
  // State transition: CREATED -> ATTACHED.
  shmem->state = SHMEM_S_ATTACHED;

  // Phase 5 — touch every page of the mapping to force the kernel to
  // allocate physical backing store now rather than lazily on first
  // access.  This avoids unpredictable page-fault latency inside the
  // real-time control loop.
  //
  // The volatile read prevents the compiler from optimising the touch away.
  pagesize = sysconf(_SC_PAGESIZE);
  for (size_t off = 0; off < size; off += pagesize) {
    volatile char c = ((char *)shmem->mem)[off];
    (void)c;
  }

  // Phase 6 — fill in the handle metadata and transition to ACTIVE.
  shmem->magic = SHMEM_MAGIC;
  shmem->size  = size;
  shmem->key   = key;
  shmem->count = 1;
  shmem->state = SHMEM_S_ACTIVE;

  return shmem - shmem_array;

cleanup_created:
  // --- Rollback path ---
  // The segment is in CREATED state (shmget succeeded, but we never
  // successfully attached).  Mark it for kernel destruction via IPC_RMID.
  // The kernel will defer actual removal until no process is attached.
  {
    struct shmid_ds dummy;
    int rm_ret = shmctl(shmem->id, IPC_RMID, &dummy);
    if (rm_ret < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          "rtapi_shmem_new rollback: shmctl(IPC_RMID) failed: %s\n",
          strerror(errno));
    }
  }
  // Restore the slot to its initial state so it can be reused.
  shmem->state = SHMEM_S_ERROR;
  shmem->id    = -1;
  shmem->mem   = NULL;
  return -saved_errno;
}


// ============================================================================
// rtapi_shmem_getptr — retrieves the virtual address of a shared memory
// segment from a valid handle.
//
// The handle must be in ACTIVE or ATTACHED state.  Access to an ACTIVE
// handle is the normal case; ATTACHED is permitted as a grace state for
// callers that need the pointer before page-touch completes (e.g. during
// initialisation of the data structure at the start of the segment).
//
// Parameters:
//   handle:  Array index returned by rtapi_shmem_new().
//   ptr:     Output parameter; receives the virtual address.
//
// Returns:
//   0        on success.
//   -EINVAL  if handle is out of range or the magic number does not match.
//   -EACCES  if the segment is not in a mappable state.
// ============================================================================
int rtapi_shmem_getptr(int handle, void **ptr)
{
  rtapi_shmem_handle *shmem;

  if (handle < 0 || handle >= MAX_SHM)
    return -EINVAL;

  shmem = &shmem_array[handle];

  // Validate the magic number — the handle must have been returned by a
  // prior successful call to rtapi_shmem_new().
  if (shmem->magic != SHMEM_MAGIC)
    return -EINVAL;

  // State guard: only return a pointer when the segment is actually mapped.
  // UNBORN and DESTROYED slots have no mapping; ERROR slots may have been
  // partially initialised and should not be used.
  if (shmem->state != SHMEM_S_ACTIVE && shmem->state != SHMEM_S_ATTACHED) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "rtapi_shmem_getptr: segment in invalid state %d (key=0x%08x)\n",
        shmem->state, shmem->key);
    return -EACCES;
  }

  *ptr = shmem->mem;
  return 0;
}


// ============================================================================
// rtapi_shmem_delete — decrements the reference count for a shared memory
// segment and, when the count reaches zero, detaches and optionally destroys
// the underlying OS segment.
//
// Lifecycle transitions when count -> 0:
//   ACTIVE -> DETACHED (shmdt) -> DESTROYED (shmctl IPC_RMID, if nattch==0)
//
// The function is idempotent with respect to OS operations:  repeated calls
// with count already at zero are safe and will simply return 0 after the
// magic number has been cleared.
//
// Parameters:
//   handle:    Array index returned by rtapi_shmem_new().
//   module_id: RTAPI module identifier (unused in userspace).
//
// Returns:
//   0        on success (count decremented, and cleanup performed if count
//            reached zero).
//   -EINVAL  if handle is out of range, magic number mismatch, or an OS
//            call failed during cleanup.
// ============================================================================
int rtapi_shmem_delete(int handle, int module_id)
{
  struct shmid_ds d;
  int r1 = 0, r2 = 0;
  rtapi_shmem_handle *shmem;
  (void)module_id;

  if (handle < 0 || handle >= MAX_SHM)
    return -EINVAL;

  shmem = &shmem_array[handle];

  // Validate the magic number.
  if (shmem->magic != SHMEM_MAGIC)
    return -EINVAL;

  // Decrement the reference count.  If other references to this segment
  // still exist within this process, return immediately without touching
  // the mapping or the OS segment.
  shmem->count--;
  if (shmem->count > 0)
    return 0;

  // --- count == 0: last reference in this process ---

  // Detach the segment if it is currently mapped.
  // States ACTIVE and ATTACHED both imply a valid mapping.
  if (shmem->state == SHMEM_S_ACTIVE || shmem->state == SHMEM_S_ATTACHED) {
    r1 = shmdt(shmem->mem);
    if (r1 < 0) {
      shmem->error_code = errno;
      rtapi_print_msg(RTAPI_MSG_ERR,
          "rtapi_shmem_delete: shmdt(%p) failed: %s\n",
          shmem->mem, strerror(errno));
    }
    // State transition: -> DETACHED (even if shmdt failed — the mapping
    // is no longer usable).
    shmem->state = SHMEM_S_DETACHED;
    shmem->mem   = NULL;
  }

  // Query the kernel for the number of remaining attachments.
  // If zero, the segment can be safely destroyed.
  r2 = shmctl(shmem->id, IPC_STAT, &d);
  if (r2 != 0) {
    shmem->error_code = errno;
    rtapi_print_msg(RTAPI_MSG_ERR,
        "shmctl(%d, IPC_STAT, ...): %s\n", shmem->id, strerror(errno));
  }

  if (r2 == 0 && d.shm_nattch == 0) {
    // No process is still attached — mark the segment for kernel removal.
    r2 = shmctl(shmem->id, IPC_RMID, &d);
    if (r2 != 0) {
      shmem->error_code = errno;
      rtapi_print_msg(RTAPI_MSG_ERR,
          "shmctl(%d, IPC_RMID, ...): %s\n", shmem->id, strerror(errno));
    }
    // State transition: -> DESTROYED.
    shmem->state = SHMEM_S_DESTROYED;
  }

  // Release the slot back to the free pool.
  shmem->magic = 0;

  if ((r1 != 0) || (r2 != 0))
    return -EINVAL;
  return 0;
}


// ============================================================================
// rtapi_shmem_getstate — returns the current lifecycle state of a shared
// memory handle.
//
// This is a diagnostic helper; it does not modify the handle.
//
// Parameters:
//   handle:  Array index returned by rtapi_shmem_new().
//
// Returns:
//   One of the SHMEM_S_* enum values.
//   SHMEM_S_ERROR if the handle is out of range or the magic number does
//   not match.
// ============================================================================
int rtapi_shmem_getstate(int handle)
{
  if (handle < 0 || handle >= MAX_SHM)
    return SHMEM_S_ERROR;

  rtapi_shmem_handle *shmem = &shmem_array[handle];
  if (shmem->magic != SHMEM_MAGIC)
    return SHMEM_S_ERROR;

  return shmem->state;
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
