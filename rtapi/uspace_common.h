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
#include <unistd.h>
#include <rtapi/rtapi_mutex.h>
#include "rtapi/rtapi.h"

// ---------------------------------------------------------------------------
// Shared memory lifecycle state machine.
//
// Every rtapi_shmem_handle transitions through these states in order:
//
//   UNBORN -> CREATED -> ATTACHED -> ACTIVE
//                       |            |
//                     (ERROR)    DETACHED -> DESTROYED
//
// - UNBORN:    Slot is free; no OS resources allocated.
// - CREATED:   shmget() succeeded; the shared memory segment exists in the
//              kernel but is not yet mapped into this process.
// - ATTACHED:  shmat() succeeded; the segment is mapped, but the data region
//              has not yet been initialised (page-touch still pending).
// - ACTIVE:    Page-touch is complete and the magic number has been written;
//              the segment is safe for concurrent read/write.
// - DETACHED:  shmdt() has been called; the segment is no longer mapped in
//              this process, but it may still be mapped in others.
// - DESTROYED: shmctl(IPC_RMID) has been called; the kernel will reclaim the
//              segment once all attached processes have detached.
// - ERROR:     A non-recoverable error occurred during creation.
// ---------------------------------------------------------------------------
enum {
  SHMEM_S_UNBORN   = 0,   // Slot idle, no OS resources held.
  SHMEM_S_CREATED  = 1,   // shmget() succeeded, segment exists in kernel.
  SHMEM_S_ATTACHED = 2,   // shmat() succeeded, segment mapped but not yet
                          // touch-initialised.
  SHMEM_S_ACTIVE   = 3,   // Fully initialised and ready for use.
  SHMEM_S_DETACHED = 4,   // shmdt() called, local mapping released.
  SHMEM_S_DESTROYED = 5,  // shmctl(IPC_RMID) called (or reference fully
                          // released).
  SHMEM_S_ERROR    = -1   // Creation/operation failed unrecoverably.
};

typedef struct {
  int magic;                 // Magic number for handle validation
                             // (SHMEM_MAGIC).
  int key;                   // System V IPC key for the shared memory area.
  int id;                    // OS-level shm identifier returned by shmget().
  int count;                 // Number of times this segment has been mapped
                             // by this process (reference count).
  unsigned long int size;    // Size of the shared memory area in bytes.
  void *mem;                 // Virtual address of the mapping in this
                             // process (NULL if not mapped).

  // --- Fields added for lifecycle management (Google-style) ---
  int state;                 // Current lifecycle state; one of the
                             // SHMEM_S_* enum values above.
                             // Initialised to SHMEM_S_UNBORN by BSS zeroing.
  pid_t owner_pid;           // PID of the process that created this shmem
                             // segment. Useful for cross-process debugging.
  int error_code;            // Last errno value captured on a failed OS
                             // operation; 0 if no error has occurred.
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
