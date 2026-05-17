//
// Created by Administrator on 2025/8/20.
//
#ifdef __linux__
#include <sys/fsuid.h>
#endif
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <signal.h>
#include <semaphore.h>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#ifdef HAVE_SYS_IO_H
#include <sys/io.h>
#endif
#include <sys/resource.h>
#include <sys/mman.h>
#ifdef __linux__
#include <malloc.h>
#include <sys/prctl.h>
#endif
#ifdef __FreeBSD__
#include <pthread_np.h>
#endif
#include <chrono>
#include <stdarg.h>
#include <thread>

#include "hal/hal.h"
#include <boost/lockfree/queue.hpp>
#include "rtapi_uspace.h"

std::atomic<int> WithRoot::level;
uid_t euid, ruid;

#include "rtapi/uspace_common.h"

boost::lockfree::queue<message_t, boost::lockfree::capacity<128>> rtapi_msg_queue;

WithRoot::WithRoot() {
    if(!level++) {
#ifdef __linux__
        setfsuid(euid);
#endif
    }
}

WithRoot::~WithRoot() {
    if(!--level) {
#ifdef __linux__
        setfsuid(ruid);
#endif
    }
}

namespace
{
    RtapiApp &App();

    static void set_namef(const char *fmt, ...) {
        char *buf = NULL;
        va_list ap;

        va_start(ap, fmt);
        if (vasprintf(&buf, fmt, ap) < 0) {
            va_end(ap);
            return;
        }
        va_end(ap);

        int res = pthread_setname_np(pthread_self(), buf);
        if (res) {
            fprintf(stderr, "pthread_setname_np() failed for %s: %d\n", buf, res);
        }
        free(buf);
    }
}

static pthread_t main_thread{};

struct rtapi_module {
  int magic;
};

#define MODULE_MAGIC  30812
#define SHMEM_MAGIC   25453

#define MAX_MODULES  64
#define MODULE_OFFSET 32768

rtapi_task::rtapi_task()
    : magic{}, id{}, owner{}, uses_fp{}, stacksize{}, prio{},
      period{}, nextstart{},
      ratio{}, pll_correction{}, pll_correction_limit{},
      arg{}, taskcode{},

      state(TASK_S_EMPTY),
      owner_pid(0),
      error_code(0)

{}

namespace
{
struct PosixTask : rtapi_task
{
    PosixTask() : rtapi_task{}, thr{}, pause_sem{}, resume_sem{}
    {
        sem_init(&pause_sem, 0, 0);
        sem_init(&resume_sem, 0, 0);
    }
    ~PosixTask()
    {
        sem_destroy(&pause_sem);
        sem_destroy(&resume_sem);
    }

    pthread_t thr;       // POSIX thread handle.
    sem_t pause_sem;     // Posted by task_pause() to request a pause;
                         // consumed by the task thread in rtapi_wait().
    sem_t resume_sem;    // Posted by task_resume() to signal the task
                         // thread to continue after a pause.
};

struct Posix : RtapiApp
{
    Posix(int policy = SCHED_FIFO) : RtapiApp(policy), do_thread_lock(policy != SCHED_FIFO) {
        pthread_once(&key_once, init_key);
        if(do_thread_lock) {
            pthread_once(&lock_once, init_lock);
        }
    }
    int task_delete(int id);
    int task_start(int task_id, unsigned long period_nsec);
    int task_pause(int task_id);
    int task_resume(int task_id);
    int task_self();
    long long task_pll_get_reference(void);
    int task_pll_set_correction(long value);
    int task_pll_get_correction(long *value);
    void wait();
    struct rtapi_task *do_task_new() {
        return new PosixTask;
    }
    int run_threads(int fd, int (*callback)(int fd));
    static void *wrapper(void *arg);
    bool do_thread_lock;

    static pthread_once_t key_once;
    static pthread_key_t key;
    static void init_key(void) {
        pthread_key_create(&key, NULL);
    }

    static pthread_once_t lock_once;
    static pthread_mutex_t thread_lock;
    static void init_lock(void) {
        pthread_mutex_init(&thread_lock, NULL);
    }

    long long do_get_time(void) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec * 1000000000LL + ts.tv_nsec;
    }

    void do_delay(long ns);
};

static RtapiApp *makeApp()
{

    rtapi_print_msg(RTAPI_MSG_ERR, "Note: Using POSIX realtime\n");
    return new Posix(SCHED_FIFO);
}


RtapiApp &App()
{
    static RtapiApp *app = makeApp();
    return *app;
}

}
/* data for all tasks */
struct rtapi_task *task_array[MAX_TASKS];


int RtapiApp::prio_highest() const
{
    return sched_get_priority_max(policy);
}

int RtapiApp::prio_lowest() const
{
  return sched_get_priority_min(policy);
}

int RtapiApp::prio_higher_delta() const {
    if(rtapi_prio_highest() > prio_lowest()) {
        return 1;
    }
    return -1;
}

int RtapiApp::prio_bound(int prio) const {
    if(rtapi_prio_highest() > prio_lowest()) {
        if (prio >= rtapi_prio_highest())
            return rtapi_prio_highest();
        if (prio < prio_lowest())
            return prio_lowest();
    } else {
        if (prio <= rtapi_prio_highest())
            return rtapi_prio_highest();
        if (prio > prio_lowest())
            return prio_lowest();
    }
    return prio;
}

int RtapiApp::prio_next_higher(int prio) const
{
    prio = prio_bound(prio);
    if(prio != rtapi_prio_highest())
        return prio + prio_higher_delta();
    return prio;
}

int RtapiApp::prio_next_lower(int prio) const
{
    prio = prio_bound(prio);
    if(prio != prio_lowest())
        return prio - prio_higher_delta();
    return prio;
}

int RtapiApp::allocate_task_id()
{
    for(int n=0; n<MAX_TASKS; n++)
    {
        rtapi_task **taskptr = &(task_array[n]);
        if(__sync_bool_compare_and_swap(taskptr, (rtapi_task*)0, TASK_MAGIC_INIT))
            return n;
    }
    return -ENOSPC;
}

// ============================================================================
// RtapiApp::task_new — allocates and initialises a real-time task structure.
//
// Lifecycle transition: EMPTY (CAS) -> INIT (CAS) -> ALLOCATED.
//
// The caller receives a task_id that can be passed to task_start().
// owner_pid is recorded for cross-process diagnostics.
// ============================================================================
int RtapiApp::task_new(void (*taskcode) (void*), void *arg,
        int prio, int owner, unsigned long int stacksize, int uses_fp) {
  /* check requested priority */
  if ((prio > rtapi_prio_highest()) || (prio < prio_lowest()))
  {
    return -EINVAL;
  }

  /* label as a valid main structure */
  int n = allocate_task_id();
  if(n < 0) return n;

  struct rtapi_task *task = do_task_new();
  if(stacksize < (1024*1024)) stacksize = (1024*1024);
  task->id = n;
  task->owner = owner;
  task->uses_fp = uses_fp;
  task->arg = arg;
  task->stacksize = stacksize;
  task->taskcode = taskcode;
  task->prio = prio;
  task->magic = TASK_MAGIC;
  task->owner_pid = getpid();
  task->error_code = 0;
  task->state = TASK_S_ALLOCATED;
  task_array[n] = task;

  /* and return handle to the caller */

  return n;
}

rtapi_task *RtapiApp::get_task(int task_id) {
    if(task_id < 0 || task_id >= MAX_TASKS) return NULL;
    /* validate main handle */
    rtapi_task *task = task_array[task_id];
    if(!task || task == TASK_MAGIC_INIT || task->magic != TASK_MAGIC)
        return NULL;

    return task;
}

void RtapiApp::unexpected_realtime_delay(rtapi_task *task, int /*nperiod*/) {
    static int printed = 0;
    if(!printed)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Unexpected realtime delay on main %d with period %ld\n"
                "This Message will only display once per session.\n"
                "Run the Latency Test and resolve before continuing.\n",
                task->id, task->period);
        printed = 1;
    }
}

int Posix::task_delete(int id)
{
  auto task = ::rtapi_get_task<PosixTask>(id);
  if(!task) return -EINVAL;

  // Guard: already deleted or never fully allocated.
  if (task->state == TASK_S_DELETED)
    return -EALREADY;
  if (task->state == TASK_S_EMPTY || task->state == TASK_S_INIT)
    return -EINVAL;

  // If the task is currently paused, wake it so pthread_cancel can take
  // effect at the next cancellation point (rtapi_wait → pthread_testcancel).
  if (task->state == TASK_S_PAUSED) {
    sem_post(&task->resume_sem);
  }

  pthread_cancel(task->thr);
  pthread_join(task->thr, 0);

  task->magic = 0;
  task->state = TASK_S_DELETED;
  task_array[id] = 0;

  // PosixTask destructor will sem_destroy the semaphores.
  delete task;

  return 0;
}

static int find_rt_cpu_number() {
    if(getenv("RTAPI_CPU_NUMBER")) return atoi(getenv("RTAPI_CPU_NUMBER"));

#ifdef __linux__
    cpu_set_t cpuset_orig;
    int r = sched_getaffinity(getpid(), sizeof(cpuset_orig), &cpuset_orig);
    if(r < 0)
        // if getaffinity fails, (it shouldn't be able to), just use CPU#0
        return 0;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    long top_probe = sysconf(_SC_NPROCESSORS_CONF);
    // in old glibc versions, it was an error to pass to sched_setaffinity bits
    // that are higher than an imagined/probed kernel-side CPU mask size.
    // this caused the message
    //     sched_setaffinity: Invalid argument
    // to be printed at startup, and the probed CPU would not take into
    // account CPUs masked from this process by default (whether by
    // isolcpus or taskset).  By only setting bits up to the "number of
    // processes configured", the call is successful on glibc versions such as
    // 2.19 and older.
    for(long i=0; i<top_probe && i<CPU_SETSIZE; i++) CPU_SET(i, &cpuset);

    r = sched_setaffinity(getpid(), sizeof(cpuset), &cpuset);
    if(r < 0)
        // if setaffinity fails, (it shouldn't be able to), go on with
        // whatever the default CPUs were.
        perror("sched_setaffinity");

    r = sched_getaffinity(getpid(), sizeof(cpuset), &cpuset);
    if(r < 0) {
        // if getaffinity fails, (it shouldn't be able to), copy the
        // original affinity list in and use it
        perror("sched_getaffinity");
        CPU_AND(&cpuset, &cpuset_orig, &cpuset);
    }

    int top = -1;
    for(int i=0; i<CPU_SETSIZE; i++) {
        if(CPU_ISSET(i, &cpuset)) top = i;
    }
    return top;
#else
    return (-1);
#endif
}

// ============================================================================
// Posix::task_start — creates the POSIX thread for a previously allocated
// task and transitions it to RUNNING state.
//
// State gate: only ALLOCATED tasks may be started.  Re-starting a RUNNING
// task returns -EALREADY.
//
// On failure, the state is left in ALLOCATED so that task_delete() can
// cleanly tear down the PosixTask (including semaphore destruction).
// ============================================================================
int Posix::task_start(int task_id, unsigned long int period_nsec)
{
  auto task = ::rtapi_get_task<PosixTask>(task_id);
  if(!task) return -EINVAL;

  // State gate: prevent double-start and invalid-state starts.
  if (task->state == TASK_S_RUNNING)
    return -EALREADY;
  if (task->state != TASK_S_ALLOCATED) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "task_start: task %d in state %d, expected ALLOCATED (%d)\n",
        task_id, task->state, TASK_S_ALLOCATED);
    return -EINVAL;
  }

//   if(period_nsec < (unsigned long)period) period_nsec = (unsigned long)period;
  task->period = period_nsec;
//   main->ratio = period_nsec / period;

  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = task->prio;

  // limit PLL correction values to +/-1% of cycle time
  task->pll_correction_limit = period_nsec / 100;
  task->pll_correction = 0;

  int nprocs = sysconf( _SC_NPROCESSORS_ONLN );

  pthread_attr_t attr;
  int ret;
  if((ret = pthread_attr_init(&attr)) != 0) {
      task->error_code = ret;
      return -ret;
  }
  if((ret = pthread_attr_setstacksize(&attr, task->stacksize)) != 0) {
      task->error_code = ret;
      pthread_attr_destroy(&attr);
      return -ret;
  }
  if((ret = pthread_attr_setschedpolicy(&attr, policy)) != 0) {
      task->error_code = ret;
      pthread_attr_destroy(&attr);
      return -ret;
  }
  if((ret = pthread_attr_setschedparam(&attr, &param)) != 0) {
      task->error_code = ret;
      pthread_attr_destroy(&attr);
      return -ret;
  }
  if((ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) != 0) {
      task->error_code = ret;
      pthread_attr_destroy(&attr);
      return -ret;
  }
  if(nprocs > 1) {
      const static int rt_cpu_number = find_rt_cpu_number();
    //   rtapi_print_msg(RTAPI_MSG_INFO, "rt_cpu_number %d\n", rt_cpu_number);
      if(rt_cpu_number != -1) {
#ifdef __FreeBSD__
          cpuset_t cpuset;
#else
          cpu_set_t cpuset;
#endif
          CPU_ZERO(&cpuset);
          CPU_SET(rt_cpu_number, &cpuset);
          if((ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset)) != 0) {
               task->error_code = ret;
               pthread_attr_destroy(&attr);
               return -ret;
          }
      }
  }

  // State transition: ALLOCATED -> RUNNING before pthread_create so that
  // task_pause/task_delete can safely reference the task during startup.
  task->state = TASK_S_RUNNING;

  if((ret = pthread_create(&task->thr, &attr, &wrapper, reinterpret_cast<void*>(task))) != 0) {
      // Rollback state on pthread_create failure.
      task->state = TASK_S_ALLOCATED;
      task->error_code = ret;
      pthread_attr_destroy(&attr);
      return -ret;
  }

  pthread_attr_destroy(&attr);
  return 0;
}

#define RTAPI_CLOCK (CLOCK_MONOTONIC)

pthread_once_t Posix::key_once = PTHREAD_ONCE_INIT;
pthread_once_t Posix::lock_once = PTHREAD_ONCE_INIT;
pthread_key_t Posix::key;
pthread_mutex_t Posix::thread_lock;

void *Posix::wrapper(void *arg)
{
  struct rtapi_task *task;

  /* use the argument to point to the main data */
  task = (struct rtapi_task*)arg;
//   long int period = App().period;
//   if(main->period < period) main->period = period;
//   main->ratio = main->period / period;
//   main->period = main->ratio * period;
    // main->period = period;
  rtapi_print_msg(RTAPI_MSG_INFO, "main %p period = %lu ratio=%u\n",
 	  task, task->period, task->ratio);

  pthread_setspecific(key, arg);
  set_namef("rtapi_app:T#%d", task->id);

  Posix &papp = reinterpret_cast<Posix&>(App());
  if(papp.do_thread_lock)
      pthread_mutex_lock(&papp.thread_lock);

  struct timespec now;
  clock_gettime(RTAPI_CLOCK, &now);
  rtapi_timespec_advance(task->nextstart, now, task->period + task->pll_correction);

  /* call the main function with the main argument */
  (task->taskcode) (task->arg);

  rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: reached end of wrapper for main %d\n", task->id);
  return NULL;
}

long long Posix::task_pll_get_reference(void) {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return 0;
    return task->nextstart.tv_sec * 1000000000LL + task->nextstart.tv_nsec;
}

int Posix::task_pll_set_correction(long value) {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return -EINVAL;
    if (value > task->pll_correction_limit) value = task->pll_correction_limit;
    if (value < -(task->pll_correction_limit)) value = -(task->pll_correction_limit);
    task->pll_correction = value;
    return 0;
}

// ============================================================================
// Posix::task_pll_get_correction — retrieves the current PLL correction
// value for the calling task thread.
//
// Must be called from within the task thread (uses TLS key lookup).
// ============================================================================
int Posix::task_pll_get_correction(long *value) {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return -EINVAL;
    *value = task->pll_correction;
    return 0;
}

// ============================================================================
// Posix::task_pause — requests a running task to pause at its next
// rtapi_wait() call.
//
// The external thread (typically the main control thread) posts the task's
// pause_sem.  The task thread, inside rtapi_wait(), detects the semaphore
// and blocks on resume_sem until task_resume() is called.
//
// Returns:
//   0         on success (pause request posted).
//   -EINVAL   if task_id is invalid or the task is not in RUNNING state.
// ============================================================================
int Posix::task_pause(int task_id) {
    auto task = ::rtapi_get_task<PosixTask>(task_id);
    if(!task) return -EINVAL;
    if(task->state != TASK_S_RUNNING) return -EINVAL;

    task->state = TASK_S_PAUSED;
    sem_post(&task->pause_sem);
    return 0;
}

// ============================================================================
// Posix::task_resume — wakes a paused task so it continues its real-time
// loop.
//
// Returns:
//   0         on success (resume signal posted).
//   -EINVAL   if task_id is invalid or the task is not in PAUSED state.
// ============================================================================
int Posix::task_resume(int task_id) {
    auto task = ::rtapi_get_task<PosixTask>(task_id);
    if(!task) return -EINVAL;
    if(task->state != TASK_S_PAUSED) return -EINVAL;

    sem_post(&task->resume_sem);
    // The task thread will set state back to RUNNING when it wakes up.
    return 0;
}

int Posix::task_self() {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return -EINVAL;
    return task->id;
}

// ============================================================================
// Posix::wait — the real-time cycle synchronisation point.
//
// Called by the task thread at the end of each iteration.  The thread:
//   1. Checks for a pause request (via pause_sem) and blocks if paused.
//   2. Checks for cancellation (pthread_testcancel).
//   3. Advances nextstart by period + pll_correction.
//   4. Sleeps until the absolute nextstart time.
// ============================================================================
void Posix::wait() {
    if(do_thread_lock)
        pthread_mutex_unlock(&thread_lock);
    pthread_testcancel();

    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));

    // Check for a pause request from an external thread.  sem_trywait is
    // non-blocking: it returns 0 if a token was consumed (pause requested),
    // or -1 with errno==EAGAIN if no pause request is pending.
    PosixTask *ptask = static_cast<PosixTask*>(task);
    if (sem_trywait(&ptask->pause_sem) == 0) {
        // State is already TASK_S_PAUSED (set by task_pause).  Block until
        // task_resume posts the resume semaphore.
        sem_wait(&ptask->resume_sem);
        // Back to running.
        ptask->state = TASK_S_RUNNING;
    }

    rtapi_timespec_advance(task->nextstart, task->nextstart, task->period + task->pll_correction);
    struct timespec now;
    clock_gettime(RTAPI_CLOCK, &now);
    if(rtapi_timespec_less(task->nextstart, now))
    {
        if(policy == SCHED_FIFO)
            unexpected_realtime_delay(task);
    }
    else
    {
        int res = rtapi_clock_nanosleep(RTAPI_CLOCK, TIMER_ABSTIME, &task->nextstart, nullptr, &now);
        if(res < 0) perror("clock_nanosleep");
    }
    if(do_thread_lock)
        pthread_mutex_lock(&thread_lock);
}

void Posix::do_delay(long ns) {
    struct timespec ts = {0, ns};
    rtapi_clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL, NULL);
}

int rtapi_prio_highest(void)
{
    return App().prio_highest();
}

int rtapi_prio_lowest(void)
{
    return App().prio_lowest();
}

int rtapi_prio_next_higher(int prio)
{
    return App().prio_next_higher(prio);
}

int rtapi_prio_next_lower(int prio)
{
    return App().prio_next_lower(prio);
}

long rtapi_clock_set_period(long nsecs)
{
    return App().clock_set_period(nsecs);
}

long RtapiApp::clock_set_period(long nsecs)
{
  if(nsecs == 0) return period;
  if(period != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "attempt to set period twice\n");
      return -EINVAL;
  }
  period = nsecs;
  return period;
}

int rtapi_task_new(void (*taskcode) (void*), void *arg,
        int prio, int owner, unsigned long int stacksize, int uses_fp) {
    return App().task_new(taskcode, arg, prio, owner, stacksize, uses_fp);
}

int rtapi_task_delete(int id) {
    return App().task_delete(id);
}

int rtapi_task_start(int task_id, unsigned long period_nsec)
{
    int ret = App().task_start(task_id, period_nsec);
    if(ret != 0) {
        errno = -ret;
        perror("rtapi_task_start()");
    }
    return ret;
}

int create_thread(pthread_t *thread, void *(*func)(void *), void *arg, int priority, int detached)
{
    pthread_attr_t attr;
    struct sched_param schedParam;
    int ret;
    ret = pthread_attr_init(&attr);
    if(ret !=0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "pthread_attr_init FAILED\n");
        pthread_attr_destroy(&attr);
        return ret;
    }
    
    ret = pthread_attr_setschedpolicy(&attr,SCHED_FIFO);
    if(ret != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "pthread_attr_setschedpolicy FAILED\n");
        pthread_attr_destroy(&attr);
        return ret;
    }

    schedParam.sched_priority = priority;
    ret = pthread_attr_setschedparam(&attr, &schedParam);
    if(ret != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "pthread_attr_setschedparam FAILED\n");
        pthread_attr_destroy(&attr);
        return ret;
    }

    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if(ret != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "pthread_attr_setinheritsched FAILED\n");
        pthread_attr_destroy(&attr);
        return ret;
    }

    ret = pthread_create(thread, &attr, func, arg);
    if(ret != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "pthread_create FAILED\n");
        pthread_attr_destroy(&attr);
        return ret;
    }

    pthread_attr_destroy(&attr);
    return ret;
}

int rtapi_task_pause(int task_id)
{
    return App().task_pause(task_id);
}

int rtapi_task_resume(int task_id)
{
    return App().task_resume(task_id);
}

int rtapi_task_self()
{
    return App().task_self();
}

long long rtapi_task_pll_get_reference(void)
{
    return App().task_pll_get_reference();
}

int rtapi_task_pll_set_correction(long value)
{
    return App().task_pll_set_correction(value);
}

// ============================================================================
// rtapi_task_pll_get_correction — thin C wrapper around the C++ PLL getter.
// ============================================================================
int rtapi_task_pll_get_correction(long *value)
{
    return App().task_pll_get_correction(value);
}

// ============================================================================
// rtapi_task_getstate — returns the current lifecycle state of a task.
//
// Returns one of the TASK_S_* enum values, or TASK_S_ERROR on invalid input.
// ============================================================================
int rtapi_task_getstate(int task_id)
{
    if (task_id < 0 || task_id >= MAX_TASKS)
        return TASK_S_ERROR;

    rtapi_task *task = task_array[task_id];
    if (!task || task->magic != TASK_MAGIC)
        return TASK_S_ERROR;

    return task->state;
}

void rtapi_wait(void)
{
    App().wait();
}

int Posix::run_threads(int fd, int(*callback)(int fd)) {
    while(callback(fd)) { /* nothing */ }
    return 0;
}

int sim_rtapi_run_threads(int fd, int (*callback)(int fd)) {
    return App().run_threads(fd, callback);
}

long long rtapi_get_time() {
    return App().do_get_time();
}

long int rtapi_delay_max() { return 10000; }

void rtapi_delay(long ns) {
    if(ns > rtapi_delay_max()) ns = rtapi_delay_max();
    App().do_delay(ns);
}

const unsigned long ONE_SEC_IN_NS = 1000000000;
void rtapi_timespec_advance(struct timespec &result, const struct timespec &src, unsigned long nsec)
{
    time_t sec = src.tv_sec;
    while(nsec >= ONE_SEC_IN_NS)
    {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    nsec += src.tv_nsec;
    if(nsec >= ONE_SEC_IN_NS)
    {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    result.tv_sec = sec;
    result.tv_nsec = nsec;
}


void *queue_function(void * /*arg*/)
{
    while(1) {
        pthread_testcancel();
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
        rtapi_msg_queue.consume_all([](const message_t &m) {
            FILE* out = (m.level == RTAPI_MSG_ALL) ? stdout : stderr;
            fputs(m.msg, out);
        });

        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return NULL;
}

pthread_t queue_thread;
int printfMaster()
{
    int result;
    if((result = pthread_create(&queue_thread, NULL, &queue_function, NULL)) != 0)
    {
        errno = result;
        printf("pthread_create (queue function) faild\n");
        return -1;
    }

    return result;
}

int stopprintf()
{
    if (queue_thread != NULL)
    {
        pthread_cancel(queue_thread);
        pthread_join(queue_thread, NULL);
        rtapi_msg_queue.consume_all([](const message_t &m) {
        fputs(m.msg, m.level == RTAPI_MSG_ALL ? stdout : stderr);});
    }

}

void default_rtapi_msg_handler(msg_level_t level, const char *fmt, va_list ap)
{
    message_t m;
    m.level = level;
    vsnprintf(m.msg, sizeof(m.msg), fmt, ap);
    rtapi_msg_queue.push(m);
}

void default_msg_handler(msg_level_t level, const char *fmt, va_list ap)
{
    message_t m;
    m.level = level;
    vsnprintf(m.msg, sizeof(m.msg), fmt, ap);
    FILE* out = (m.level == RTAPI_MSG_ALL) ? stdout : stderr;
    fputs(m.msg, out);
}


int rtapi_clock_nanosleep(clockid_t clock_id, int flags,
        const struct timespec *prequest, struct timespec *remain,
        const struct timespec *pnow)
{
    (void)pnow;
#if defined(HAVE_CLOCK_NANOSLEEP)
    return clock_nanosleep(clock_id, flags, prequest, remain);
#else
    if(flags == 0)
        return nanosleep(prequest, remain);
    if(flags != TIMER_ABSTIME)
    {
        errno = EINVAL;
        return -1;
    }
    struct timespec now;
    if(!pnow)
    {
        int res = clock_gettime(clock_id, &now);
        if(res < 0) return res;
        pnow = &now;
    }
#undef timespecsub
#define	timespecsub(tvp, uvp, vvp)					\
do {								\
 (vvp)->tv_sec = (tvp)->tv_sec - (uvp)->tv_sec;		\
 (vvp)->tv_nsec = (tvp)->tv_nsec - (uvp)->tv_nsec;	\
 if ((vvp)->tv_nsec < 0) {				\
 (vvp)->tv_sec--;				\
 (vvp)->tv_nsec += 1000000000;			\
 }							\
} while (0)
    struct timespec request;
    timespecsub(prequest, pnow, &request);
    return nanosleep(&request, remain);
#endif
}
