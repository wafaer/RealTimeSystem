//
// Created by Administrator on 2025/8/20.
//

#ifndef RTAPI_USPACE_H
#define RTAPI_USPACE_H

#ifdef __linux__
#include <sys/fsuid.h>
#endif
#include <atomic>
#include <unistd.h>
#include <pthread.h>



inline void rtapi_timespec_add(timespec &result, const timespec &ta, const timespec &tb) {
    result.tv_sec = ta.tv_sec + tb.tv_sec;
    result.tv_nsec = ta.tv_nsec + tb.tv_nsec;
    if (result.tv_nsec >= 1000000000) {
        result.tv_sec++;
        result.tv_nsec -= 1000000000;
    }
}

inline bool rtapi_timespec_less(const struct timespec &ta, const struct timespec &tb) {
    if(ta.tv_sec < tb.tv_sec) return 1;
    if(ta.tv_sec > tb.tv_sec) return 0;
    return ta.tv_nsec < tb.tv_nsec;
}

void rtapi_timespec_advance(struct timespec &result, const struct timespec &src, unsigned long nsec);

// ---------------------------------------------------------------------------
// Real-time task lifecycle state machine.
//
// Every rtapi_task transitions through one of:
//
//   EMPTY -> INIT (CAS) -> ALLOCATED -> RUNNING <-> PAUSED
//                            |            |            |
//                            v            v            v
//                         DELETED      DELETED      DELETED
//
// - EMPTY:     Slot is free; task_array[n] == NULL.
// - INIT:      CAS placeholder (TASK_MAGIC_INIT) set by allocate_task_id().
// - ALLOCATED: Object constructed, magic written, ready to start.
// - RUNNING:   pthread created and executing taskcode/rtapi_wait loop.
// - PAUSED:    Task blocked on pause semaphore, awaiting resume.
// - DELETED:   Thread cancelled and joined, object freed, slot released.
// - ERROR:     Non-recoverable failure.
// ---------------------------------------------------------------------------
enum {
  TASK_S_EMPTY     = 0,   // Slot free, no task allocated.
  TASK_S_INIT      = 1,   // CAS placeholder (TASK_MAGIC_INIT).
  TASK_S_ALLOCATED = 2,   // Object constructed, magic written.
  TASK_S_RUNNING   = 3,   // pthread active and executing.
  TASK_S_PAUSED    = 4,   // Task blocked on pause semaphore.
  TASK_S_DELETED   = 5,   // Task cancelled, object freed, slot released.
  TASK_S_ERROR     = -1   // Non-recoverable error.
};

struct WithRoot
{
    WithRoot();
    ~WithRoot();
    static std::atomic<int> level;
};

struct rtapi_task
{
  rtapi_task();

  int magic;			/* to check for valid handle */
  int id;
  int owner;
  int uses_fp;
  size_t stacksize;
  int prio;
  long period;
  struct timespec nextstart;
  unsigned ratio;
  long pll_correction;
  long pll_correction_limit;
  void *arg;
  void (*taskcode) (void*);	/* pointer to main function */

  // --- Fields added for lifecycle management (Google-style) ---
  int state;             // Current lifecycle state; one of the TASK_S_*
                         // enum values.  Initialised to TASK_S_EMPTY by
                         // rtapi_task constructor.
  pid_t owner_pid;       // PID of the process that created this task.
                         // Set in task_new().
  int error_code;        // Last errno value captured on a failed OS
                         // operation; 0 if no error has occurred.
};

struct RtapiApp
{
    RtapiApp(int policy = SCHED_FIFO) : policy(policy), period(0) {}

    virtual int prio_highest() const;
    virtual int prio_lowest() const;
    int prio_higher_delta() const;
    int prio_bound(int prio) const;
    int prio_next_higher(int prio) const;
    int prio_next_lower(int prio) const;
    long clock_set_period(long int period_nsec);
    int task_new(void (*taskcode)(void*), void *arg,
            int prio, int owner, unsigned long int stacksize, int uses_fp);
    virtual rtapi_task *do_task_new() = 0;
    static int allocate_task_id();
    static struct rtapi_task *get_task(int task_id);
    void unexpected_realtime_delay(rtapi_task *task, int nperiod=1);
    virtual int task_delete(int id) = 0;
    virtual int task_start(int task_id, unsigned long period_nsec) = 0;
    virtual int task_pause(int task_id) = 0;
    virtual int task_resume(int task_id) = 0;
    virtual int task_self() = 0;
    virtual long long task_pll_get_reference(void) = 0;
    virtual int task_pll_set_correction(long value) = 0;
    virtual int task_pll_get_correction(long *value) = 0;
    virtual void wait() = 0;
    virtual int run_threads(int fd, int (*callback)(int fd)) = 0;
    virtual long long do_get_time(void) = 0;
    virtual void do_delay(long ns) = 0;
    int policy;
    long period;
};

template<class T=rtapi_task>
T *rtapi_get_task(int task_id) {
    return static_cast<T*>(RtapiApp::get_task(task_id));
}


#define MAX_TASKS  64
#define TASK_MAGIC    21979	/* random numbers used as signatures */
#define TASK_MAGIC_INIT   ((rtapi_task*)(-1))

extern struct rtapi_task *task_array[MAX_TASKS];

#define WITH_ROOT WithRoot root

#endif //RTAPI_USPACE_H
