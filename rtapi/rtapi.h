//
// Created by Administrator on 2025/8/21.
//

#ifndef RTAPI_H
#define RTAPI_H
#include <stdio.h>
#include <bits/types/clockid_t.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RTAPI_MSG_NONE = 0,
    RTAPI_MSG_ERR,
    RTAPI_MSG_WARN,
    RTAPI_MSG_INFO,
    RTAPI_MSG_DBG,
    RTAPI_MSG_ALL
    } msg_level_t;

    typedef struct {
        msg_level_t level;
        char msg[1024-sizeof(msg_level_t)];
    }message_t ;

typedef void(*rtapi_msg_handler_t)(msg_level_t level, const char *fmt, va_list ap);

    extern void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
    __attribute__((format(printf,2,3)));

    extern void print_msg(msg_level_t level, const char *fmt, ...)
__attribute__((format(printf,2,3)));

extern int rt_snprintf(char *buf, unsigned long int size,
const char *fmt, ...)
    __attribute__((format(printf,3,4)));

    extern int rt_vsnprintf(char *buffer, unsigned long int size, const char *fmt,va_list args);

extern long int rtapi_clock_set_period(long int nsecs);

extern int rtapi_prio_highest(void);
extern int rtapi_prio_next_lower(int prio);

extern int rtapi_shmem_new(int key, int module_id,
   unsigned long int size);
extern int rtapi_shmem_delete(int handle, int module_id);
extern int rtapi_shmem_getptr(int handle, void **ptr);
extern int rtapi_shmem_getstate(int handle);
extern int rtapi_init(const char *modname);
extern int rtapi_exit(int module_id);
extern long long int rtapi_get_clocks(void);
extern int rtapi_task_pll_set_correction(long value);
extern int rtapi_task_pll_get_correction(long *value);
extern int rtapi_task_getstate(int task_id);
extern int rtapi_task_new(void (*taskcode) (void *), void *arg, int prio, int owner, unsigned long int stacksize, int uses_fp);
extern int rtapi_task_start(int task_id, unsigned long int period_nsec);
extern int create_thread(pthread_t *thread, void *(*func)(void *), void *arg, int priority, int detached);
extern void rtapi_wait(void);
    extern long long int rtapi_get_time(void);
extern int rtapi_task_pause(int task_id);
extern int rtapi_task_resume(int task_id);
extern int rtapi_task_delete(int task_id);
extern int rtapi_task_self(void);
extern long long rtapi_task_pll_get_reference(void);

    extern int printfMaster();
    extern int stopprintf();
    extern void default_rtapi_msg_handler(msg_level_t level, const char *fmt, va_list ap);
    extern void default_msg_handler(msg_level_t level, const char *fmt, va_list ap);
    extern int rtapi_clock_nanosleep(clockid_t clock_id, int flags,
        const struct timespec *prequest, struct timespec *remain,
        const struct timespec *pnow);

#ifdef __cplusplus
}
#endif

#endif //RTAPI_H
