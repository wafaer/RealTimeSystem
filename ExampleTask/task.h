#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <limits.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rtapi/rtapi_mutex.h"
#include "hal/hal.h"
#include "simple_tp.h"

// Default cycle time in nanoseconds (1 ms).
#define DEFAULT_TASK_PERIOD_NSEC   1000000

// Shared memory key for the task.
#define TASK_SHM_KEY     0x54534B31    // "TSK1"

// Header sentinels — written by the creator into the first bytes of the
// segment so a later instance can tell whether an existing segment is one of
// ours and whether the original creator is still alive.
#define TASK_SHM_MAGIC    0x5453484Du  // "TSHM"
#define TASK_SHM_VERSION  1u

// PID gains limits
#define PID_KP_MIN  0.0f
#define PID_KP_MAX  10000.0f
#define PID_KI_MIN  0.0f
#define PID_KI_MAX  1000.0f
#define PID_KD_MIN  0.0f
#define PID_KD_MAX  10000.0f
#define PID_OUT_MAX  1000.0f
#define PID_OUT_MIN  -1000.0f

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float output;
} pid_ctrl_t;

static inline void pid_init(pid_ctrl_t *p)
{
    p->kp = 0.0f; p->ki = 0.0f; p->kd = 0.0f;
    p->prev_error = 0.0f; p->integral = 0.0f; p->output = 0.0f;
}

static inline void pid_update(pid_ctrl_t *p, float error, float period_sec)
{
    float p_term, d_term;
    p->integral += error * period_sec;
    p_term = p->kp * error;
    d_term = (period_sec > 0.0f) ? p->kd * (error - p->prev_error) / period_sec : 0.0f;
    p->output = p_term + p->ki * p->integral + d_term;
    p->prev_error = error;
}

static inline void pid_reset(pid_ctrl_t *p)
{
    p->prev_error = 0.0f;
    p->integral  = 0.0f;
    p->output    = 0.0f;
}

// Shared memory — accessible from both HAL thread and external components.
typedef struct {
    // ---- Header (must stay first; used for orphan-segment detection) ----
    uint32_t      shm_magic;       // TASK_SHM_MAGIC when initialised.
    uint32_t      shm_version;     // TASK_SHM_VERSION.
    pid_t         creator_pid;     // getpid() of creator at init time.

    rtapi_mutex_t cmd_mutex;

    simple_tp_t  tp;
    pid_ctrl_t   pid;

    float        actual_pos;    // simulated encoder position
    float        pid_output;    // torque/voltage command output

    int          heartbeat;
    int          cycle_count;
    long long    last_wakeup_ns;
} task_shared_t;

// HAL pins and parameters — stored in HAL shmem via hal_malloc.
typedef struct {
    // Input pins (written by external HAL components)
    void *enable;       // HAL_BIT  — overall enable
    void *pos_cmd;      // HAL_FLOAT — target position command
    void *vel_cmd;      // HAL_FLOAT — max velocity (runtime parameter)

    // Output pins (read by external HAL components)
    void *planned_pos;  // HAL_FLOAT — trajectory planned position
    void *planned_vel;  // HAL_FLOAT — trajectory planned velocity
    void *actual_pos_out; // HAL_FLOAT — actual (simulated) position
    void *pid_out;      // HAL_FLOAT — PID output

    // HAL parameters (read/write from external components)
    hal_float_t *tp_vel;       // max trajectory velocity
    hal_float_t *tp_acc;       // max trajectory acceleration
    hal_float_t *pid_kp;       // PID proportional gain
    hal_float_t *pid_ki;       // PID integral gain
    hal_float_t *pid_kd;       // PID derivative gain
    hal_s32_t   *heartbeat_out;
    hal_s32_t   *cycle_count_out;
} task_hal_data_t;

extern int mot_comp_id;
extern int emc_shmem_id;
extern task_shared_t *taskShared;
extern task_hal_data_t *taskHalData;

int  task_thread_main(void);
void task_thread_exit(void);
void taskSetCycleTime(unsigned long nsec);

extern void taskCommandHandler(void *arg, long period);
extern void taskController(void *arg, long period);

#ifdef __cplusplus
}
#endif

#endif // TASK_H
