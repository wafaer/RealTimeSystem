#include "task.h"
#include "rtapi/rtapi.h"
#include <math.h>

void taskController(void *arg, long period)
{
    (void)arg;
    float pos_err;
    float period_sec;
    float sim_output;

    if (!taskShared || !taskHalData) return;

    period_sec = (float)period * 1e-9f;

    rtapi_mutex_get(&taskShared->cmd_mutex);

    // ---- 1. Trajectory planning ----
    simple_tp_update(&taskShared->tp, (int32_t)period);

    // ---- 2. PID position control ----
    pos_err = taskShared->tp.curr_pos - taskShared->actual_pos;

    pid_update(&taskShared->pid, pos_err, period_sec);

    // ---- 3. Simulate motor dynamics ----
    // Clamp PID output to safe range.
    sim_output = taskShared->pid.output;
    if (sim_output >  100.0f) sim_output =  100.0f;
    if (sim_output < -100.0f) sim_output = -100.0f;

    // Simple first-order plant: actual_vel = output, actual_pos integrates vel.
    taskShared->actual_pos += sim_output * period_sec;

    // ---- 4. Write outputs back to HAL pins ----
    *(hal_float_t *)taskHalData->planned_pos   = taskShared->tp.curr_pos;
    *(hal_float_t *)taskHalData->planned_vel   = taskShared->tp.curr_vel;
    *(hal_float_t *)taskHalData->actual_pos_out = taskShared->actual_pos;
    *(hal_float_t *)taskHalData->pid_out        = taskShared->pid.output;

    // Diagnostic counters.
    taskShared->cycle_count++;
    taskShared->heartbeat++;

    rtapi_mutex_give(&taskShared->cmd_mutex);
}
