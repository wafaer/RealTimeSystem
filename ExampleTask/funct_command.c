#include "task.h"
#include "rtapi/rtapi.h"

void taskCommandHandler(void *arg, long period)
{
    (void)arg;
    hal_bit_t enable;
    float pos_cmd, vel_cmd;
    pid_ctrl_t pid_snap;

    if (!taskShared || !taskHalData) return;

    enable = *(hal_bit_t *)taskHalData->enable;
    pos_cmd = *(hal_float_t *)taskHalData->pos_cmd;
    vel_cmd = *(hal_float_t *)taskHalData->vel_cmd;

    rtapi_mutex_get(&taskShared->cmd_mutex);

    taskShared->tp.enable = (int)enable;
    taskShared->tp.pos_cmd = pos_cmd;

    if (vel_cmd > 0.0f && vel_cmd <= 10000.0f) {
        taskShared->tp.vel = vel_cmd;
    }

    // Sync PID gains from HAL parameters into the shared-memory pid struct.
    pid_snap = taskShared->pid;  // snapshot for display
    taskShared->pid.kp = *taskHalData->pid_kp;
    taskShared->pid.ki = *taskHalData->pid_ki;
    taskShared->pid.kd = *taskHalData->pid_kd;

    if (taskHalData->heartbeat_out)
        *taskHalData->heartbeat_out = taskShared->heartbeat;
    if (taskHalData->cycle_count_out)
        *taskHalData->cycle_count_out = taskShared->cycle_count;

    rtapi_mutex_give(&taskShared->cmd_mutex);
}
