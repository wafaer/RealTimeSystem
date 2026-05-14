#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <limits.h>
#include <stdio.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rtapi/rtapi_mutex.h"
#include "hal/hal.h"

// Default cycle times in nanoseconds.
#define DEFAULT_TASK_PERIOD_NSEC   1000000   // 1 ms

// Maximum number of supported worker items.
#define MAX_AXES  4

// Shared memory key for the task command buffer.
#define TASK_SHM_KEY     0x54534B31    // "TSK1"

// Command IDs for the command channel.
typedef enum {
    TASK_CMD_NONE        = 0,   // No pending command.
    TASK_CMD_SET_TARGET  = 1,   // Set a new target value.
    TASK_CMD_ENABLE      = 2,   // Enable the controller.
    TASK_CMD_DISABLE     = 3,   // Disable the controller.
    TASK_CMD_PAUSE       = 4,   // Pause the real-time thread.
    TASK_CMD_RESUME      = 5,   // Resume the real-time thread.
    TASK_CMD_SET_PLL     = 6,   // Adjust PLL correction value.
    TASK_CMD_SHUTDOWN    = 7    // Graceful shutdown request.
} task_command_id_t;

// Task runtime state.
typedef enum {
    TASK_STATE_UNINIT    = 0,
    TASK_STATE_IDLE      = 1,
    TASK_STATE_RUNNING   = 2,
    TASK_STATE_PAUSED    = 3,
    TASK_STATE_ERROR     = 4
} task_runtime_state_t;

// Shared memory layout — placed in the System V segment identified by
// TASK_SHM_KEY.  Both the real-time thread and the user-space control
// side access this structure concurrently, synchronised via command_mutex
// and the head/tail ring-buffer protocol.
typedef struct {
    rtapi_mutex_t command_mutex;     // Protects the command channel.
                                     // Realtime side: rtapi_mutex_try().
                                     // User side: rtapi_mutex_get().
    int           head;              // Producer index (user side).
    int           tail;              // Consumer index (realtime side).
                                     // head == tail => no pending command.

    task_command_id_t command_id;    // Active command ID.
    int               command_data;  // Integer parameter for the command.

    int           heartbeat;         // Controller heartbeat counter.
    int           cycle_count;       // Total cycles executed.
    long long     last_wakeup_ns;    // Last rtapi_wait() return timestamp.
} task_shared_t;

// HAL pin / parameter data — allocated via hal_malloc().
typedef struct {
    // Input pins (written externally, read by controller).
    hal_bit_t   *enable_in;
    hal_s32_t   *target_pos;
    hal_float_t *feed_rate;

    // Output pins (written by controller, read externally).
    hal_s32_t   *current_pos;
    hal_bit_t   *running;
    hal_bit_t   *error;
    hal_s32_t   *heartbeat_out;

    // Parameters (read/write, persistent).
    hal_s32_t    max_velocity;
    hal_s32_t    max_accel;
} task_hal_data_t;

// Global variables (defined in task.c).
extern int mot_comp_id;
extern int emc_shmem_id;
extern task_shared_t *taskShared;
extern task_hal_data_t *taskHalData;
extern int num_axis;

// Public API (implemented in task.c).
int task_thread_main(void);
void task_thread_exit(void);
void task_config_change(void);
void taskSetCycleTime(unsigned long nsec);

// Realtime functions (registered with HAL).
extern void taskController(void *arg, long period);
extern void taskCommandHandler(void *arg, long period);

#ifdef __cplusplus
}
#endif

#endif //TASK_H
