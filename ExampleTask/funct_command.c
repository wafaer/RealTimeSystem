//
// funct_command.c — real-time command handler for the ExampleTask.
//
// This function is executed once per real-time cycle (before the controller)
// by the example-thread.  It reads pending commands from the shared memory
// command channel protected by command_mutex, processes them, and clears
// the channel for the next command.
//
// Synchronisation: rtapi_mutex_try() is used on the real-time side so that
// the thread never blocks waiting for the user-space writer.  If the lock is
// held, the function returns immediately and retries on the next cycle.
//
#include <osal.h>
#include <stdlib.h>
#include <time.h>
#include "task.h"
#include "rtapi/rtapi.h"
#include "rtapi/rtapi_mutex.h"

// ============================================================================
// taskCommandHandler — real-time entry point registered with HAL as
// "example-cmd-handler".
//
// Parameters:
//   arg:    unused (NULL in the example).
//   period: current thread period in nanoseconds (passed by the HAL thread
//           dispatcher).
// ============================================================================
void taskCommandHandler(void *arg, long period)
{
    task_shared_t *shared = taskShared;
    (void)arg;
    (void)period;

    // Guard: shared memory must be initialised.
    if (shared == NULL) return;

    // Try to acquire the command channel mutex without blocking.
    // If another thread (the user-space control side) holds the lock,
    // skip processing this cycle — we will retry on the next iteration.
    if (rtapi_mutex_try(&shared->command_mutex) != 0) {
        return;
    }

    // Check whether a command is pending (head != tail).
    if (shared->head != shared->tail) {
        task_command_id_t cmd  = shared->command_id;
        int               data = shared->command_data;

        // Process the command.
        switch (cmd) {

        case TASK_CMD_SET_TARGET:
            // Write the target value to the HAL input pin.
            if (taskHalData && taskHalData->target_pos) {
                *(taskHalData->target_pos) = (hal_s32_t)data;
            }
            rtapi_print_msg(RTAPI_MSG_DBG,
                "task: CMD_SET_TARGET → %d\n", data);
            break;

        case TASK_CMD_ENABLE:
            // Enable the controller via the HAL input pin.
            if (taskHalData && taskHalData->enable_in) {
                *(taskHalData->enable_in) = 1;
            }
            rtapi_print_msg(RTAPI_MSG_DBG,
                "task: CMD_ENABLE\n");
            break;

        case TASK_CMD_DISABLE:
            // Disable the controller via the HAL input pin.
            if (taskHalData && taskHalData->enable_in) {
                *(taskHalData->enable_in) = 0;
            }
            rtapi_print_msg(RTAPI_MSG_DBG,
                "task: CMD_DISABLE\n");
            break;

        case TASK_CMD_PAUSE:
            // Forward the pause request to the RTAPI task layer.
            // The real-time thread will block at the next rtapi_wait() call.
            {
                int task_id = rtapi_task_self();
                if (task_id >= 0) {
                    rtapi_task_pause(task_id);
                    rtapi_print_msg(RTAPI_MSG_DBG,
                        "task: CMD_PAUSE → task %d\n", task_id);
                }
            }
            break;

        case TASK_CMD_RESUME:
            // Forward the resume request to the RTAPI task layer.
            {
                int task_id = rtapi_task_self();
                if (task_id >= 0) {
                    rtapi_task_resume(task_id);
                    rtapi_print_msg(RTAPI_MSG_DBG,
                        "task: CMD_RESUME → task %d\n", task_id);
                }
            }
            break;

        case TASK_CMD_SET_PLL:
            // Adjust the PLL correction value for the current task.
            rtapi_task_pll_set_correction((long)data);
            {
                long cur = 0;
                rtapi_task_pll_get_correction(&cur);
                rtapi_print_msg(RTAPI_MSG_DBG,
                    "task: CMD_SET_PLL → %ld (now %ld)\n",
                    (long)data, cur);
            }
            break;

        case TASK_CMD_SHUTDOWN:
            rtapi_print_msg(RTAPI_MSG_INFO,
                "task: CMD_SHUTDOWN received\n");
            // The user-space control loop will detect this and clean up.
            break;

        case TASK_CMD_NONE:
        default:
            break;
        }

        // Mark the command as consumed by advancing the consumer pointer.
        shared->tail = shared->head;
    }

    // Release the mutex so the user-space side can write the next command.
    rtapi_mutex_give(&shared->command_mutex);
}
