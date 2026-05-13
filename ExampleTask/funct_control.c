//
// funct_control.c — real-time controller for the ExampleTask.
//
// Executed once per cycle by the example-thread, after the command handler.
// Reads HAL input pins (enable-in, target-pos, feed-rate), performs a simple
// position-tracking control loop, and writes HAL output pins (current-pos,
// running, error, heartbeat-out).
//
// All pin accesses are direct memory reads/writes on the HAL shared memory
// segment — no locking is needed because the real-time thread is the sole
// writer of the output pins and the sole reader of the input pins during
// normal operation.
//
#include "task.h"
#include "rtapi/rtapi.h"

// Internal controller state.
static int  last_target  = 0;     // Target position from the previous cycle.
static int  integral     = 0;     // Simple integral accumulator.
static int  error_count  = 0;     // Consecutive error counter.

// ============================================================================
// taskController — real-time entry point registered with HAL as
// "example-controller".
//
// Parameters:
//   arg:    unused (NULL).
//   period: current thread period in nanoseconds.
// ============================================================================
void taskController(void *arg, long period)
{
    (void)arg;

    // Guard: HAL data must be initialised.
    if (taskHalData == NULL) return;

    // --- Read input pins ---
    int      enable = *(taskHalData->enable_in);
    int      target = *(taskHalData->target_pos);
    float    rate   = *(taskHalData->feed_rate);
    int      pos    = *(taskHalData->current_pos);
    int      max_v  = (int)taskHalData->max_velocity;
    (void)period;

    // --- Controller logic ---
    if (enable) {
        // Clamp the feed rate to a reasonable range.
        if (rate < 0.0f) rate = 0.0f;
        if (rate > 2.0f) rate = 2.0f;

        // Compute the position step for this cycle:  max_velocity * rate.
        // In a real system this would be scaled by period_nsec / 1e9.
        // Here we use a fixed step for simplicity of demonstration.
        int step = (int)((float)max_v * rate * 0.001f);
        if (step < 1) step = 1;

        // Simple proportional controller:  move toward the target.
        if (pos < target) {
            pos += step;
            if (pos > target) pos = target;
        } else if (pos > target) {
            pos -= step;
            if (pos < target) pos = target;
        }

        // Detect a target change for logging.
        if (target != last_target) {
            rtapi_print_msg(RTAPI_MSG_DBG,
                "task: controller → target changed %d → %d\n",
                last_target, target);
            last_target = target;
        }

        // Update the output pin.
        *(taskHalData->current_pos) = (hal_s32_t)pos;

        // Running flag: set when enabled.
        *(taskHalData->running) = 1;

        // Clear error flag and reset error tracking.
        *(taskHalData->error)  = 0;
        error_count = 0;

    } else {
        // Controller is disabled: hold position, signal not-running.
        *(taskHalData->running) = 0;

        // Latch an error if the target changed while disabled
        // (indicates a misconfiguration).
        if (target != last_target) {
            error_count++;
            if (error_count > 10) {
                *(taskHalData->error) = 1;
                rtapi_print_msg(RTAPI_MSG_ERR,
                    "task: controller → error latched (target changed "
                    "while disabled: %d → %d)\n", last_target, target);
            }
        }
    }

    // --- Update shared-memory heartbeat and cycle counter ---
    if (taskShared) {
        taskShared->heartbeat++;
        taskShared->cycle_count++;
    }

    // --- Mirror the heartbeat to the HAL output pin for monitoring ---
    *(taskHalData->heartbeat_out) = (taskShared ? taskShared->heartbeat : 0);

    // --- Update the last wakeup timestamp ---
    if (taskShared) {
        taskShared->last_wakeup_ns = rtapi_get_time();
    }
}
