//
// ExampleTask — complete real-time task demonstration.
//
// This file implements the full lifecycle of an RTAPI / HAL task:
//   1. HAL component registration (hal_init).
//   2. HAL pin and parameter creation (hal_malloc, hal_pin_newf, hal_param).
//   3. Shared memory allocation (rtapi_shmem_new / getptr / getstate).
//   4. Real-time thread creation (hal_create_thread).
//   5. Function export and mounting (hal_export_funct, hal_add_funct_to_thread).
//   6. Thread start/stop (hal_start_threads / hal_stop_threads).
//   7. Graceful shutdown (hal_stop_threads, rtapi_shmem_delete, hal_exit).
//
// All functions use Google-style comments and follow the project's existing
// error-handling patterns.
//
#include "task.h"
#include "hal/hal.h"
#include "rtapi/rtapi.h"

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
int mot_comp_id    = 0;             // HAL component ID.
int emc_shmem_id   = 0;             // RTAPI shared memory handle.
task_shared_t *taskShared = NULL;   // Application-level shared memory.
task_hal_data_t *taskHalData = NULL;// HAL-allocated pin/param block.
int num_axis       = MAX_AXES;      // Number of configured axes.

// ---------------------------------------------------------------------------
// Module-local constants
// ---------------------------------------------------------------------------
static long task_period_nsec  = DEFAULT_TASK_PERIOD_NSEC;
static long subtask_period_nsec = 0;
int base_thread_fp = 0;

// ---------------------------------------------------------------------------
// Module interface stubs — called during initialisation to register
// callbacks and data pointers with downstream subsystems.
// ---------------------------------------------------------------------------
static int module_intfc(void) {
    // In a full system this would register callbacks with the trajectory
    // planner.  For the example we leave it empty but document the hook.
    return 0;
}


// ============================================================================
// task_thread_main — full task initialisation.
//
// Call sequence:
//   hal_init              → comp_id
//   init_hal_pins          → allocate HAL pin/param block
//   init_task_comm_buffers → allocate application shared memory
//   module_intfc           → register downstream callbacks
//   init_task_threads      → create thread + export/mount functions
//   hal_ready              → mark component ready
//
// Returns 0 on success, -1 on any failure.  Each failure point cleans up
// previously acquired resources before returning.
// ============================================================================
int task_thread_main(void)
{
    int retval;

    // Phase 1 — register with the HAL subsystem.
    mot_comp_id = hal_init("task1");
    if (mot_comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: hal_init failed\n");
        return -1;
    }

    // Print initial HAL status for diagnostics.
    task_print_hal_status();

    // Phase 2 — validate configuration.
    if ((num_axis < 1) || (num_axis > MAX_AXES)) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: num_axis is %d, must be between 1 and %d\n",
            num_axis, MAX_AXES);
        hal_exit(mot_comp_id);
        return -1;
    }

    // Phase 3 — create HAL pins and parameters.
    retval = init_hal_param();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: init_hal_param failed\n");
        hal_exit(mot_comp_id);
        return -1;
    }

    // Phase 4 — allocate application-level shared memory.
    retval = init_task_comm_buffers();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: init_task_comm_buffers failed\n");
        hal_exit(mot_comp_id);
        return -1;
    }

    // Print shared memory lifecycle state for diagnostics.
    task_print_shmem_state();

    // Phase 5 — register downstream interface callbacks.
    module_intfc();

    // Phase 6 — create the real-time thread and mount functions.
    retval = init_task_threads();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: init_task_threads failed\n");
        hal_exit(mot_comp_id);
        return -1;
    }

    // Phase 7 — mark the component ready.  After this point the component
    // may be started via hal_start_threads().
    hal_ready(mot_comp_id);

    rtapi_print_msg(RTAPI_MSG_INFO,
        "task: rtapi_app_main complete (comp_id=%d)\n", mot_comp_id);

    return 0;
}


// ============================================================================
// task_thread_exit — graceful shutdown.
//
// Stops running threads, releases the application shared memory segment,
// and deregisters from the HAL subsystem.
// ============================================================================
void task_thread_exit(void)
{
    int retval;

    // Phase 1 — stop all threads.
    retval = hal_stop_threads();
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: hal_stop_threads() failed, returned %d\n", retval);
    }

    // Phase 2 — release application shared memory.
    // rtapi_shmem_delete is idempotent: safe to call even if already
    // detached or not mapped.
    task_print_shmem_state();  // Diagnostics before deletion.
    retval = rtapi_shmem_delete(emc_shmem_id, mot_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: rtapi_shmem_delete() failed, returned %d\n", retval);
    }

    // Phase 3 — deregister from HAL.  This frees the component's pins,
    // params, and functions, and decrements the HAL reference count.
    retval = hal_exit(mot_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: hal_exit() failed, returned %d\n", retval);
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "task: shutdown complete\n");
}


// ============================================================================
// init_task_comm_buffers — allocate and initialise the application-level
// shared memory segment for the command channel.
//
// Lifecycle: rtapi_shmem_new → getptr → initialise fields → getstate check.
// ============================================================================
static int init_task_comm_buffers(void)
{
    int retval;

    // Allocate the System V shared memory segment.
    emc_shmem_id = rtapi_shmem_new(TASK_SHM_KEY, mot_comp_id,
                                   sizeof(task_shared_t));
    if (emc_shmem_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: rtapi_shmem_new failed, returned %d\n", emc_shmem_id);
        return -1;
    }

    // Map the segment into process address space.
    retval = rtapi_shmem_getptr(emc_shmem_id, (void **)&taskShared);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: rtapi_shmem_getptr failed, returned %d\n", retval);
        return -1;
    }

    // Verify the segment has reached ACTIVE state.
    {
        int shm_state = rtapi_shmem_getstate(emc_shmem_id);
        rtapi_print_msg(RTAPI_MSG_INFO,
            "task: shmem state after creation = %d (expected %d=ACTIVE)\n",
            shm_state, 3);
    }

    // Initialise the command channel fields.
    // The BSS-initialised fields (by the shmem page-touch) are all zero,
    // which gives us the correct initial state: command_mutex == 0 (unlocked),
    // head == tail == 0 (no pending command), heartbeat == 0.
    taskShared->command_id = TASK_CMD_NONE;
    taskShared->command_data = 0;

    return 0;
}


// ============================================================================
// init_hal_param — allocate HAL-managed shared memory for pins and parameters
// and register them with the HAL subsystem.
//
// HAL pins   (6): enable-in, target-pos, feed-rate, current-pos, running,
//                 error, heartbeat-out
// HAL params  (2): max-velocity, max-accel
// ============================================================================
static int init_hal_param(void)
{
    // Allocate the pin/param block from HAL-managed shared memory.
    taskHalData = (task_hal_data_t *)hal_malloc(sizeof(task_hal_data_t));
    if (taskHalData == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: hal_malloc for taskHalData failed\n");
        return -1;
    }

    // Set default parameter values before registering.
    taskHalData->max_velocity = 1000;
    taskHalData->max_accel    = 500;

#define PIN_NEW(type, field, name)                                    \
    do {                                                              \
        int _r = hal_pin_newf(type, (hal_s32_t **)&(taskHalData->field), \
                              mot_comp_id, "%s", name);                \
        if (_r != 0) {                                                \
            rtapi_print_msg(RTAPI_MSG_ERR,                            \
                "task: failed to create pin '%s'\n", name);           \
            return -1;                                                \
        }                                                             \
    } while (0)

    // Create input pins.
    PIN_NEW(HAL_BIT,   enable_in,    "example.enable-in");
    PIN_NEW(HAL_S32,   target_pos,   "example.target-pos");
    PIN_NEW(HAL_FLOAT, feed_rate,    "example.feed-rate");

    // Create output pins.
    PIN_NEW(HAL_S32,   current_pos,   "example.current-pos");
    PIN_NEW(HAL_BIT,   running,       "example.running");
    PIN_NEW(HAL_BIT,   error,         "example.error");
    PIN_NEW(HAL_S32,   heartbeat_out, "example.heartbeat-out");

#undef PIN_NEW

    // Set initial pin values.
    *(taskHalData->enable_in)     = 0;
    *(taskHalData->target_pos)    = 0;
    *(taskHalData->feed_rate)     = 1.0f;
    *(taskHalData->current_pos)   = 0;
    *(taskHalData->running)       = 0;
    *(taskHalData->error)         = 0;
    *(taskHalData->heartbeat_out) = 0;

    // Create HAL parameters.
    if (hal_param_s32_new("example.max-velocity", HAL_RW, HAL_S32,
                          &(taskHalData->max_velocity), mot_comp_id) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: failed to create param 'example.max-velocity'\n");
        return -1;
    }
    if (hal_param_s32_new("example.max-accel", HAL_RW, HAL_S32,
                          &(taskHalData->max_accel), mot_comp_id) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: failed to create param 'example.max-accel'\n");
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
        "task: created %d HAL pins and %d parameters\n", 7, 2);

    return 0;
}


// ============================================================================
// task_config_change — notify the system that configuration has changed.
//
// Called whenever a user-space component modifies HAL parameters or the
// shared memory configuration block.  In a full system this would trigger
// re-initialisation of dependent structures.
// ============================================================================
void task_config_change(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO,
        "task: configuration changed (max_vel=%d, max_accel=%d)\n",
        (int)taskHalData->max_velocity, (int)taskHalData->max_accel);
}


// ============================================================================
// taskSetCycleTime — set the top-level cycle time and propagate to
// sub-cycle times.
// ============================================================================
void taskSetCycleTime(unsigned long nsec)
{
    int mult;

    if (nsec == 0) return;

    mult = (int)(subtask_period_nsec ? task_period_nsec / nsec : 1);
    if (mult < 1) mult = 1;

    settaskCycleTime((double)nsec * 1e-9);
    setsubtaskCycleTime((double)(nsec * mult) * 1e-9);

    rtapi_print_msg(RTAPI_MSG_INFO,
        "task: cycle time set to %lu ns (sub-cycle %d x)\n", nsec, mult);
}


// ============================================================================
// settaskCycleTime — update the task-level cycle time.
// ============================================================================
static int settaskCycleTime(double secs)
{
    if (secs <= 0.0) return -1;

    task_period_nsec = (long)(secs * 1e9);
    task_config_change();

    return 0;
}


// ============================================================================
// setsubtaskCycleTime — update the sub-task-level cycle time.
// ============================================================================
static int setsubtaskCycleTime(double secs)
{
    if (secs <= 0.0) return -1;

    subtask_period_nsec = (long)(secs * 1e9);

    return 0;
}


// ============================================================================
// export — export a single axis's HAL pins.
//   (Stub preserved from original skeleton; creates per-axis pins when
//    num_axis > 1.  Not currently called; retained for API compatibility.)
// ============================================================================
static int export(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO,
        "task: export() called (num_axis=%d)\n", num_axis);
    return 0;
}


// ============================================================================
// init_task_threads — create the real-time thread, export functions to HAL,
// and mount them on the thread in the desired execution order.
//
// Thread:      "example-thread",  task_period_nsec * 1000, uses_fp=1, prio=98
// Functions:   example-cmd-handler   (priority 1 — runs first)
//              example-controller    (priority 2 — runs second)
// ============================================================================
static int init_task_threads(void)
{
    int retval;

    if (subtask_period_nsec == 0) {
        subtask_period_nsec = task_period_nsec * 1000;
    }

    // Phase 1 — create the thread.
    retval = hal_create_thread("example-thread",
                               task_period_nsec * 1000, 1, 98);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: failed to create %ld nsec example thread\n",
            task_period_nsec * 1000);
        return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,
        "task: created thread 'example-thread' (period=%ld ns, prio=98)\n",
        task_period_nsec * 1000);

    // Phase 2 — export the command handler function to HAL.
    retval = hal_export_funct("example-cmd-handler",
                              taskCommandHandler, NULL /* arg */,
                              1 /* uses_fp */,
                              0 /* reentrant */,
                              mot_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: failed to export command handler function\n");
        return -1;
    }

    // Phase 3 — export the controller function to HAL.
    retval = hal_export_funct("example-controller",
                              taskController, NULL /* arg */,
                              1 /* uses_fp */,
                              0 /* reentrant */,
                              mot_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "task: failed to export controller function\n");
        return -1;
    }

    // Phase 4 — mount both functions on the thread.
    //   Position 1 (command handler) runs first.
    //   Position 2 (controller)      runs second.
    hal_add_funct_to_thread("example-cmd-handler",  "example-thread", 1);
    hal_add_funct_to_thread("example-controller",   "example-thread", 2);

    // Phase 5 — propagate cycle times to downstream planners.
    settaskCycleTime((double)(task_period_nsec) * 1e-9);
    setsubtaskCycleTime((double)(subtask_period_nsec) * 1e-9);

    rtapi_print_msg(RTAPI_MSG_INFO,
        "task: thread initialisation complete\n");

    return 0;
}
