//
// operate.cpp — C++ RAII lifecycle wrapper for the ExampleTask.
//
// The ExampleTaskGuard class manages the full lifetime of:
//   - HAL component registration / deregistration.
//   - Application shared memory (System V) allocation / release.
//   - HAL pin and parameter creation (delegated to task.c).
//   - Real-time thread start / stop.
//
// Usage pattern:
//
//   ExampleTaskGuard task("example", TASK_SHM_KEY, sizeof(task_shared_t));
//   if (task.init() != 0) return 1;
//   if (task.start() != 0) return 1;
//
//   // ... runtime interaction ...
//   task.send_command(TASK_CMD_SET_TARGET, 1000);
//   task.send_command(TASK_CMD_PAUSE, 0);
//   task.print_stats();
//
//   task.stop();
//   // ~ExampleTaskGuard() automatically does teardown if not already done.
//
// The destructor is idempotent — it may be called multiple times safely.
//

#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "task.h"
#include "hal/hal.h"
#include "rtapi/rtapi.h"
}

// ============================================================================
// ExampleTaskGuard — RAII wrapper for the complete ExampleTask lifecycle.
// ============================================================================
class ExampleTaskGuard {
public:
    // Construct with a HAL component name, shared memory key and size.
    // The constructor does NOT allocate resources — call init() for that.
    ExampleTaskGuard(const char *name, key_t shm_key, size_t shm_size);

    // Destructor — calls shutdown() if the owner flag is still set.
    ~ExampleTaskGuard();

    // Full initialisation:  hal_init → create pins/params → shared memory
    // → create thread → export/mount functions → hal_ready.
    // Returns 0 on success, -1 on failure.
    int init();

    // Start the real-time threads.  Returns 0 on success.
    int start();

    // Stop the real-time threads.  Returns 0 on success.
    int stop();

    // Send a command to the real-time thread through the shared memory
    // command channel.  The user-space side acquires the mutex with
    // rtapi_mutex_get() (blocking) since this is not time-critical.
    // Returns 0 on success, -1 if the shared memory is not mapped.
    int send_command(task_command_id_t cmd, int data);

    // Shut down the task:  stop threads, release shared memory, deregister
    // from HAL.  Idempotent.
    void shutdown();

private:
    char    name_[48];       // HAL component name.
    key_t   shm_key_;        // System V IPC key.
    size_t  shm_size_;       // Shared memory segment size.
    bool    owns_;           // True if resources are still held.
    bool    started_;        // True if threads have been started.
};


ExampleTaskGuard::ExampleTaskGuard(const char *name,
                                   key_t shm_key,
                                   size_t shm_size)
    : shm_key_(shm_key),
      shm_size_(shm_size),
      owns_(false),
      started_(false)
{
    std::snprintf(name_, sizeof(name_), "%s", name);
}


ExampleTaskGuard::~ExampleTaskGuard()
{
    shutdown();
}


int ExampleTaskGuard::init()
{
    // Already initialised.
    if (owns_) return 0;

    if (task_thread_main() != 0) {
        std::fprintf(stderr, "ExampleTaskGuard: init failed\n");
        return -1;
    }

    owns_ = true;
    return 0;
}


int ExampleTaskGuard::start()
{
    if (!owns_) return -1;
    if (started_) return 0;

    int ret = hal_start_threads();
    if (ret < 0) {
        std::fprintf(stderr, "ExampleTaskGuard: start failed (ret=%d)\n", ret);
        return -1;
    }
    started_ = true;


    return 0;
}


int ExampleTaskGuard::stop()
{
    if (!started_) return 0;

    int ret = hal_stop_threads();
    if (ret < 0) {
        std::fprintf(stderr, "ExampleTaskGuard: stop failed (ret=%d)\n", ret);
        return -1;
    }
    started_ = false;

    return 0;
}


int ExampleTaskGuard::send_command(task_command_id_t cmd, int data)
{
    if (!taskShared) return -1;

    // Acquire the mutex — the user-space side can block here because
    // command submission is not time-critical.
    rtapi_mutex_get(&taskShared->command_mutex);

    // Place the command in the shared memory channel.
    taskShared->command_id   = cmd;
    taskShared->command_data = data;
    taskShared->head++;     // Advance the producer pointer.

    rtapi_mutex_give(&taskShared->command_mutex);

    return 0;
}


void ExampleTaskGuard::shutdown()
{
    if (!owns_) return;

    // Stop threads if still running.
    if (started_) {
        stop();
    }

    // Tear down everything.
    task_thread_exit();

    owns_    = false;
    started_ = false;
}


// ============================================================================
// Standalone demo main() — exercises the full lifecycle.
//
// Compile as a standalone binary to verify the ExampleTask is self-contained
// and correctly exercises all RTAPI / HAL / shared-memory features.
// ============================================================================
#ifdef EXAMPLE_TASK_STANDALONE
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    // --- Initialisation ---
    ExampleTaskGuard task("example-task",
                          TASK_SHM_KEY,
                          sizeof(task_shared_t));

    if (task.init() != 0) {
        std::fprintf(stderr, "FATAL: initialisation failed\n");
        return 1;
    }

    if (task.start() != 0) {
        std::fprintf(stderr, "FATAL: start failed\n");
        return 1;
    }

    // --- Runtime: send a sequence of commands ---

    // 1. Enable the controller.
    task.send_command(TASK_CMD_ENABLE, 0);

    // 2. Set a target position.
    task.send_command(TASK_CMD_SET_TARGET, 1000);

    // 3. Wait a few cycles for the controller to track.
    rtapi_delay(5000);

    // 4. Adjust the PLL correction.
    task.send_command(TASK_CMD_SET_PLL, 50);

    // 5. Change the target.
    task.send_command(TASK_CMD_SET_TARGET, 500);

    // 6. Pause the thread.
    task.send_command(TASK_CMD_PAUSE, 0);
    rtapi_delay(2000);

    // 7. Resume the thread.
    task.send_command(TASK_CMD_RESUME, 0);

    // 8. Set a new target.
    task.send_command(TASK_CMD_SET_TARGET, 0);

    // 9. Disable the controller.
    task.send_command(TASK_CMD_DISABLE, 0);

    // 10. Print final statistics.
    task.print_stats();

    // --- Shutdown ---
    task.shutdown();

    std::printf("ExampleTask demo complete.\n");
    return 0;
}
#endif  // EXAMPLE_TASK_STANDALONE
