//
// main/stubs.cpp — minimal stub implementations for missing external symbols
// referenced by taskmain.cpp.
//
// These symbols are part of the original LinuxCNC motion-controller
// infrastructure that has not been ported to this standalone example.
// They are provided as stubs so the project compiles, links, and can
// be exercised during development.
//

#include <unistd.h>
#include <time.h>

extern "C" {

// ---------------------------------------------------------------------------
// emcMotionUpdate — service the NML motion subsystem.
//
// In the full system this polls NML buffers, processes incoming motion
// commands, and updates the trajectory planner.  The stub returns 0 to
// indicate the subsystem is alive.
// ---------------------------------------------------------------------------
int emcMotionUpdate(void)
{
    return 0;
}

// ---------------------------------------------------------------------------
// esleep — sleep for the specified number of seconds.
//
// Used during startup to wait for NML subsystems to become responsive.
// ---------------------------------------------------------------------------
int esleep(double seconds)
{
    if (seconds <= 0.0) return 0;

    // Split into whole seconds + sub-second remainder.
    unsigned int sec  = (unsigned int)seconds;
    useconds_t   usec = (useconds_t)((seconds - (double)sec) * 1000000.0);

    if (sec > 0) {
        sleep(sec);
    }
    if (usec > 0) {
        usleep(usec);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// etime — elapsed time in seconds (monotonic clock).
//
// Returns wall-clock seconds since an unspecified epoch, suitable for
// measuring iteration latency in the main control loop.
// ---------------------------------------------------------------------------
double etime(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

} // extern "C"

// ---------------------------------------------------------------------------
// Global variables referenced by taskmain.cpp.
// ---------------------------------------------------------------------------
double DEFAULT_EMC_TASK_CYCLE_TIME = 0.1;  // 100 ms — default cycle time
                                            // for latency warning threshold.

int   emcTaskNoDelay = 0;   // When non-zero, skip the per-cycle delay.
int   emcTaskEager   = 0;   // When non-zero, process without waiting.

void *timer = NULL;         // Placeholder for RCS_TIMER object.
