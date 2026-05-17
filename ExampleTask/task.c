#include "task.h"
#include "hal/hal.h"
#include "hal/hal_priv.h"
#include "rtapi/rtapi.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/shm.h>

int mot_comp_id  = 0;
int emc_shmem_id = 0;
task_shared_t *taskShared  = NULL;
task_hal_data_t *taskHalData = NULL;

static long task_period_nsec = DEFAULT_TASK_PERIOD_NSEC;

static int init_task_comm_buffers(void);
static int init_hal_pins_and_params(void);
static int init_task_threads(void);
static int reclaim_orphan_task_shm(void);

int task_thread_main(void)
{
    int retval;

    mot_comp_id = hal_init("servo_task");
    if (mot_comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "servo_task: hal_init failed\n");
        return -1;
    }

    retval = init_hal_pins_and_params();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "servo_task: init_hal_pins failed\n");
        hal_exit(mot_comp_id);
        return -1;
    }

    retval = init_task_comm_buffers();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "servo_task: init_task_comm_buffers failed\n");
        hal_exit(mot_comp_id);
        return -1;
    }

    retval = init_task_threads();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "servo_task: init_task_threads failed\n");
        hal_exit(mot_comp_id);
        return -1;
    }

    hal_ready(mot_comp_id);

    rtapi_print_msg(RTAPI_MSG_INFO,
        "servo_task: started (period=%ld ns, comp_id=%d)\n",
        task_period_nsec, mot_comp_id);
    return 0;
}

void task_thread_exit(void)
{
    int retval;
    int hal_state = hal_get_state();

    if (hal_state == HAL_S_RUNNING) {
        retval = hal_stop_threads();
        if (retval < 0 && retval != -EALREADY) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "servo_task: hal_stop_threads returned %d\n", retval);
        }
    }

    if (hal_state != HAL_S_DESTROYED) {
        retval = rtapi_shmem_delete(emc_shmem_id, mot_comp_id);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "servo_task: rtapi_shmem_delete returned %d\n", retval);
        }
    }

    retval = hal_exit(mot_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: hal_exit returned %d\n", retval);
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "servo_task: shutdown complete\n");
}

static int reclaim_orphan_task_shm(void)
{
    int id = shmget((key_t)TASK_SHM_KEY, 0, 0);
    if (id < 0) {
        if (errno == ENOENT) return 0;
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: shmget probe failed: %s\n", strerror(errno));
        return -1;
    }

    struct shmid_ds info;
    if (shmctl(id, IPC_STAT, &info) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: shmctl(IPC_STAT) failed: %s\n", strerror(errno));
        return -1;
    }

    void *addr = shmat(id, NULL, 0);
    if (addr == (void *)-1) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: shmat probe failed: %s — removing segment\n",
            strerror(errno));
        shmctl(id, IPC_RMID, NULL);
        return 0;
    }

    task_shared_t *old = (task_shared_t *)addr;
    int orphan = 0;
    pid_t live_pid = 0;

    if (info.shm_segsz < sizeof(task_shared_t)
        || old->shm_magic != TASK_SHM_MAGIC
        || old->shm_version != TASK_SHM_VERSION) {
        orphan = 1;
    } else if (old->creator_pid <= 0) {
        orphan = 1;
    } else if (kill(old->creator_pid, 0) == -1 && errno == ESRCH) {
        orphan = 1;
    } else {
        live_pid = old->creator_pid;
    }

    shmdt(addr);

    if (live_pid != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: another instance is running (pid=%d)\n",
            (int)live_pid);
        return -1;
    }

    if (orphan) {
        if (shmctl(id, IPC_RMID, NULL) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "servo_task: failed to reclaim orphan shm: %s\n",
                strerror(errno));
            return -1;
        }
        rtapi_print_msg(RTAPI_MSG_INFO,
            "servo_task: reclaimed orphan task shm (id=%d)\n", id);
    }
    return 0;
}

static int init_task_comm_buffers(void)
{
    int retval;
    int shm_state;

    if (reclaim_orphan_task_shm() != 0) return -1;

    emc_shmem_id = rtapi_shmem_new(TASK_SHM_KEY, mot_comp_id, sizeof(task_shared_t));
    if (emc_shmem_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: rtapi_shmem_new failed (retval=%d)\n", emc_shmem_id);
        return -1;
    }

    retval = rtapi_shmem_getptr(emc_shmem_id, (void **)&taskShared);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: rtapi_shmem_getptr failed (retval=%d)\n", retval);
        return -1;
    }

    shm_state = rtapi_shmem_getstate(emc_shmem_id);
    if (shm_state != 3) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: shmem not ACTIVE (state=%d)\n", shm_state);
        return -1;
    }

    // Zero all shared memory to ensure deterministic initial state.
    memset(taskShared, 0, sizeof(task_shared_t));

    // Header — written first so a future instance can identify this segment
    // as ours and probe whether the creator process is still alive.
    taskShared->shm_magic   = TASK_SHM_MAGIC;
    taskShared->shm_version = TASK_SHM_VERSION;
    taskShared->creator_pid = getpid();

    // Trajectory planner initial state — auto-start motion at boot:
    // pos_cmd=1e6, vel=1000, acc=10000, period=1ms (≈ 1000 s total).
    taskShared->tp.enable   = 1;
    taskShared->tp.active   = 0;
    taskShared->tp.pos_cmd  = 10000.0f;
    taskShared->tp.curr_pos = 0.0f;
    taskShared->tp.curr_vel = 0.0f;
    taskShared->tp.vel      = 1000.0f;
    taskShared->tp.acc      = 10000.0f;

    // PID initial state.
    pid_init(&taskShared->pid);
    taskShared->pid.kp = 10.0f;
    taskShared->pid.ki = 0.0f;
    taskShared->pid.kd = 0.0f;

    // Simulation state.
    taskShared->actual_pos = 0.0f;
    taskShared->pid_output = 0.0f;
    taskShared->heartbeat  = 0;
    taskShared->cycle_count = 0;

    rtapi_print_msg(RTAPI_MSG_INFO,
        "servo_task: shared memory initialised "
        "(tp.vel=%.1f, tp.acc=%.1f, pid.kp=%.1f)\n",
        taskShared->tp.vel, taskShared->tp.acc, taskShared->pid.kp);

    return 0;
}

static int init_hal_pins_and_params(void)
{
    taskHalData = (task_hal_data_t *)hal_malloc(sizeof(task_hal_data_t));
    if (!taskHalData) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: hal_malloc for taskHalData failed\n");
        return -1;
    }
    memset(taskHalData, 0, sizeof(task_hal_data_t));

    // ---- Input pins ----
    if (hal_pin_newf(HAL_BIT,   (hal_s32_t **)&taskHalData->enable,
            mot_comp_id, "enable") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->pos_cmd,
            mot_comp_id, "pos-cmd") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->vel_cmd,
            mot_comp_id, "vel-cmd") != 0) return -1;

    // Drive the motion automatically at startup — these are the values the
    // command handler will copy into the trajectory planner on its first cycle.
    *(hal_bit_t   *)taskHalData->enable  = 1;
    *(hal_float_t *)taskHalData->pos_cmd = 10000.0f;
    *(hal_float_t *)taskHalData->vel_cmd = 1000.0f;

    // ---- Output pins ----
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->planned_pos,
            mot_comp_id, "planned-pos") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->planned_vel,
            mot_comp_id, "planned-vel") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->actual_pos_out,
            mot_comp_id, "actual-pos") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->pid_out,
            mot_comp_id, "pid-out") != 0) return -1;

    // ---- Parameters (allocated in HAL shmem, read/write by external components) ----
    #define ALLOC_PARAM(T, ptr) do { \
        ptr = (T *)hal_malloc(sizeof(T)); \
        if (!(ptr)) return -1; \
    } while (0)

    ALLOC_PARAM(hal_float_t, taskHalData->tp_vel);
    ALLOC_PARAM(hal_float_t, taskHalData->tp_acc);
    ALLOC_PARAM(hal_float_t, taskHalData->pid_kp);
    ALLOC_PARAM(hal_float_t, taskHalData->pid_ki);
    ALLOC_PARAM(hal_float_t, taskHalData->pid_kd);
    ALLOC_PARAM(hal_s32_t,   taskHalData->heartbeat_out);
    ALLOC_PARAM(hal_s32_t,   taskHalData->cycle_count_out);

    // Default parameter values.
    *taskHalData->tp_vel  = 1000.0f;
    *taskHalData->tp_acc  = 10000.0f;
    *taskHalData->pid_kp  = 10.0f;
    *taskHalData->pid_ki  = 0.0f;
    *taskHalData->pid_kd  = 0.0f;
    *taskHalData->heartbeat_out  = 0;
    *taskHalData->cycle_count_out = 0;

    if (hal_param_float_newf(HAL_RW, taskHalData->tp_vel, mot_comp_id, "tp-vel") != 0) return -1;
    if (hal_param_float_newf(HAL_RW, taskHalData->tp_acc, mot_comp_id, "tp-acc") != 0) return -1;
    if (hal_param_float_newf(HAL_RW, taskHalData->pid_kp,  mot_comp_id, "pid-kp")  != 0) return -1;
    if (hal_param_float_newf(HAL_RW, taskHalData->pid_ki,  mot_comp_id, "pid-ki")  != 0) return -1;
    if (hal_param_float_newf(HAL_RW, taskHalData->pid_kd,  mot_comp_id, "pid-kd")  != 0) return -1;

    rtapi_print_msg(RTAPI_MSG_INFO,
        "servo_task: created %d pins and %d parameters\n", 7, 5);
    return 0;
}

static int init_task_threads(void)
{
    int retval;

    // Single servo thread — drives both the command handler and the controller.
    retval = hal_create_thread("servo-thread", task_period_nsec, 1, 98);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: hal_create_thread failed (retval=%d)\n", retval);
        return -1;
    }

    // Controller runs first (pos=1) so trajectory update is visible to
    // command handler when it runs later (pos=2).
    retval = hal_export_funct("servo-controller",
            taskController, NULL, 1, 0, mot_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: failed to export servo-controller\n");
        return -1;
    }

    retval = hal_add_funct_to_thread("servo-controller",
            "servo-thread", 1);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: failed to add servo-controller to thread\n");
        return -1;
    }

    // Command handler runs second (pos=2) so it can read the updated planned
    // position and feed back to the planner if needed.
    retval = hal_export_funct("servo-command-handler",
            taskCommandHandler, NULL, 1, 0, mot_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: failed to export servo-command-handler\n");
        return -1;
    }

    retval = hal_add_funct_to_thread("servo-command-handler",
            "servo-thread", 2);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: failed to add servo-command-handler to thread\n");
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
        "servo_task: thread 'servo-thread' initialised (period=%ld ns)\n",
        task_period_nsec);
    return 0;
}
