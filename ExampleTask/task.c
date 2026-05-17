/// \file task.c
///
/// 实时伺服任务核心模块实现。
///
/// 负责以下初始化流程：HAL 组件初始化 → HAL 引脚/参数注册 → 共享内存分配 →
/// 轨迹/PID/仿真状态初始化 → 创建实时线程。退出时依次停止线程、
/// 删除共享内存并退出 HAL 组件。

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

/// 全局 HAL 组件 ID。
int mot_comp_id  = 0;
/// 任务共享内存 System V IPC ID。
int emc_shmem_id = 0;
/// 指向任务共享内存结构体的指针。
task_shared_t *taskShared  = NULL;
/// 指向任务 HAL 数据结构体的指针。
task_hal_data_t *taskHalData = NULL;

/// 默认调度周期（纳秒）。
static long task_period_nsec = DEFAULT_TASK_PERIOD_NSEC;

static int init_task_comm_buffers(void);
static int init_hal_pins_and_params(void);
static int init_task_threads(void);
static int reclaim_orphan_task_shm(void);

/// \brief 任务线程入口（启动时被调用）。
///
/// 按顺序执行以下初始化步骤：
/// 1. 调用 hal_init() 注册 HAL 组件。
/// 2. 注册 HAL 引脚和参数。
/// 3. 分配并初始化共享内存缓冲区和任务状态。
/// 4. 创建实时线程（servo-thread）。
/// 5. 调用 hal_ready() 通知 HAL 组件已就绪。
///
/// \return 0 成功；-1 失败（HAL 初始化失败或任意子步骤失败）。
int task_thread_main(void) {
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

/// \brief 任务线程退出处理。
///
/// 按以下顺序清理资源：
/// 1. 若 HAL 处于 RUNNING 状态，停止所有线程。
/// 2. 若共享内存尚未销毁，删除共享内存段。
/// 3. 调用 hal_exit() 退出 HAL 组件。
void task_thread_exit(void) {
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

/// \brief 检测并回收孤儿共享内存段。
///
/// 通过 shmget/shmat 探测是否存在由已死亡进程遗留的共享内存段。
/// 若该段的 magic/version/creator_pid 不匹配或创建者进程已不存在，
/// 则视为孤儿并调用 shmctl(IPC_RMID) 回收。
/// 若发现另一个健康实例仍在运行，则返回错误以防止重复启动。
///
/// \return 0 可以安全继续（无冲突或已回收孤儿）；-1 发现冲突实例或操作失败。
static int reclaim_orphan_task_shm(void) {
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

    // 判断是否为孤儿段：大小不匹配、magic/version 错误、creator_pid 无效、
    // 或创建者进程已不存在。
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

/// \brief 初始化任务共享内存缓冲区。
///
/// 1. 调用 reclaim_orphan_task_shm() 检查孤儿段。
/// 2. 创建新的 System V 共享内存段并映射到 taskShared。
/// 3. 将共享内存全部清零以保证确定性初始状态。
/// 4. 写入魔数、版本号、创建者 PID 作为头部标识。
/// 5. 初始化轨迹规划器（tp）、PID 和仿真状态。
///
/// \return 0 成功；-1 失败。
static int init_task_comm_buffers(void) {
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

    // 清零共享内存，保证确定性初始状态。
    memset(taskShared, 0, sizeof(task_shared_t));

    // 写入头部标识（magic/version/creator_pid），供后续实例探测和回收。
    taskShared->shm_magic   = TASK_SHM_MAGIC;
    taskShared->shm_version = TASK_SHM_VERSION;
    taskShared->creator_pid = getpid();

    // 轨迹规划器初始状态：启动时自动向 pos_cmd=10000 运动。
    taskShared->tp.enable   = 1;
    taskShared->tp.active   = 0;
    taskShared->tp.pos_cmd  = 10000.0f;
    taskShared->tp.curr_pos = 0.0f;
    taskShared->tp.curr_vel = 0.0f;
    taskShared->tp.vel      = 1000.0f;
    taskShared->tp.acc      = 10000.0f;

    // PID 初始状态。
    pid_init(&taskShared->pid);
    taskShared->pid.kp = 10.0f;
    taskShared->pid.ki = 0.0f;
    taskShared->pid.kd = 0.0f;

    // 仿真状态初始值。
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

/// \brief 注册 HAL 引脚和参数。
///
/// 为 taskHalData 分配内存，然后依次注册：
/// - 3 个输入引脚（enable / pos-cmd / vel-cmd）。
/// - 4 个输出引脚（planned-pos / planned-vel / actual-pos / pid-out）。
/// - 5 个读写参数（tp-vel / tp-acc / pid-kp / pid-ki / pid-kd）。
/// - 2 个输出参数（heartbeat-out / cycle-count-out）。
/// 启动时将 enable / pos-cmd / vel-cmd 设为默认值，驱动首次运动。
///
/// \return 0 成功；-1 失败。
static int init_hal_pins_and_params(void) {
    taskHalData = (task_hal_data_t *)hal_malloc(sizeof(task_hal_data_t));
    if (!taskHalData) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: hal_malloc for taskHalData failed\n");
        return -1;
    }
    memset(taskHalData, 0, sizeof(task_hal_data_t));

    // ---- 输入引脚 ----
    if (hal_pin_newf(HAL_BIT,   (hal_s32_t **)&taskHalData->enable,
            mot_comp_id, "enable") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->pos_cmd,
            mot_comp_id, "pos-cmd") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->vel_cmd,
            mot_comp_id, "vel-cmd") != 0) return -1;

    // 启动时驱动运动：首次周期时命令处理函数会读取这些值并写入轨迹规划器。
    *(hal_bit_t   *)taskHalData->enable  = 1;
    *(hal_float_t *)taskHalData->pos_cmd = 10000.0f;
    *(hal_float_t *)taskHalData->vel_cmd = 1000.0f;

    // ---- 输出引脚 ----
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->planned_pos,
            mot_comp_id, "planned-pos") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->planned_vel,
            mot_comp_id, "planned-vel") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->actual_pos_out,
            mot_comp_id, "actual-pos") != 0) return -1;
    if (hal_pin_newf(HAL_FLOAT, (hal_s32_t **)&taskHalData->pid_out,
            mot_comp_id, "pid-out") != 0) return -1;

    // ---- 参数（分配于 HAL 共享内存，可由外部组件读写） ----
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

    // 参数默认值。
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

/// \brief 创建实时线程并注册回调函数。
///
/// 创建名为 "servo-thread" 的实时线程（周期由 task_period_nsec 指定，优先级 98）。
/// 在该线程中依次注册两个函数（按执行顺序）：
/// 1. servo-controller（pos=1）：先执行轨迹更新，使命令处理函数可见最新规划位置。
/// 2. servo-command-handler（pos=2）：后执行，读取 HAL 输入并写回共享内存命令。
///
/// \return 0 成功；-1 失败。
static int init_task_threads(void) {
    int retval;

    // 创建单一伺服线程，驱动命令处理和控制两个回调。
    retval = hal_create_thread("servo-thread", task_period_nsec, 1, 98);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "servo_task: hal_create_thread failed (retval=%d)\n", retval);
        return -1;
    }

    // servo-controller 先执行，使更新后的轨迹状态对 servo-command-handler 可见。
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

    // servo-command-handler 后执行，读取 HAL 输入后写回共享内存。
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
