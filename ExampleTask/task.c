#include "task.h"
#include "hal/hal.h"

static long task_period_nsec = 1000000;

/* base_thread_fp — 基础线程浮点使用标志 */
int base_thread_fp = 0;

/*
 * module_intfc — 注册模块接口函数到轨迹规划器
 */
static int module_intfc() {

    /* 注册函数到函数表 */
	tpMotFunctions(emcmotSetRotaryUnlock,emcmotGetRotaryIsUnlocked,axis_get_vel_limit,axis_get_acc_limit);

    /* 注册相关数据*/
	tpMotData(emcmotStatus,emcmotConfig);
	return 0;
}


int task_thread_main(void)
{
    int retval;

    /* 步骤 2：连接 HAL 和 RTAPI，注册 "motmod" 组件 */
    mot_comp_id = hal_init("task1");
    if (mot_comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "task: hal_init failed\n");
	return -1;
    }
    /* 步骤 3：检查轴数量配置的合法性 */
    if (( num_axis < 1 ) || ( num_axis > EMCMOT_MAX_AXIS )) {
	rtapi_print_msg(RTAPI_MSG_ERR,"task: num_joints is %d, must be between 1 and %d\n", num_axis, EMCMOT_MAX_AXIS);
	hal_exit(mot_comp_id);
	return -1;
    }

	/* 步骤 4：初始化 HAL 引脚和参数（创建所有 HAL 引脚） */
	retval = init_hal_param();
	if (retval != 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: init_hal_io failed\n");
		hal_exit(mot_comp_id);
		return -1;
	}

    /* 步骤 5：初始化通信缓冲区（分配共享内存并初始化数据结构） */
    retval = init_motion_comm_buffers();
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, ("MOTION: init_comm_buffers failed\n"));
	hal_exit(mot_comp_id);
	return -1;
    }

	/* 步骤 6：注册模块接口函数到轨迹规划器 */
	module_intfc();

    /* 步骤 8：创建实时线程并挂载函数 */
    retval = init_motion_threads();
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, ("MOTION: init_threads failed\n"));
	hal_exit(mot_comp_id);
	return -1;
    }

    /* 打印模块加载成功信息 */
    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: rtapi_app_main complete\n");

    /* 通知 HAL 子系统本组件已就绪，可以开始工作 */
    hal_ready(mot_comp_id);

    return 0;
}

/*
 * motion_thread_exit — 运动控制模块退出清理
 */
void motion_thread_exit(void)
{
	int retval;

    /* 步骤 1：停止所有实时线程 */
	retval = hal_stop_threads();
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			("MOTION: hal_stop_threads() failed, returned %d\n"), retval);
	}

	/* free shared memory */
    /* 步骤 2：释放共享内存 */
	retval = rtapi_shmem_delete(emc_shmem_id, mot_comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			("MOTION: rtapi_shmem_delete() failed, returned %d\n"), retval);
	}
	/* disconnect from HAL and RTAPI */
    /* 步骤 3：断开与 HAL 和 RTAPI 的连接 */
	retval = hal_exit(mot_comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			("MOTION: hal_exit() failed, returned %d\n"), retval);
	}
}

/*
 * init_motion_comm_buffers — 分配共享内存并初始化所有数据结构
 */
static int init_motion_comm_buffers(void)
{
    int axis_num, n;
    emcmot_axis_t *axis;
    int retval;

    /* 步骤 1：将所有结构体指针初始化为 NULL */
    emcmotStruct = 0;
    emcmotInternal = 0;
    emcmotStatus = 0;
    emcmotCommand = 0;
    emcmotConfig = 0;

    /* 步骤 2：分配共享内存
     * rtapi_shmem_new() 在 RTAPI 管理的共享内存区域中分配一块内存，
     * 大小为 sizeof(emcmot_struct_t)。返回的 emc_shmem_id 是这块共享内存的句柄。 */
    emc_shmem_id = rtapi_shmem_new(key, mot_comp_id, sizeof(emcmot_struct_t));
    if (emc_shmem_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_new failed, returned %d\n", emc_shmem_id);
	return -1;
    }
    /* 步骤 3：获取共享内存的用户空间指针
     * rtapi_shmem_getptr() 返回共享内存的内存地址，
     * 之后可以通过 emcmotStruct 指针自由访问这块内存 */
    retval = rtapi_shmem_getptr(emc_shmem_id, (void **) &emcmotStruct);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_getptr failed, returned %d\n", retval);
	return -1;
    }

    /* we'll reference emcmotStruct directly */
    /* 步骤 4：将各子结构体指针关联到共享内存中的正确偏移
     * emcmotStruct->command 是共享内存内 command 字段的地址，
     * &emcmotStruct->command 即为 emcmotCommand 的值 */
    emcmotCommand = &emcmotStruct->command;
    emcmotStatus = &emcmotStruct->status;
    emcmotConfig = &emcmotStruct->config;
    emcmotInternal = &emcmotStruct->internal;
    emcmotError = &emcmotStruct->error;

    /* init error struct */
    /* 步骤 5：初始化错误环形缓冲区 */
    emcmotErrorInit(emcmotError);

    /* init status struct */
    /* 步骤 6：初始化状态结构体的通信字段 */
    emcmotStatus->head = 0;
    emcmotStatus->commandEcho = 0;
    emcmotStatus->commandNumEcho = 0;
    emcmotStatus->commandStatus = 0;

    /* init more stuff */
    /* 步骤 7-8：初始化内部状态和通信标志 */
    emcmotInternal->head = 0;
    emcmotConfig->head = 0;
	emcmotInternal->enabling = 0;       /* 初始禁用运动使能 */
    emcmotStatus->motionFlag = 0;

    /* 复位状态标志（无错误，不在 Coord 模式）*/
    SET_MOTION_ERROR_FLAG(0);
    SET_MOTION_COORD_FLAG(0);

    emcmotInternal->split = 0;
    emcmotStatus->heartbeat = 0;

    /* 复位笛卡尔位置（将所有分量设为零）*/
    ZERO_EMC_POSE(emcmotStatus->carte_pos_cmd);
    ZERO_EMC_POSE(emcmotStatus->carte_pos_fb);
    emcmotStatus->Maxvel = 0;
    emcmotConfig->limitVel = 0;
    emcmotStatus->Maxacc = 0;
    /* 进给倍率、快移倍率和净倍率均初始化为 1.0（100%）*/
    emcmotStatus->feed_scale = 1.0;
    emcmotStatus->rapid_scale = 1.0;
    emcmotStatus->net_feed_scale = 1.0;

    /* 圆弧混合参数初始化 */
	emcmotConfig->arcBlendOptDepth = 50;   /* 圆弧混合优化深度 */
	emcmotConfig->arcBlendRampFreq = 100;  /* 圆弧混合斜坡频率 */

    /* 使能标志初始化：进给倍率、主轴倍率、进给保持均使能 */
    emcmotStatus->enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;
    emcmotStatus->enables_queued = emcmotStatus->enables_new;
    emcmotStatus->id = 0;
    emcmotStatus->depth = 0;
    emcmotStatus->activeDepth = 0;
    emcmotStatus->paused = 0;
    emcmotStatus->overrideLimitMask = 0;
    SET_MOTION_INPOS_FLAG(1);   /* 初始标记所有轴到位 */
    SET_MOTION_ENABLE_FLAG(0); /* 初始禁用运动 */
    emcmot_config_change();     /* 触发配置变更通知 */

    /* 初始化所有轴的内部数据（axis.c 模块提供）*/
    axis_init_all();

    /* init per-joint stuff */
    /* 步骤 15：遍历所有轴，初始化每个轴的参数 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
    {
		/* point to structure for this joint */
		axis = &axes[axis_num];

		/* init the config fields with some "reasonable" defaults" */
        /* 设置轴类型：0=线性轴（1=旋转轴）*/
		axis->type = 0;
        /* 设置默认软限位（保守的初始值，等待用户配置覆盖）*/
		axis->max_pos_limit = 1;
		axis->min_pos_limit = -1;
        /* 设置默认速度和加速度限制（保守的初始值）*/
		axis->vel_limit = 1;
		axis->acc_limit = 1;
        /* 设置跟随误差限值 */
		axis->min_ferror = 10000;       /* 最小跟随误差（零速时）*/
		axis->max_ferror = 27486951;  /* 最大跟随误差（高速时）*/

		/* init joint flags */
        /* 设置轴标志：活跃且到位 */
		axis->flag = 1;
		SET_AXIS_INPOS_FLAG(axis, 1);

		/* init status info */
        /* 复位所有位置、速度、加速度命令和反馈为 0 */
		axis->coarse_pos = 0;
		axis->pos_cmd = 0;
		axis->vel_cmd = 0;
		axis->acc_cmd = 0;
		axis->motor_pos_cmd = 0;
		axis->motor_pos_fb = 0;
		axis->pos_fb = 0;
		axis->ferror = 0;
        /* 跟随误差限值初始化为 min_ferror */
		axis->ferror_limit = axis->min_ferror;
		axis->ferror_high_mark = 0;

        /* 初始化三次样条插值器 */
		cubicInit(&(axis->cubic));
    }

    /* 步骤 16：同步读指针 */
    emcmotStatus->tail = 0;
	emcmotConfig->tail = emcmotConfig->head;

    return 0;
}

/*
 * updata_axis_param — 从原子变量更新轴的实际位置
 */
void updata_axis_param(int numAxes)
{
	for (int i = 0; i < numAxes; i++)
	{
		emcmot_axis_t *axis = &axes[i];
        /* 【潜在 BUG】所有轴都从同一个 atomic_actpos 读取，
         * 应该是 atomic_actpos[i] 才对。 */
		axis->free_tp.curr_pos = atomic_load(&atomic_actpos);
	}
}

/*
 * emcmot_config_change — 通知系统配置发生了变化
 */
void emcmot_config_change(void)
{
    /* head == tail 表示缓冲区为空（所有数据已被读取），可以安全地更新版本号 */
	if (emcmotConfig->head == emcmotConfig->tail) {
		emcmotConfig->config_num++;           /* 递增配置版本号 */
		emcmotStatus->config_num = emcmotConfig->config_num;  /* 同步到状态结构体 */
		emcmotConfig->head++;                /* 移动写指针 */
	}
}

/*
 * init_motion_threads — 创建实时线程并挂载函数
 */
static int init_motion_threads(void)
{
    int retval;

    /* 步骤 1：如果轨迹周期未指定，默认为伺服周期的 1000 倍
     * 例如：servo_period = 1ms = 1,000,000ns
     *       traj_period = 1,000,000 * 1000 = 1,000,000,000ns = 1s */
	if (traj_period_nsec == 0) {
		traj_period_nsec = servo_period_nsec*1000;
	}

    /* 步骤 2：创建名为 "ExampleTask-thread" 的实时线程
     * 线程周期 = servo_period_nsec * 1000（纳秒）
     * 优先级 = 98（较低）
     * uses_fp = 1（使用浮点运算）*/
    retval = hal_create_thread("ExampleTask-thread", servo_period_nsec*1000, 1,98);
    if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,"MOTION: failed to create %ld nsec ExampleTask thread\n", servo_period_nsec);
		return -1;
    }

    /* 步骤 3：导出命令处理函数到 HAL
     * hal_export_funct() 参数：
     *   - name      : 函数在 HAL 中的名称
     *   - funct     : 实际函数指针
     *   - arg       : 传给函数的参数（此处为 0）
     *   - uses_fp   : 是否使用浮点运算（1=是）
     *   - reentrant : 是否可重入（0=否）
     *   - comp_id   : HAL 组件 ID */
	retval = hal_export_funct("ExampleTask-command-handler", emcmotCommandHandler, 0	/* arg
	 */ , 1 /* uses_fp */ , 0 /* reentrant */ , mot_comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"MOTION: failed to export command handler function\n");
		return -1;
	}

    /* export realtime functions that do the real work */
    /* 步骤 4：导出运动控制器函数到 HAL */
    retval = hal_export_funct("ExampleTask-controller", emcmotController, 0	/* arg
	 */ , 1 /* uses_fp */ , 0 /* reentrant */ , mot_comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to export controller function\n");
	return -1;
    }

    /* 步骤 5：将两个函数挂载到 ExampleTask-thread 上
     * hal_add_funct_to_thread() 参数：
     *   - funct_name : 函数名称
     *   - thread_name: 线程名称
     *   - priority  : 执行优先级（数字越小越先执行）*/
	hal_add_funct_to_thread("ExampleTask-command-handler", "ExampleTask-thread", 1);
	hal_add_funct_to_thread("ExampleTask-controller", "ExampleTask-thread", 2);

    // if we don't set cycle times based on these guesses, emc doesn't
    // start up right
    /* 步骤 6：将周期参数通知给规划器
     * 1e-9 用于将纳秒转换为秒 */
    setServoCycleTime(servo_period_nsec * 1e-9);
    setTrajCycleTime(traj_period_nsec * 1e-9);

    return 0;
}

/*
 * emcmotSetCycleTime — 设置伺服周期并更新相关参数
 */
void emcmotSetCycleTime(unsigned long nsec )
{
    int servo_mult;
    servo_mult = traj_period_nsec / nsec;  /* 计算轨迹周期与伺服周期的比值 */
    if(servo_mult < 0) servo_mult = 1;    /* 防御性检查 */
    setTrajCycleTime(nsec * 1e-9);       /* 更新轨迹周期（单位：秒）*/
    setServoCycleTime(nsec * servo_mult * 1e-9);  /* 更新伺服周期 */
}


/*
 * setTrajCycleTime — 设置轨迹周期并更新相关规划器
 */
static int setTrajCycleTime(double secs)
{
    static int t;  /* 循环变量（static 避免栈分配）*/

    /* make sure it's not zero */
    if (secs <= 0.0) {
	return -1;
    }

    /* 步骤 2：通知配置变更 */
    emcmot_config_change();

    /* 计算插值率：每个轨迹周期内的伺服周期数（四舍五入）*/
    if(emcmotConfig->servoCycleTime)
        emcmotConfig->interpolationRate = (int) (secs / emcmotConfig->servoCycleTime + 0.5);
    else
        emcmotConfig->interpolationRate = 1;  /* 避免除零 */

    /* 步骤 4：设置坐标轨迹规划器的周期 */
    tpSetCycleTime(&emcmotInternal->coord_tp, secs);

    /* 步骤 5：遍历所有轴，设置三次样条插值器的插值率 */
    for (t = 0; t < EMCMOT_MAX_AXIS; t++) {
	cubicSetInterpolationRate(&(axes[t].cubic), emcmotConfig->interpolationRate);
    }

    /* 步骤 6：保存周期值并同步读指针 */
    emcmotConfig->trajCycleTime = secs;
	emcmotConfig->tail = emcmotConfig->head;

    return 0;
}


/*
 * setServoCycleTime — 设置伺服周期并更新相关规划器
 */

/* call this when setting the servo cycle time */
static int setServoCycleTime(double secs)
{
    static int t;  /* 循环变量 */

    /* make sure it's not zero */
    if (secs <= 0.0) {
	return -1;
    }

    /* 步骤 2：通知配置变更 */
    emcmot_config_change();

    /* 计算插值率（注意：这里是 traj/servo，与 setTrajCycleTime 中相同）*/
    emcmotConfig->interpolationRate =
	(int) (emcmotConfig->trajCycleTime / secs + 0.5);

    /* 步骤 4：遍历所有轴，设置三次样条插值器的插值率和段时长 */
    for (t = 0; t < EMCMOT_MAX_AXIS; t++)
    {
		cubicSetInterpolationRate(&(axes[t].cubic), emcmotConfig->interpolationRate);
    	cubicSetSegmentTime(&(axes[t].cubic), emcmotConfig->trajCycleTime);  /* 设置段时长 */
    }

    /* 步骤 5：保存周期值并同步读指针 */
    emcmotConfig->servoCycleTime = secs;
	emcmotConfig->tail = emcmotConfig->head;

    return 0;
}

/*
 * export_axis — 导出单个轴的所有 HAL 引脚
 */
static int export_axis(int num, axis_hal_t * addr)
{
	int retval;  /* HAL API 的返回值 */

	if((retval = hal_pin_newf(HAL_S32, &(addr->coarse_pos_cmd), mot_comp_id, "axis.%d.coarse-pos-cmd",num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_vel_cmd), mot_comp_id, "axis.%d.vel-cmd",num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_acc_cmd), mot_comp_id, "axis.%d.acc-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_dec_cmd), mot_comp_id, "axis.%d.dec-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->motor_pos_cmd), mot_comp_id, "axis.%d.motor-pos-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->motor_pos_fb), mot_comp_id, "axis.%d.motor-pos-fb", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->motor_offset), mot_comp_id, "axis.%d.motor-offset", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_pos_cmd), mot_comp_id, "axis.%d.pos-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_pos_fb), mot_comp_id, "axis.%d.pos-fb", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->f_error), mot_comp_id, "axis.%d.f-error", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->f_error_lim), mot_comp_id, "axis.%d.f-error-lim", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_motion), mot_comp_id, "axis.%d.axis_motion", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_status), mot_comp_id, "axis.%d.axis_status", num)) !=0) return retval;

	if((retval = hal_pin_newf(HAL_BIT, &(addr->active), mot_comp_id, "axis.%d.active", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->motionenable), mot_comp_id, "axis.%d.motionenable", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->motionrunning), mot_comp_id, "axis.%d.motionrunning", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->in_position), mot_comp_id, "axis.%d.in-position", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->error), mot_comp_id, "axis.%d.error", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->phl), mot_comp_id, "axis.%d.pos-hard-limit", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->nhl), mot_comp_id, "axis.%d.neg-hard-limit", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->f_errored), mot_comp_id, "axis.%d.f-errored", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->faulted), mot_comp_id, "axis.%d.faulted", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->pos_lim_sw), mot_comp_id, "axis.%d.pos-lim-sw-in", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->neg_lim_sw), mot_comp_id, "axis.%d.neg-lim-sw-in", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->amp_fault), mot_comp_id, "axis.%d.amp-fault-in", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->amp_enable), mot_comp_id, "axis.%d.amp-enable-out", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->unlock), mot_comp_id, "axis.%d.unlock", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->is_unlocked), mot_comp_id, "axis.%d.is-unlocked", num)) !=0) return retval;

	return 0;
}

/*
 * init_hal_param — 导出所有 HAL 引脚并设置默认值
 */
static int init_hal_param(void)
{
	int n, retval;
	axis_hal_t      *axis_data;  /* 指向轴 HAL 数据的临时指针 */

    /* 第一阶段：分配 HAL 共享内存
     * hal_malloc() 在 HAL 共享内存区域中分配空间，
     * 确保所有实时线程都可以访问这些数据 */
	emcmot_hal_data = hal_malloc(sizeof(emcmot_hal_data_t));
	if (emcmot_hal_data == 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: emcmot_hal_data malloc failed\n");
		return -1;
	}

	hal_pin_newf(HAL_BIT, &(emcmot_hal_data->jog_inhibit), mot_comp_id,"ExampleTask.jog-inhibit");          /* 点动禁止（外部输入）*/
	hal_pin_newf(HAL_BIT, &(emcmot_hal_data->jog_stop), mot_comp_id, "ExampleTask.jog-stop");              /* 点动停止请求（外部输入）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->jog_stop_immediate), mot_comp_id, "ExampleTask.jog-stop-immediate"); /* 点动急停请求（外部输入）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->enable), mot_comp_id, "ExampleTask.enable");                 /* 全局运动使能（外部输入）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->motion_enabled), mot_comp_id, "ExampleTask.ExampleTask-enabled"); /* 运动使能状态（输出）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->in_position), mot_comp_id, "ExampleTask.in-position");       /* 全局到位状态（输出）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->coord_mode), mot_comp_id, "ExampleTask.coord-mode");         /* 坐标模式状态（输出）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->coord_error), mot_comp_id, "ExampleTask.coord-error");       /* 坐标模式错误（输出）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->on_soft_limit), mot_comp_id, "ExampleTask.on-soft-limit");   /* 软限位触发（输出）*/
	hal_pin_newf(HAL_U32,  &(emcmot_hal_data->last_period), mot_comp_id, "ExampleTask.servo.last-period"); /* 上次伺服周期（CPU 时钟周期）*/
	hal_pin_newf(HAL_U32,  &(emcmot_hal_data->jog_is_active), mot_comp_id, "ExampleTask.jog-is-active"); /* 点动活跃状态（输出）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->tp_reverse), mot_comp_id, "ExampleTask.tp-reverse");           /* 轨迹反向运行 */
	hal_pin_newf(HAL_FLOAT, &(emcmot_hal_data->last_period_ns), mot_comp_id, "ExampleTask.servo.last-period-ns"); /* 上次伺服周期（纳秒）*/
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->eoffset_limited), mot_comp_id, "ExampleTask.eoffset-limited"); /* 外部偏移越限 */
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->eoffset_active), mot_comp_id, "ExampleTask.eoffset-active");   /* 外部偏移活跃 */

	*(emcmot_hal_data->jog_inhibit) = 0;         /* 初始不禁用点动 */
	*(emcmot_hal_data->jog_stop) = 0;           /* 初始不请求停止 */
	*(emcmot_hal_data->jog_stop_immediate) = 0; /* 初始不请求急停 */
	*(emcmot_hal_data->enable) = 1;            /* 初始使能运动（等待激活）*/
	*(emcmot_hal_data->motion_enabled) = 0;      /* 初始运动未激活 */
	*(emcmot_hal_data->in_position) = 0;       /* 初始不到位 */
	*(emcmot_hal_data->coord_mode) = 0;         /* 初始非坐标模式 */
	*(emcmot_hal_data->coord_error) = 0;        /* 初始无错误 */
	*(emcmot_hal_data->on_soft_limit) = 0;      /* 初始无软限位 */
	*(emcmot_hal_data->last_period) = 0;      /* 初始周期为 0 */

    /* 第四阶段：导出每个轴的 HAL 引脚 */
	for (n = 0; n < num_axis; n++)
	{
        /* 获取第 n 个轴的 HAL 数据指针 */
		axis_data = &(emcmot_hal_data->axis[n]);
		/* 调用 export_axis() 创建该轴的所有引脚 */
		retval = export_axis(n, axis_data);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: axis %d pin/param export failed\n", n);
			return -1;
		}
        /* 初始化轴的伺服放大器使能引脚为 1（使能状态）*/
		*(axis_data->amp_enable) = 1;
	}

    /* 第五阶段：调用 axis.c 的 HAL 初始化函数
     * axis_init_hal_param() 会创建遥操作和外部偏移相关的 HAL 引脚，
     * 这些引脚由 axis.c 模块管理 */
	axis_init_hal_param(mot_comp_id);

	return 0;
}
