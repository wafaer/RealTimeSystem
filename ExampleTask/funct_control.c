#include "task.h"
#include "rtapi/rtapi.h"

void emcmotController(void *arg, long period)
{

    (void)arg;

	task1();

	task2();

	output_to_hal();
}

static void task1() {
}

static void task2() {
}

static void output_to_hal(void)
{
	*(axis_data->motor_offset) = axis->motor_offset;
	*(axis_data->motor_pos_cmd) = axis->motor_pos_cmd;
	*(axis_data->axis_pos_cmd) = axis->pos_cmd;

	*(axis_data->active) = GET_AXIS_ACTIVE_FLAG(axis);
	*(axis_data->motionenable) = emcmotInternal->enabling;
	*(axis_data->motionrunning) = axis->free_tp.enable;

}
