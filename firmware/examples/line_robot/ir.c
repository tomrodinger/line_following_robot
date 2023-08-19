#include "bflb_platform.h"
#include "bl702_glb.h"
#include <FreeRTOS.h>
#include "task.h"
#include "ir.h"
#include "hal_pwm.h"
#include "hal_gpio.h"

#define IR_PWM_CHAN          PWM_CH2_INDEX

struct device *ir_tx;

void ir_init(void)
{
    pwm_dutycycle_config_t pwm_cfg = {200, 400};

    pwm_register(IR_PWM_CHAN, "ir_tx");
    ir_tx = device_find("ir_tx");

    PWM_DEV(ir_tx)->period = 400;
    PWM_DEV(ir_tx)->threshold_low = 200;
    PWM_DEV(ir_tx)->threshold_high = 400;
    device_open(ir_tx, DEVICE_OFLAG_STREAM_TX);
    device_control(ir_tx, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
    pwm_channel_start(ir_tx);
}