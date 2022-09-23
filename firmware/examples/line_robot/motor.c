#include "bflb_platform.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
#include "motor.h"

#define MOTOR_LB_PWM_CHAN          PWM_CH4_INDEX
#define MOTOR_LA_PWM_CHAN          PWM_CH3_INDEX
#define MOTOR_RA_PWM_CHAN          PWM_CH0_INDEX
#define MOTOR_RB_PWM_CHAN          PWM_CH1_INDEX

struct device *motor_l[2];
struct device *motor_r[2];

void motor_init(void)
{
    pwm_register(MOTOR_LA_PWM_CHAN, "motor_la");
    pwm_register(MOTOR_LB_PWM_CHAN, "motor_lb");
    pwm_register(MOTOR_RA_PWM_CHAN, "motor_ra");
    pwm_register(MOTOR_RB_PWM_CHAN, "motor_rb");

    motor_l[0] = device_find("motor_la");
    motor_l[1] = device_find("motor_lb");
    motor_r[0] = device_find("motor_ra");
    motor_r[1] = device_find("motor_rb");

    PWM_DEV(motor_l[0])->period = 100;
    PWM_DEV(motor_l[0])->threshold_low = 0;
    PWM_DEV(motor_l[0])->threshold_high = 100;
    device_open(motor_l[0], DEVICE_OFLAG_STREAM_TX);
    pwm_channel_start(motor_l[0]);

    PWM_DEV(motor_l[1])->period = 100;
    PWM_DEV(motor_l[1])->threshold_low = 0;                
    PWM_DEV(motor_l[1])->threshold_high = 100;
    device_open(motor_l[1], DEVICE_OFLAG_STREAM_TX);
    pwm_channel_start(motor_l[1]);

    PWM_DEV(motor_r[0])->period = 100;
    PWM_DEV(motor_r[0])->threshold_low = 0;
    PWM_DEV(motor_r[0])->threshold_high = 100;
    device_open(motor_r[0], DEVICE_OFLAG_STREAM_TX);
    pwm_channel_start(motor_r[0]);

    PWM_DEV(motor_r[1])->period = 100;
    PWM_DEV(motor_r[1])->threshold_low = 0;
    PWM_DEV(motor_r[1])->threshold_high = 100;
    device_open(motor_r[1], DEVICE_OFLAG_STREAM_TX);
    pwm_channel_start(motor_r[1]);
}

void motor_run(motor_direction_t dir, uint8_t speed)
{
    pwm_dutycycle_config_t pwm_cfg = {0, 100};
    uint8_t turn_speed;

    device_control(motor_l[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
    device_control(motor_l[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
    device_control(motor_r[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
    device_control(motor_r[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
    bflb_platform_delay_us(1);

    if (speed > 100) speed = 100;

    if (speed == 100) {
        speed = 0;
        turn_speed = 25;
    } else {
        turn_speed = 100 - (speed / 4);
        speed = 100 - speed;
    }

    switch(dir) {
        case FORWARD:
            pwm_cfg.threshold_high = speed;
            device_control(motor_l[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            device_control(motor_r[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case BACKWARD:
            pwm_cfg.threshold_high = speed;
            device_control(motor_l[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            device_control(motor_r[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case FORWARD_LEFT:
            pwm_cfg.threshold_high = turn_speed;
            device_control(motor_l[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            pwm_cfg.threshold_high = speed;
            device_control(motor_r[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case FORWARD_RIGHT:
            pwm_cfg.threshold_high = turn_speed;
            device_control(motor_r[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            pwm_cfg.threshold_high = speed;
            device_control(motor_l[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case BACKWARD_LEFT:
            pwm_cfg.threshold_high = turn_speed;
            device_control(motor_l[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            pwm_cfg.threshold_high = speed;
            device_control(motor_r[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case BACKWARD_RIGHT:
            pwm_cfg.threshold_high = turn_speed;
            device_control(motor_r[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            pwm_cfg.threshold_high = speed;
            device_control(motor_l[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case CIRCLE_LEFT:
            pwm_cfg.threshold_high = speed;
            device_control(motor_l[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            device_control(motor_r[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case CIRCLE_RIGHT:
            pwm_cfg.threshold_high = speed;
            device_control(motor_r[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            device_control(motor_l[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
            break;
        case STOP:
        default:
            break;
    }
}

void motor_run_manual(int8_t r_speed, int8_t l_speed)
{
    pwm_dutycycle_config_t pwm_cfg = {0, 100};

    if (r_speed > 0) {
        r_speed = 100 - r_speed;
        l_speed = 100 - l_speed;

        pwm_cfg.threshold_high = l_speed;
        device_control(motor_l[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        pwm_cfg.threshold_high = r_speed;
        device_control(motor_r[0], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
    } else {
        r_speed = 100 - (r_speed * -1);
        l_speed = 100 - (l_speed * -1);
        pwm_cfg.threshold_high = l_speed;
        device_control(motor_l[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        pwm_cfg.threshold_high = r_speed;
        device_control(motor_r[1], DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
    }
}