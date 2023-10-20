#include <math.h>
#include "bflb_platform.h"
#include "motor.h"
#include "sensor.h"
#include <FreeRTOS.h>
#include "task.h"
#include "ble_app.h"
#include "pid.h"
#include "hal_gpio.h"

#define TURN_THRESHOLD          -20
#define CALIBRATION_SAMPLES     5
#define CALIBRATION_LEFT_TIMEOUT        80
#define CALIBRATION_RIGHT_TIMEOUT       180
#define CALIBRATION_CENTER_TIMEOUT      100
#define TURN_TIMEOUT                    5000

#define LED_R_0  GPIO_PIN_22
#define LED_R_1  GPIO_PIN_0

#define LED_G_0  GPIO_PIN_24
#define LED_G_1  GPIO_PIN_2

#define LED_B_0  GPIO_PIN_23
#define LED_B_1  GPIO_PIN_1

// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata;
pid_control_t pid;

// Control loop input,output and setpoint variables
float input = 0, output = 0;
float setpoint = 0;

// Control loop gains
float kp = 0.1, ki = 0.0001, kd = 0.005;
uint64_t prev_calib_time;

#define MOTOR_DIV   1

void robot_init(void)
{
    sensor_init();
    motor_init();

    // Prepare PID controller for operation
	pid = pid_create(&ctrldata, &input, &output, &setpoint, kp, ki, kd);
	// Set controler output limits from 0 to 200
	pid_limits(pid, -50, 50);
	// Allow PID to compute and change output
	pid_auto(pid);

    prev_calib_time = bflb_platform_get_time_ms();

    gpio_set_mode(LED_R_0, GPIO_OUTPUT_PP_MODE);
    gpio_write(LED_R_0, 1);
    gpio_set_mode(LED_R_1, GPIO_OUTPUT_PP_MODE);
    gpio_write(LED_R_1, 1);

    gpio_set_mode(LED_G_0, GPIO_OUTPUT_PP_MODE);
    gpio_write(LED_G_0, 1);
    gpio_set_mode(LED_G_1, GPIO_OUTPUT_PP_MODE);
    gpio_write(LED_G_1, 1);

    gpio_set_mode(LED_B_0, GPIO_OUTPUT_PP_MODE);
    gpio_write(LED_B_0, 1);
    gpio_set_mode(LED_B_1, GPIO_OUTPUT_PP_MODE);
    gpio_write(LED_B_1, 1);
}

void robot_run(void)
{
    static sensor_light_t sen_light_calib_val;
    static uint32_t prev_diff_speed = 0;
    static bool is_calib = false;
    sensor_light_t sen_light_val;
    sensor_motor_t sen_motor_val;
    int32_t left_diff, right_diff;
    int left_speed, right_speed;
    static uint8_t robot_detect_cnt = 0;
    bool is_robot_detect = false;
    int calib_timeout = CALIBRATION_LEFT_TIMEOUT;
    int max_speed = 50 / MOTOR_DIV;
    static sensor_motor_t prev_sen_motor_val; 
    static int cur_left_speed, cur_right_speed, prev_left_speed, prev_right_speed = 0;
    bool is_high_spike_detected = false;
    uint32_t timeout_turn;

    if ((bflb_platform_get_time_ms() - prev_calib_time) >= 5000) {
        if (prev_diff_speed <= 40) {
            is_calib = false;
        }
    }

    if (!is_calib) {
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (sensor_ir_is_measuring()) {
            while (!sensor_ir_is_result_ready()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        sensor_ir_start_measure();
        while (!sensor_ir_is_result_ready()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!sensor_ir_store_calib()) {
            robot_detect_cnt++;
            goto robot_detected;
        } else {
            robot_detect_cnt = 0;
        }
        
        calib_timeout = CALIBRATION_LEFT_TIMEOUT;
        motor_run(CIRCLE_LEFT, 25 / MOTOR_DIV);
        while(1) {
            sensor_light_read(&sen_light_val, &sen_motor_val, CALIBRATION_SAMPLES, 1);
            if ((sen_light_val.left > sen_light_val.right) &&
                    ((sen_light_val.left - sen_light_val.right) >= 100)) {
                break;
            }

            calib_timeout--;
            if (calib_timeout == 0) {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }
        sen_light_calib_val.left = sen_light_val.left;

        calib_timeout = CALIBRATION_RIGHT_TIMEOUT;
        motor_run(CIRCLE_RIGHT, 25 / MOTOR_DIV);
        while(1) {
            sensor_light_read(&sen_light_val, &sen_motor_val, CALIBRATION_SAMPLES, 1);
            if ((sen_light_val.right > sen_light_val.left) &&
                    ((sen_light_val.right - sen_light_val.left) >= 100)) {
                break;
            }

            calib_timeout--;
            if (calib_timeout == 0) {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }
        sen_light_calib_val.right = sen_light_val.right;

        calib_timeout = CALIBRATION_CENTER_TIMEOUT;
        motor_run(CIRCLE_LEFT, 25 / MOTOR_DIV);
        while(1) {
            sensor_light_read(&sen_light_val, &sen_motor_val, CALIBRATION_SAMPLES, 1);
            if (abs(sen_light_val.left - sen_light_val.right) <= 20) {
                break;
            }

            calib_timeout--;
            if (calib_timeout == 0) {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        prev_calib_time = bflb_platform_get_time_ms();
        is_calib = true;
    }

    sensor_light_read(&sen_light_val, &sen_motor_val, CALIBRATION_SAMPLES, 1);
    if (sensor_ir_is_result_ready()) {
        is_robot_detect = sensor_ir_is_robot_detect();
        if (is_robot_detect) {
            robot_detect_cnt++;
        } else {
            robot_detect_cnt = 0;
        }

        sensor_ir_start_measure();
    }

    left_diff = sen_light_val.left - sen_light_calib_val.left;
    right_diff = sen_light_val.right - sen_light_calib_val.right;

    if ((left_diff <= TURN_THRESHOLD) || (right_diff <= TURN_THRESHOLD)) {
        if ((left_diff <= TURN_THRESHOLD) && (right_diff <= TURN_THRESHOLD)) {
            if (left_diff < right_diff) {
                input = abs(left_diff);
            } else if (right_diff < left_diff) {
                input = -1 * abs(right_diff);
            }
        } else {
            if (left_diff <= TURN_THRESHOLD) {
                    input = abs(left_diff);
            }

            if (right_diff <= TURN_THRESHOLD) {
                    input = -1 * abs(right_diff);
            }
        }
    } else {
        input = 0;
    }

    // Compute new PID output value
    pid_compute(pid);

    right_speed = 50 + output;
    left_speed = 50 - output;

    if (left_speed < 0) left_speed = 0;
    if (left_speed > 100) left_speed = 100;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 100) right_speed = 100;

    prev_diff_speed = abs(left_speed - right_speed);

    if ((bflb_platform_get_time_ms() - prev_calib_time) < 500) {
        max_speed = 40 / MOTOR_DIV;
        prev_sen_motor_val = sen_motor_val;
    } else if ((bflb_platform_get_time_ms() - prev_calib_time) >= 4500) {
        max_speed = 40 / MOTOR_DIV;
        prev_sen_motor_val = sen_motor_val;
    } else {
        if (prev_diff_speed >= 30) {
            max_speed = 50 / MOTOR_DIV;
        }

        if ((((int)(sen_motor_val.left - prev_sen_motor_val.left) > 30) && 
                (abs(cur_left_speed - prev_left_speed) <= 10)) ||
                (((int)(sen_motor_val.right - prev_sen_motor_val.right) > 30) &&
                (abs(cur_right_speed - prev_right_speed) <= 10))) {
            is_high_spike_detected = true;
            gpio_write(LED_R_0, 0);
            gpio_write(LED_R_1, 0);
        } else {
            is_high_spike_detected = false;
            prev_sen_motor_val = sen_motor_val;
        }

        prev_left_speed = cur_left_speed;
        prev_right_speed = cur_right_speed;

        if (is_high_spike_detected) {
            max_speed = 35 / MOTOR_DIV;
        }
    }

    left_speed = (left_speed * max_speed) / 100;
    right_speed = (right_speed * max_speed) / 100;

    if (robot_detect_cnt) {
        gpio_write(LED_R_0, 0);
        gpio_write(LED_R_1, 0);
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        gpio_write(LED_R_0, 1);
        gpio_write(LED_R_1, 1);
        motor_run_manual(right_speed, left_speed);
        vTaskDelay(pdMS_TO_TICKS(5));

        cur_left_speed = left_speed;
        cur_right_speed = right_speed;
    }

    if (sensor_ir_is_result_ready()) {
        is_robot_detect = sensor_ir_is_robot_detect();
        if (is_robot_detect) {
            robot_detect_cnt++;
        } else {
            robot_detect_cnt = 0;
        }

        sensor_ir_start_measure();
    }

robot_detected:
    if (robot_detect_cnt >= 3) {
        gpio_write(LED_B_0, 1);
        gpio_write(LED_B_1, 1);
        gpio_write(LED_G_0, 1);
        gpio_write(LED_G_1, 1);
        gpio_write(LED_R_0, 0);
        gpio_write(LED_R_1, 0);
        motor_run(STOP, 0);

        robot_detect_cnt = 0;
        vTaskDelay(pdMS_TO_TICKS(2000));
        motor_run(BACKWARD, 30 / MOTOR_DIV);
        vTaskDelay(pdMS_TO_TICKS(300));
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        is_calib = false;
        sensor_ir_clear_calib();
        
        motor_run(CIRCLE_LEFT, 30 / MOTOR_DIV);
        vTaskDelay(pdMS_TO_TICKS(300));
        motor_run(CIRCLE_LEFT, 20 / MOTOR_DIV);

        timeout_turn = TURN_TIMEOUT;

        while (timeout_turn) {
            sensor_light_read(&sen_light_val, &sen_motor_val, 2, 1);

            if ((sen_light_val.right < sen_light_val.left) &&
                    ((sen_light_val.left - sen_light_val.right) >= 150)) {
                motor_run(CIRCLE_RIGHT, 20 / MOTOR_DIV);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
            }

            timeout_turn--;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (timeout_turn == 0) {
            gpio_write(LED_G_0, 0);
            gpio_write(LED_G_1, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_write(LED_G_0, 1);
            gpio_write(LED_G_1, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_write(LED_G_0, 0);
            gpio_write(LED_G_1, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_write(LED_G_0, 1);
            gpio_write(LED_G_1, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_write(LED_G_0, 0);
            gpio_write(LED_G_1, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_write(LED_G_0, 1);
            gpio_write(LED_G_1, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        prev_calib_time = bflb_platform_get_time_ms();
    } else {
        gpio_write(LED_G_0, 0);
        gpio_write(LED_G_1, 0);
        gpio_write(LED_R_0, 1);
        gpio_write(LED_R_1, 1);
    }
}