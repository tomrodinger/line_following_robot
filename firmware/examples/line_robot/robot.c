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

#define LED_R_0  GPIO_PIN_22
#define LED_R_1  GPIO_PIN_0

#define LED_G_0  GPIO_PIN_24
#define LED_G_1  GPIO_PIN_2

// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata;
pid_control_t pid;

// Control loop input,output and setpoint variables
float input = 0, output = 0;
float setpoint = 0;

// Control loop gains
float kp = 0.1, ki = 0.0001, kd = 0.005;
uint32_t prev_calib_time;

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
}

void robot_run(void)
{
    static sensor_light_t sen_light_calib_val;
    static uint32_t prev_diff_speed = 0;
    static bool is_calib = false;
    sensor_light_t sen_light_val;
    int32_t left_diff, right_diff;
    int left_speed, right_speed, loop, detect_cnt;
    bool is_robot_detect = false;
    int max_speed = 100;

    if ((bflb_platform_get_time_ms() - prev_calib_time) >= 5000) {
        if (prev_diff_speed <= 40) {
            prev_calib_time = bflb_platform_get_time_ms();
            is_calib = false;
        }
    }

    if (!is_calib) {
        motor_run(STOP, 0);

        do {
            sensor_ir_start_measure();
            while (sensor_ir_is_measuring()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        } while (!sensor_ir_store_calib());
        
        motor_run(CIRCLE_LEFT, 45);
        while(1) {
            sensor_light_read(&sen_light_val, CALIBRATION_SAMPLES);
            if ((sen_light_val.left > sen_light_val.right) &&
                    ((sen_light_val.left - sen_light_val.right) >= 100)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        sen_light_calib_val.left = sen_light_val.left;

        motor_run(CIRCLE_RIGHT, 45);
        while(1) {
            sensor_light_read(&sen_light_val, CALIBRATION_SAMPLES);
            if ((sen_light_val.right > sen_light_val.left) &&
                    ((sen_light_val.right - sen_light_val.left) >= 100)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        sen_light_calib_val.right = sen_light_val.right;

        motor_run(CIRCLE_LEFT, 45);
        while(1) {
            sensor_light_read(&sen_light_val, CALIBRATION_SAMPLES);
            if (abs(sen_light_val.left - sen_light_val.right) <= 20) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(50));

        is_calib = true;
    }

    sensor_light_read(&sen_light_val, CALIBRATION_SAMPLES);
    if (!sensor_ir_is_measuring()) {
        is_robot_detect = sensor_ir_is_robot_detect();
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
        max_speed = 40;
    }

    if ((bflb_platform_get_time_ms() - prev_calib_time) >= 4500) {
        max_speed = 60;
    }

    if (prev_diff_speed >= 20) {
        max_speed = 70;
    }

    left_speed = (left_speed * max_speed) / 100;
    right_speed = (right_speed * max_speed) / 100;

    if (!is_robot_detect) {
        motor_run_manual(right_speed, left_speed);

        loop = 5;

        while (loop--) {
            if (!sensor_ir_is_measuring()) {
                if (sensor_ir_is_robot_detect()) {
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if ((is_robot_detect) || (sensor_ir_is_robot_detect())) {
        gpio_write(LED_G_0, 1);
        gpio_write(LED_G_1, 1);
        gpio_write(LED_R_0, 0);
        gpio_write(LED_R_1, 0);
        motor_run(STOP, 0);

        detect_cnt = 0;

        do {
            detect_cnt++;
            sensor_ir_start_measure();
            while (sensor_ir_is_measuring()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (detect_cnt >= 5) {
                break;
            }
        } while (sensor_ir_is_robot_detect());

        if (detect_cnt >= 5) {
            motor_run(BACKWARD, 80);
            vTaskDelay(pdMS_TO_TICKS(200));
            motor_run(STOP, 0);
            vTaskDelay(pdMS_TO_TICKS(100));

            is_calib = false;
            
            motor_run(CIRCLE_LEFT, 35);
            vTaskDelay(pdMS_TO_TICKS(200));

            while (1) {
                sensor_light_read(&sen_light_val, CALIBRATION_SAMPLES);

                if ((sen_light_val.left < sen_light_val.right) &&
                        ((sen_light_val.right - sen_light_val.left) >= 150)) {
                    break;
                }
            }

            motor_run(CIRCLE_LEFT, 80);
            vTaskDelay(pdMS_TO_TICKS(50));
            motor_run(STOP, 0);
            vTaskDelay(pdMS_TO_TICKS(100));

            prev_calib_time = bflb_platform_get_time_ms();
        }
    } else {
        gpio_write(LED_G_0, 0);
        gpio_write(LED_G_1, 0);
        gpio_write(LED_R_0, 1);
        gpio_write(LED_R_1, 1);
    }
}