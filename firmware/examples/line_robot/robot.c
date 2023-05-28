#include <math.h>
#include "bflb_platform.h"
#include "motor.h"
#include "sensor.h"
#include <FreeRTOS.h>
#include "task.h"
#include "ble_app.h"
#include "pid.h"

#define TURN_THRESHOLD          -20
#define CALIBRATION_SAMPLES     5

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
	pid_limits(pid, -100, 100);
	// Allow PID to compute and change output
	pid_auto(pid);

    prev_calib_time = bflb_platform_get_time_ms();
}

void robot_run(void)
{
    static uint32_t center_base[2] = {0};
    uint32_t center_data[2];
    int32_t left_diff, right_diff;
    static uint32_t prev_diff_speed = 0;
    int left_speed, right_speed;
    static bool is_calib = false;
    sensor_t sen_data;

    if ((bflb_platform_get_time_ms() - prev_calib_time) >= 5000) {
        if (prev_diff_speed <= 40) {
            prev_calib_time = bflb_platform_get_time_ms();
            is_calib = false;
        }
    }

    if (!is_calib) {
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        motor_run(CIRCLE_LEFT, 80);
        vTaskDelay(pdMS_TO_TICKS(120));
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        while (!sensor_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        sensor_read_data(&sen_data);
        while (!sensor_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        sensor_read_data(&sen_data);
        center_base[SENSOR_LEFT_IDX] = sen_data.left;

        motor_run(CIRCLE_RIGHT, 80);
        vTaskDelay(pdMS_TO_TICKS(240));
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        while (!sensor_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        sensor_read_data(&sen_data);
        while (!sensor_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        sensor_read_data(&sen_data);
        center_base[SENSOR_RIGHT_IDX] = sen_data.right;

        motor_run(CIRCLE_LEFT, 80);
        vTaskDelay(pdMS_TO_TICKS(130));
        motor_run(STOP, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        is_calib = true;
    }

    if (sensor_is_robot_detection()) {
        motor_run(STOP, 0);
        do {
            vTaskDelay(pdMS_TO_TICKS(5000));
        } while (sensor_is_robot_detection());

        is_calib = false;
        return;
    }			

    if (sensor_is_ready()) {
        sensor_read_data(&sen_data);
        center_data[SENSOR_LEFT_IDX] = sen_data.left;
        center_data[SENSOR_RIGHT_IDX] = sen_data.right;

        left_diff = center_data[SENSOR_LEFT_IDX] - center_base[SENSOR_LEFT_IDX];
        right_diff = center_data[SENSOR_RIGHT_IDX] - center_base[SENSOR_RIGHT_IDX];

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
        if (left_speed > 80) left_speed = 80;
        if (right_speed < 0) right_speed = 0;
        if (right_speed > 80) right_speed = 80;

        prev_diff_speed = abs(left_speed - right_speed);

        motor_run_manual(right_speed, left_speed);
        vTaskDelay(pdMS_TO_TICKS(20));
    } else {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}