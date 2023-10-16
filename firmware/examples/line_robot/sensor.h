#ifndef SENSOR_H
#define SENSOR_H

typedef struct {
    uint32_t left;
    uint32_t right;
} sensor_light_t;

typedef struct {
    uint32_t left;
    uint32_t right;
} sensor_motor_t;

void sensor_init(void);
void sensor_light_read(sensor_light_t *sen_val, sensor_motor_t *sen_motor_val, uint32_t sample_num, uint32_t sample_motor_num);
void sensor_ir_start_measure(void);
bool sensor_ir_is_robot_detect(void);
bool sensor_ir_is_result_ready(void);
int sensor_ir_store_calib(void);
void sensor_ir_clear_calib(void);
bool sensor_ir_is_measuring(void);

#endif