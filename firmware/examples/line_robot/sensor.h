#ifndef SENSOR_H
#define SENSOR_H

typedef enum {
    SENSOR_LEFT_IDX = 0,
    SENSOR_RIGHT_IDX,
    SENSOR_IR_IDX,
    SENSOR_MAX
} sensor_idx_t;

typedef struct {
    uint32_t left;
    uint32_t right;
} sensor_t;

void sensor_init(void);
int sensor_is_ready(void);
void sensor_read_data(sensor_t *sen_data);
int sensor_is_robot_detection(void);

#endif