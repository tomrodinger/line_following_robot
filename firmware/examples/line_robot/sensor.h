#ifndef SENSOR_H
#define SENSOR_H

typedef enum {
    SENSOR_LEFT_IDX = 0,
    SENSOR_RIGHT_IDX
} sensor_idx_t;

void sensor_init(void);
void sensor_read_data(uint32_t *result, uint32_t accum_num);

#endif