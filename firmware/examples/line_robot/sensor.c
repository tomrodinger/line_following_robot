#include <math.h>
#include "bflb_platform.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "sensor.h"

#define ADC_CHANNEL 2

struct device *adc_sensor;

void sensor_init(void)
{
    adc_channel_t posChList[] = { ADC_CHANNEL8, ADC_CHANNEL9 };
    adc_channel_t negChList[] = { ADC_CHANNEL_GND, ADC_CHANNEL_GND };
    adc_channel_cfg_t adc_channel_cfg;

    adc_channel_cfg.pos_channel = posChList;
    adc_channel_cfg.neg_channel = negChList;
    adc_channel_cfg.num = ADC_CHANNEL;

    adc_register(ADC0_INDEX, "adc");

    adc_sensor = device_find("adc");

    if (adc_sensor) {
        // ADC_DEV(adc_sensor)->continuous_conv_mode = ENABLE;
        device_open(adc_sensor, DEVICE_OFLAG_STREAM_RX);
        device_control(adc_sensor, DEVICE_CTRL_ADC_VBAT_ON, NULL);
        if (device_control(adc_sensor, DEVICE_CTRL_ADC_CHANNEL_CONFIG, &adc_channel_cfg) == ERROR) {
            MSG("ADC channel config error , Please check the channel corresponding to IO is initial success by board system or Channel is invaild \r\n");
            while (1)
                ;
        }
    }
}

void sensor_read_data(uint32_t *result, uint32_t accum_num)
{
    adc_channel_val_t result_val[ADC_CHANNEL];
    uint32_t left_val = 0;
    uint32_t left_accum_num = 0;
    uint32_t right_val = 0;
    uint32_t right_accum_num = 0;
    int idx;

    while (accum_num--) {
        adc_channel_start(adc_sensor);
        device_read(adc_sensor, 0, (void *)result_val, ADC_CHANNEL);
        for (idx = 0; idx < ADC_CHANNEL; idx++) {
            if (result_val[idx].posChan == ADC_CHANNEL9) {
                left_val += (uint32_t)(result_val[idx].volt * 1000);
                left_accum_num++;
            } else if (result_val[idx].posChan == ADC_CHANNEL8) {
                right_val += (uint32_t)(result_val[idx].volt * 1000);
                right_accum_num++;
            }
        }
    }

    if (left_accum_num) result[SENSOR_LEFT_IDX] = left_val / left_accum_num;
    else result[SENSOR_LEFT_IDX] = 0;
    
    if (right_accum_num) result[SENSOR_RIGHT_IDX] = right_val / right_accum_num;
    else result[SENSOR_RIGHT_IDX] = 0;
}