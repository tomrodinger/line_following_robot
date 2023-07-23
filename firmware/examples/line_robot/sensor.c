#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <complex.h>
#include <math.h>
#include "bflb_platform.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "sensor.h"

#define SENSOR_LIGHT_CHAN_NUM   2
#define SENSOR_IR_CHAN_NUM      1

#define SENSOR_IR_RAW_SIZE      128
#define SENSOR_IR_DETECT_THRESHOLD  70

static struct device *adc_sensor;
static struct device *dma_sensor;
static volatile bool sensor_ir_is_sampling = false;
static volatile bool sensor_ir_is_processing = false;
static volatile bool sensor_ir_is_detect = false;
static SemaphoreHandle_t dma_done_sem;
static uint32_t sensor_ir_raw[SENSOR_IR_RAW_SIZE];
static int ir_sen_data[SENSOR_IR_RAW_SIZE];
static float complex ir_sen_fdata[SENSOR_IR_RAW_SIZE];
static volatile int ir_sen_fdata_mag[4];
static volatile int ir_sen_fdata_prev_mag[4] = {0};
static StackType_t sensor_stack[512];
static StaticTask_t sensor_task_handle;

#define PI 3.14159265358979323846

static void fft_radix2(int* x, float complex* X, unsigned int N, unsigned int s) {
    unsigned int k;
    float complex t;

    // At the lowest level pass through (delta T=0 means no phase).
    if (N == 1) {
        X[0] = x[0];
        return;
    }

    // Cooley-Tukey: recursively split in two, then combine beneath.
    fft_radix2(x, X, N/2, 2*s);
    fft_radix2(x+s, X + N/2, N/2, 2*s);

    for (k = 0; k < N/2; k++) {
        t = X[k];
        X[k] = t + cexp(-2 * PI * I * k / N) * X[k + N/2];
        X[k + N/2] = t - cexp(-2 * PI * I * k / N) * X[k + N/2];
    }
}

static void fft(int* x, float complex* X, unsigned int N) {
    fft_radix2(x, X, N, 1);
}

#undef I
#include "hal_dma.h"

static void sensor_ir_dma_done(struct device *dev, void *args, uint32_t size, uint32_t state)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(dma_done_sem, &xHigherPriorityTaskWoken );
}

static void sensor_ir_processing_task(void *pvParameters)
{
    uint8_t adc_chan;
    uint32_t adc_val;
    uint16_t idx, ir_idx;
    uint8_t over_threshold_cnt;

    while(1) {
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);
        sensor_ir_is_sampling = false;
        sensor_ir_is_processing = true;

        ir_idx = 0;
        for (idx = 0; idx < SENSOR_IR_RAW_SIZE; idx++) {
            adc_chan = sensor_ir_raw[idx] >> 21;
            adc_val = (((sensor_ir_raw[idx] & 0xffff) >> 2) * 2000)  / 16384;

            if (adc_chan == ADC_CHANNEL0) {
                ir_sen_data[ir_idx++] = adc_val;
            }
        }

        fft(ir_sen_data, ir_sen_fdata, ir_idx);

        /* Sampling frequency: 125000/3 = 41667
        * Max sample : 128
        * Bin 30: 9765 Hz
        * Bin 31: 10091 Hz
        * Bin 32: 10416 Hz
        * Bin 33: 10742 Hz
        */

        ir_sen_fdata_mag[0] = (int)round(sqrt(crealf(ir_sen_fdata[30]) * crealf(ir_sen_fdata[30]) + cimagf(ir_sen_fdata[30]) * cimagf(ir_sen_fdata[30])));
        ir_sen_fdata_mag[1] = (int)round(sqrt(crealf(ir_sen_fdata[31]) * crealf(ir_sen_fdata[31]) + cimagf(ir_sen_fdata[31]) * cimagf(ir_sen_fdata[31])));
        ir_sen_fdata_mag[2] = (int)round(sqrt(crealf(ir_sen_fdata[32]) * crealf(ir_sen_fdata[32]) + cimagf(ir_sen_fdata[32]) * cimagf(ir_sen_fdata[32])));
        ir_sen_fdata_mag[3] = (int)round(sqrt(crealf(ir_sen_fdata[33]) * crealf(ir_sen_fdata[33]) + cimagf(ir_sen_fdata[33]) * cimagf(ir_sen_fdata[33])));

        over_threshold_cnt = 0;

        for (idx = 0; idx < 4; idx++) {
            if ((ir_sen_fdata_mag[idx] - ir_sen_fdata_prev_mag[idx]) >= SENSOR_IR_DETECT_THRESHOLD) {
                over_threshold_cnt++;
            }
        }

        if (over_threshold_cnt >= 1) {
            sensor_ir_is_detect = true;
        } else {
            sensor_ir_is_detect = false;
        }

        sensor_ir_is_processing = false;
    }
}

void sensor_init(void)
{
    adc_register(ADC0_INDEX, "adc");

    adc_sensor = device_find("adc");
    device_open(adc_sensor, DEVICE_OFLAG_STREAM_RX);

    dma_register(DMA0_CH1_INDEX, "adc_dma");
    dma_sensor = device_find("adc_dma");

    dma_done_sem = xSemaphoreCreateBinary();

    if (dma_sensor) {
        DMA_DEV(dma_sensor)->direction = DMA_PERIPH_TO_MEMORY;
        DMA_DEV(dma_sensor)->transfer_mode = DMA_LLI_ONCE_MODE;
        DMA_DEV(dma_sensor)->src_req = DMA_REQUEST_ADC0;
        DMA_DEV(dma_sensor)->dst_req = DMA_REQUEST_NONE;
        DMA_DEV(dma_sensor)->src_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
        DMA_DEV(dma_sensor)->dst_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
        DMA_DEV(dma_sensor)->src_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dma_sensor)->dst_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dma_sensor)->src_width = DMA_TRANSFER_WIDTH_32BIT;
        DMA_DEV(dma_sensor)->dst_width = DMA_TRANSFER_WIDTH_32BIT;
        device_open(dma_sensor, 0);
    }

    device_control(dma_sensor, DEVICE_CTRL_SET_INT, NULL);
    device_set_callback(dma_sensor, sensor_ir_dma_done);

    xTaskCreateStatic(sensor_ir_processing_task, (char *)"sensor", sizeof(sensor_stack) / 4, NULL, configMAX_PRIORITIES - 4, sensor_stack, &sensor_task_handle);
}

void sensor_light_read(sensor_light_t *sen_val, uint32_t sample_num)
{
    adc_channel_val_t adc_val[SENSOR_LIGHT_CHAN_NUM];
    adc_channel_t posChList[] = { ADC_CHANNEL8, ADC_CHANNEL9 };
    adc_channel_t negChList[] = { ADC_CHANNEL_GND, ADC_CHANNEL_GND };
    adc_channel_cfg_t adc_channel_cfg;
    uint32_t left_val = 0;
    uint32_t left_sample_num = 0;
    uint32_t right_val = 0;
    uint32_t right_sample_num = 0;
    int idx;

    while (sensor_ir_is_sampling) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    adc_channel_stop(adc_sensor);
    device_close(adc_sensor);

    device_open(adc_sensor, DEVICE_OFLAG_STREAM_RX);

    ADC_DEV(adc_sensor)->continuous_conv_mode = false;

    adc_channel_cfg.pos_channel = posChList;
    adc_channel_cfg.neg_channel = negChList;
    adc_channel_cfg.num = SENSOR_LIGHT_CHAN_NUM;

    if (device_control(adc_sensor, DEVICE_CTRL_ADC_CHANNEL_CONFIG, &adc_channel_cfg) == ERROR) {
        return;
    }

    while (sample_num--) {
        adc_channel_start(adc_sensor);
        device_read(adc_sensor, 0, (void *)adc_val, SENSOR_LIGHT_CHAN_NUM);
        for (idx = 0; idx < SENSOR_LIGHT_CHAN_NUM; idx++) {
            if (adc_val[idx].posChan == ADC_CHANNEL9) {
                left_val += (uint32_t)(adc_val[idx].volt * 1000);
                left_sample_num++;
            } else if (adc_val[idx].posChan == ADC_CHANNEL8) {
                right_val += (uint32_t)(adc_val[idx].volt * 1000);
                right_sample_num++;
            }
        }
    }

    if (left_sample_num) sen_val->left = left_val / left_sample_num;
    else sen_val->left = 0;
    
    if (right_sample_num) sen_val->right = right_val / right_sample_num;
    else sen_val->right = 0;
}

void sensor_ir_start_measure(void)
{
    adc_channel_t posChList[] = { ADC_CHANNEL0 };
    adc_channel_t negChList[] = { ADC_CHANNEL_GND };
    adc_channel_cfg_t adc_channel_cfg;

    if ((!sensor_ir_is_sampling) && (!sensor_ir_is_processing)) {
        xSemaphoreTake(dma_done_sem, 0);

        adc_channel_stop(adc_sensor);
        device_close(adc_sensor);

        device_open(adc_sensor, DEVICE_OFLAG_DMA_RX);

        ADC_DEV(adc_sensor)->continuous_conv_mode = true;

        adc_channel_cfg.pos_channel = posChList;
        adc_channel_cfg.neg_channel = negChList;
        adc_channel_cfg.num = SENSOR_IR_CHAN_NUM;

        if (device_control(adc_sensor, DEVICE_CTRL_ADC_CHANNEL_CONFIG, &adc_channel_cfg) == ERROR) {
            return;
        }

        dma_reload(dma_sensor, (uint32_t)DMA_ADDR_ADC_RDR, (uint32_t)sensor_ir_raw, sizeof(sensor_ir_raw));
        device_control(dma_sensor, DEVICE_CTRL_DMA_CHANNEL_START, NULL);
        adc_channel_start(adc_sensor);

        sensor_ir_is_sampling = true;
    }
}

bool sensor_ir_is_robot_detect(void)
{
    return sensor_ir_is_detect;
}

int sensor_ir_store_calib(void)
{
    int idx;
    int is_calib_valid = 1;

    for (idx = 0; idx < 4; idx++) {
        if (ir_sen_fdata_mag[idx] > 200) {
            is_calib_valid = 0;
            break;
        }
    }

    if (is_calib_valid) {
        sensor_ir_is_detect = false;
        memcpy((void*)ir_sen_fdata_prev_mag, (void*)ir_sen_fdata_mag, sizeof(ir_sen_fdata_mag));
    }

    return is_calib_valid;
}

bool sensor_ir_is_measuring(void)
{
    return ((sensor_ir_is_sampling) || (sensor_ir_is_processing));
}