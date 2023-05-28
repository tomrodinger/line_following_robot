#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <complex.h>
#include <math.h>
#include "bflb_platform.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "sensor.h"

#define ADC_CHANNEL 3
#define ADC_CHANNEL_BUFFER_SIZE 128
#define ADC_MAX_BUFFER_SIZE     (ADC_CHANNEL_BUFFER_SIZE * ADC_CHANNEL)

struct device *adc_sensor;
struct device *dma_sensor;
static uint32_t adc_raw_data[ADC_MAX_BUFFER_SIZE];
static int ir_sen_data[ADC_CHANNEL_BUFFER_SIZE];
static float complex ir_sen_fdata[ADC_CHANNEL_BUFFER_SIZE];
static int ir_sen_fdata_mag[4];
static int ir_sen_fdata_prev_mag[4];
static SemaphoreHandle_t dma_done_sem;
static SemaphoreHandle_t robot_detect_sem;
static StackType_t sensor_stack[512];
static StaticTask_t sensor_task_handle;
static uint8_t sensor_queue[1 * sizeof(sensor_t)];
static StaticQueue_t sensor_st_queue;
static QueueHandle_t sensor_queue_hdl;

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

static void dma_sensor_transfer_done(struct device *dev, void *args, uint32_t size, uint32_t state)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(dma_done_sem, &xHigherPriorityTaskWoken );
}

static void sensor_task(void *pvParameters)
{
    uint16_t idx, ir_idx;
    uint8_t adc_chan;
    uint32_t adc_val;
    sensor_t light_sen_data;
    sensor_t light_sen_count;

    while(1) {
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);

        memset(&light_sen_data, 0, sizeof(light_sen_data));
        memset(&light_sen_count, 0, sizeof(light_sen_count));
        ir_idx = 0;

        for (idx = 0; idx < ADC_MAX_BUFFER_SIZE; idx++) {
            adc_chan = adc_raw_data[idx] >> 21;
            adc_val = (((adc_raw_data[idx] & 0xffff) >> 2) * 2000)  / 16384;

            if (adc_chan == ADC_CHANNEL0) {
                ir_sen_data[ir_idx++] = adc_val;
            } else if (adc_chan == ADC_CHANNEL8) {
                light_sen_data.right += adc_val;
                light_sen_count.right++;
            }  else if (adc_chan == ADC_CHANNEL9) {
                light_sen_data.left += adc_val;
                light_sen_count.left++;
            }
        }

        fft(ir_sen_data, ir_sen_fdata, ADC_CHANNEL_BUFFER_SIZE);
        light_sen_data.right = light_sen_data.right / light_sen_count.right;
        light_sen_data.left = light_sen_data.left / light_sen_count.left;

        /* Sampling frequency: 125000/3 = 41667
         * Max sample : 128
         * Bin 30: 9765 Hz
         * Bin 31: 10091 Hz
         * Bin 32: 10416 Hz
         * Bin 33: 10742 Hz
         */
        memcpy(ir_sen_fdata_prev_mag, ir_sen_fdata_mag, sizeof(ir_sen_fdata_mag));

        ir_sen_fdata_mag[0] = (int)round(sqrt(crealf(ir_sen_fdata[30]) * crealf(ir_sen_fdata[30]) + cimagf(ir_sen_fdata[30]) * cimagf(ir_sen_fdata[30])));
        ir_sen_fdata_mag[1] = (int)round(sqrt(crealf(ir_sen_fdata[31]) * crealf(ir_sen_fdata[31]) + cimagf(ir_sen_fdata[31]) * cimagf(ir_sen_fdata[31])));
        ir_sen_fdata_mag[2] = (int)round(sqrt(crealf(ir_sen_fdata[32]) * crealf(ir_sen_fdata[32]) + cimagf(ir_sen_fdata[32]) * cimagf(ir_sen_fdata[32])));
        ir_sen_fdata_mag[3] = (int)round(sqrt(crealf(ir_sen_fdata[33]) * crealf(ir_sen_fdata[33]) + cimagf(ir_sen_fdata[33]) * cimagf(ir_sen_fdata[33])));
        for (idx = 0; idx < 4; idx++) {
            if (abs(ir_sen_fdata_mag[idx] - ir_sen_fdata_prev_mag[idx]) >= 300) {
                xSemaphoreGive(robot_detect_sem);
                // printf("max_ir_10khz_data %u\r\n", abs(ir_sen_fdata_mag[idx] - ir_sen_fdata_prev_mag[idx]));
                break;
            }
        }

        xQueueSend(sensor_queue_hdl, &light_sen_data, 0);
        dma_reload(dma_sensor, (uint32_t)DMA_ADDR_ADC_RDR, (uint32_t)adc_raw_data, sizeof(adc_raw_data));
        device_control(dma_sensor, DEVICE_CTRL_DMA_CHANNEL_START, NULL);
    }
}

void sensor_init(void)
{
    adc_channel_t posChList[] = { ADC_CHANNEL0, ADC_CHANNEL8, ADC_CHANNEL9 };
    adc_channel_t negChList[] = { ADC_CHANNEL_GND, ADC_CHANNEL_GND, ADC_CHANNEL_GND };
    adc_channel_cfg_t adc_channel_cfg;

    dma_done_sem = xSemaphoreCreateBinary();
    robot_detect_sem = xSemaphoreCreateBinary();
    sensor_queue_hdl = xQueueCreateStatic(1, sizeof(sensor_t), sensor_queue, &sensor_st_queue);

    adc_channel_cfg.pos_channel = posChList;
    adc_channel_cfg.neg_channel = negChList;
    adc_channel_cfg.num = ADC_CHANNEL;

    adc_register(ADC0_INDEX, "adc");

    adc_sensor = device_find("adc");

    if (adc_sensor) {
        device_open(adc_sensor, DEVICE_OFLAG_DMA_RX);
        if (device_control(adc_sensor, DEVICE_CTRL_ADC_CHANNEL_CONFIG, &adc_channel_cfg) == ERROR) {
            MSG("ADC channel config error , Please check the channel corresponding to IO is initial success by board system or Channel is invaild \r\n");
            while (1)
                ;
        }
    }

    dma_register(DMA0_CH1_INDEX, "adc_dma");
    dma_sensor = device_find("adc_dma");

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

    device_set_callback(dma_sensor, dma_sensor_transfer_done);
    dma_reload(dma_sensor, (uint32_t)DMA_ADDR_ADC_RDR, (uint32_t)adc_raw_data, sizeof(adc_raw_data));
    device_control(dma_sensor, DEVICE_CTRL_DMA_CHANNEL_START, NULL);
    adc_channel_start(adc_sensor);

    xTaskCreateStatic(sensor_task, (char *)"sensor", sizeof(sensor_stack) / 4, NULL, configMAX_PRIORITIES - 4, sensor_stack, &sensor_task_handle);
}

int sensor_is_ready(void)
{
    return (uxQueueMessagesWaiting(sensor_queue_hdl) > 0);
}

void sensor_read_data(sensor_t *sen_data)
{
    xQueueReceive(sensor_queue_hdl, sen_data, 0);
}

int sensor_is_robot_detection(void)
{
    return (xSemaphoreTake(robot_detect_sem, 0) == pdTRUE);
}