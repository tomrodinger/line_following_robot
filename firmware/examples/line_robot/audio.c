#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <math.h>
#include "bflb_platform.h"
#include "hal_dac.h"
#include "hal_dma.h"
#include "hal_gpio.h"
#include "mp3dec.h"
#include "mp3_data.h"
#include "motor.h"
#include "ring_buffer.h"

#define READBUF_SIZE		(1940)	/* feel free to change this, but keep big enough for >= one frame at high bitrates */
#define MAX_ARM_FRAMES		100
#define ARMULATE_MUL_FACT	1

#define AUDIO_SHUTDOWN_PIN  GPIO_PIN_6

static uint32_t audio_file_idx = 0;
static struct device *dac;
static struct device *dac_dma;
static SemaphoreHandle_t dma_done_sem;
static unsigned char readBuf[READBUF_SIZE];
static short outBuf[MAX_NGRAN * MAX_NSAMP];
static short pcm_out[MAX_NSAMP];
static short audio_rb_buf[MAX_NGRAN * MAX_NSAMP * 1];
HMP3Decoder hMP3Decoder;
static StackType_t audio_stack[512];
static StaticTask_t audio_task_handle;
static Ring_Buffer_Type audio_rb;

#define AUDIO_VOLUME_MAX_GAIN   10

#define LOW_VOLUME      0
#define MEDIUM_VOLUME   1
#define MAX_VOLUME      2

#define AUDIO_VOLUME    MAX_VOLUME

#if AUDIO_VOLUME == LOW_VOLUME
#define AUDIO_MAX_BITS  8
#elif AUDIO_VOLUME == MEDIUM_VOLUME
#define AUDIO_MAX_BITS  9
#else
#define AUDIO_MAX_BITS  10
#endif

typedef enum {
    AUDIO_STATE_INIT = 0,
    AUDIO_STATE_LOAD_DATA,
    AUDIO_STATE_WAIT_DMA
} audio_state_t;

static void dma_transfer_done(struct device *dev, void *args, uint32_t size, uint32_t state)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (Ring_Buffer_Get_Length(&audio_rb)) {
        uint32_t avalibleCnt = Ring_Buffer_Read(&audio_rb, (uint8_t*)pcm_out, sizeof(pcm_out));

        if (avalibleCnt) {
            device_write(dac, DAC_CHANNEL_1, pcm_out, avalibleCnt);
        }
    }

    xSemaphoreGiveFromISR(dma_done_sem, &xHigherPriorityTaskWoken );
}

static int FillReadBuffer(unsigned char *readBuf, unsigned char *readPtr, int bufSize, int bytesLeft)
{
	int nRead;
    uint32_t read_length = bufSize - bytesLeft;
    read_length = ((read_length + audio_file_idx) <= mp3_data_length) ? 
                                                read_length: 
                                                (mp3_data_length - audio_file_idx);

	/* move last, small chunk from end of buffer to start, then fill with new data */
	memmove(readBuf, readPtr, bytesLeft);	
    memcpy(readBuf + bytesLeft, &mp3_data[audio_file_idx], read_length);
    audio_file_idx += read_length;			
	nRead = read_length;
	/* zero-pad to avoid finding false sync word after last frame (from old data in readBuf) */
	if (nRead < bufSize - bytesLeft)
		memset(readBuf + bytesLeft + nRead, 0, bufSize - bytesLeft - nRead);	

	return nRead;
}

static void audio_to_10_bit(short *in, short *out, int count) 
{ 
    static int32_t m = 0;
    uint32_t idx;
    short max = 0;
    float mul;
    int i, j;
    short o;

    idx = 0;
    while (idx < count) {
        if (max < abs(in[idx])) {
            max = abs(in[idx]);
        }
        idx++;
    }

    mul = 32768 / max;
    idx = 0;

    if (mul >= AUDIO_VOLUME_MAX_GAIN) {
        mul = AUDIO_VOLUME_MAX_GAIN;
    } 
       
    while(idx < count) { 
        i = (short)((float)(in[idx]) * mul);
        // i = in[idx]/2;
        
        i += m; 
        j = i + 32768 - pow(2, AUDIO_MAX_BITS) / 2; 

        if( j < 0 ) { 
            o = 0; 
        } else if( j > 65535 ) { 
            o = pow(2, AUDIO_MAX_BITS) - 1; 
        } else { 
            o = ((j>>(16 - AUDIO_MAX_BITS))&((uint32_t)(pow(2, AUDIO_MAX_BITS) - 1))); 
        } 
        
        m = ((j-32768+(pow(2, AUDIO_MAX_BITS) / 2))-i);
        out[idx] = o;
        idx++;
    }
}

#if 0
void audio_run(void)
{
    static audio_state_t audio_state = AUDIO_STATE_INIT;
    static int bytesLeft, eofReached;
    static unsigned char *readPtr;
    MP3FrameInfo mp3FrameInfo;
    int nRead, err, offset;

    switch(audio_state) {
        case AUDIO_STATE_INIT:
            bytesLeft = 0;
            eofReached = 0;
            nRead = 0;
            audio_file_idx = 0;
            pcm_length = 0;
            pcm_dma_idx = 0;
            readPtr = readBuf;

            audio_state = AUDIO_STATE_LOAD_DATA;
            break;
        case AUDIO_STATE_LOAD_DATA:
            motor_run(STOP, 0);
            /* somewhat arbitrary trigger to refill buffer - should always be enough for a full frame */
            if (bytesLeft < 2*MAINBUF_SIZE && !eofReached) {
                nRead = FillReadBuffer(readBuf, readPtr, READBUF_SIZE, bytesLeft);
                bytesLeft += nRead;
                readPtr = readBuf;
                if (nRead == 0)
                    eofReached = 1;
            }

            /* find start of next MP3 frame - assume EOF if no sync found */
            offset = MP3FindSyncWord(readPtr, bytesLeft);
            if (offset < 0) {
                audio_state = AUDIO_STATE_INIT;
                break;
            }
            readPtr += offset;
            bytesLeft -= offset;

            /* decode one MP3 frame - if offset < 0 then bytesLeft was less than a full frame */
            err = MP3Decode(hMP3Decoder, &readPtr, &bytesLeft, outBuf, 0);

            if (err) {
                /* error occurred */
                switch (err) {
                case ERR_MP3_INDATA_UNDERFLOW:
                    audio_state = AUDIO_STATE_INIT;
                    break;
                case ERR_MP3_MAINDATA_UNDERFLOW:
                    /* do nothing - next call to decode will provide more mainData */
                    break;
                case ERR_MP3_FREE_BITRATE_SYNC:
                default:
                    audio_state = AUDIO_STATE_INIT;
                    break;
                }
            } else {
                /* no error */
                MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
                audio_to_10_bit(outBuf, &pcm_out[pcm_dma_idx][0], mp3FrameInfo.outputSamps);
                
                if (dma_channel_check_busy(dac_dma) != SET) {
                    device_write(dac, DAC_CHANNEL_1, &pcm_out[pcm_dma_idx][0], mp3FrameInfo.outputSamps * 2);
                    pcm_dma_idx++;
                    if (pcm_dma_idx >= 2) pcm_dma_idx = 0;
                } else {
                    pcm_length = mp3FrameInfo.outputSamps * 2;
                    audio_state = AUDIO_STATE_WAIT_DMA;
                }
            }
            break;
        case AUDIO_STATE_WAIT_DMA:
            if (xSemaphoreTake(dma_done_sem, 0) == pdTRUE) {
                audio_state = AUDIO_STATE_LOAD_DATA;
                pcm_dma_idx++;
                if (pcm_dma_idx >= 2) pcm_dma_idx = 0;
            }
            break;
        default:
            break;
    }
}
#endif

static void ringbuffer_lock()
{
    
}

static void ringbuffer_unlock()
{
    
}

static void audio_task(void *pvParameters)
{
    audio_state_t audio_state = AUDIO_STATE_INIT;
    int bytesLeft, eofReached;
    unsigned char *readPtr;
    MP3FrameInfo mp3FrameInfo;
    int nRead, err, offset;
    uint32_t avail_cnt;

    hMP3Decoder = MP3InitDecoder();
    Ring_Buffer_Init(&audio_rb, (uint8_t*)audio_rb_buf, sizeof(audio_rb_buf), ringbuffer_lock, ringbuffer_unlock);

    while(1) {
        switch(audio_state) {
            case AUDIO_STATE_INIT:
                if (dma_channel_check_busy(dac_dma) == SET) {
                    do {
                        vTaskDelay(pdMS_TO_TICKS(100));
                    } while (Ring_Buffer_Get_Length(&audio_rb));
                }

                xSemaphoreTake(dma_done_sem, 0);

                bytesLeft = 0;
                eofReached = 0;
                nRead = 0;
                audio_file_idx = 0;
                readPtr = readBuf;
                audio_state = AUDIO_STATE_LOAD_DATA;
                break;
            case AUDIO_STATE_LOAD_DATA:
                /* somewhat arbitrary trigger to refill buffer - should always be enough for a full frame */
                if (bytesLeft < 2*MAINBUF_SIZE && !eofReached) {
                    nRead = FillReadBuffer(readBuf, readPtr, READBUF_SIZE, bytesLeft);
                    bytesLeft += nRead;
                    readPtr = readBuf;
                    if (nRead == 0)
                        eofReached = 1;
                }

                /* find start of next MP3 frame - assume EOF if no sync found */
                offset = MP3FindSyncWord(readPtr, bytesLeft);
                if (offset < 0) {
                    audio_state = AUDIO_STATE_INIT;
                    break;
                }
                readPtr += offset;
                bytesLeft -= offset;

                /* decode one MP3 frame - if offset < 0 then bytesLeft was less than a full frame */
                err = MP3Decode(hMP3Decoder, &readPtr, &bytesLeft, outBuf, 0);

                if (err) {
                    /* error occurred */
                    switch (err) {
                    case ERR_MP3_INDATA_UNDERFLOW:
                        audio_state = AUDIO_STATE_INIT;
                        break;
                    case ERR_MP3_MAINDATA_UNDERFLOW:
                        /* do nothing - next call to decode will provide more mainData */
                        break;
                    case ERR_MP3_FREE_BITRATE_SYNC:
                    default:
                        audio_state = AUDIO_STATE_INIT;
                        break;
                    }
                } else {
                    /* no error */
                    MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
                    audio_to_10_bit(outBuf, outBuf, mp3FrameInfo.outputSamps);

                    if ((mp3FrameInfo.outputSamps * 2) > Ring_Buffer_Get_Empty_Length(&audio_rb)) {
                        do {
                            xSemaphoreTake(dma_done_sem, portMAX_DELAY);
                        } while ((mp3FrameInfo.outputSamps * 2) > Ring_Buffer_Get_Empty_Length(&audio_rb));
                    }

                    Ring_Buffer_Write(&audio_rb, (uint8_t *)outBuf, mp3FrameInfo.outputSamps * 2);

                    if (dma_channel_check_busy(dac_dma) != SET) {
                        avail_cnt = Ring_Buffer_Read(&audio_rb, (uint8_t*)pcm_out, sizeof(pcm_out));
                        device_write(dac, DAC_CHANNEL_1, pcm_out, avail_cnt);
                    }
                }
                break;
            default:
                break;
        }
    }
}

void audio_init(void)
{
    /* Enable OP-AMP */
    gpio_set_mode(AUDIO_SHUTDOWN_PIN, GPIO_OUTPUT_PP_MODE);
    gpio_write(AUDIO_SHUTDOWN_PIN, 0);

    dma_done_sem = xSemaphoreCreateBinary();

    dac_register(DAC0_INDEX, "dac");
    dac = device_find("dac");

    if (dac) {
        device_open(dac, DEVICE_OFLAG_DMA_TX);
    }

    dma_register(DMA0_CH0_INDEX, "dac_dma");
    dac_dma = device_find("dac_dma");

    if (dac_dma) {
        DMA_DEV(dac_dma)->direction = DMA_MEMORY_TO_PERIPH;
        DMA_DEV(dac_dma)->transfer_mode = DMA_LLI_ONCE_MODE;
        DMA_DEV(dac_dma)->src_req = DMA_REQUEST_NONE;
        DMA_DEV(dac_dma)->dst_req = DMA_REQUEST_DAC0;
        DMA_DEV(dac_dma)->src_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
        DMA_DEV(dac_dma)->dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
        DMA_DEV(dac_dma)->src_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dac_dma)->dst_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dac_dma)->src_width = DMA_TRANSFER_WIDTH_16BIT;
        DMA_DEV(dac_dma)->dst_width = DMA_TRANSFER_WIDTH_16BIT;
        device_open(dac_dma, 0);
    }

    /* connect dac device and dma device */
    device_set_callback(dac_dma, dma_transfer_done);
    device_control(dac_dma, DEVICE_CTRL_SET_INT, NULL);
    device_control(dac, DEVICE_CTRL_ATTACH_TX_DMA, dac_dma);

    xTaskCreateStatic(audio_task, (char *)"audio", sizeof(audio_stack) / 4, NULL, configMAX_PRIORITIES - 3, audio_stack, &audio_task_handle);
}