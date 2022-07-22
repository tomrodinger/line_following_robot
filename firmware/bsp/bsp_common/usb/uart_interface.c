/**
 * @file uart_interface.c
 * @brief
 *
 * Copyright (c) 2021 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */
#include "bflb_platform.h"
#include "hal_gpio.h"
#include "uart_interface.h"
#include "hal_usb.h"
#include "hal_dma.h"

#ifndef USB_MULTIPLE_CDC

#define USB_OUT_RINGBUFFER_SIZE (8 * 1024)
#define UART_RX_RINGBUFFER_SIZE (8 * 1024)
#define UART_TX_DMA_SIZE        (4095)

uint8_t usb_rx_mem[USB_OUT_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));
uint8_t uart_rx_mem[UART_RX_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));

uint8_t src_buffer[UART_TX_DMA_SIZE] __attribute__((section(".tcm_code")));

struct device *uart1;
struct device *dma_ch2;

Ring_Buffer_Type usb_rx_rb;
Ring_Buffer_Type uart1_rx_rb;

void uart_irq_callback(struct device *dev, void *args, uint32_t size, uint32_t state)
{
    if (state == UART_EVENT_RX_FIFO) {
        if (size && size < Ring_Buffer_Get_Empty_Length(&uart1_rx_rb)) {
            Ring_Buffer_Write(&uart1_rx_rb, (uint8_t *)args, size);
        } else {
            MSG("RF OV\r\n");
        }
    } else if (state == UART_EVENT_RTO) {
        if (size && size < Ring_Buffer_Get_Empty_Length(&uart1_rx_rb)) {
            Ring_Buffer_Write(&uart1_rx_rb, (uint8_t *)args, size);
        } else {
            MSG("RTO OV\r\n");
        }
    } else if (state == UART_RX_FER_IT) {
        MSG("RX ERR\r\n");
    }
}
void uart1_init(void)
{
    uart_register(UART1_INDEX, "uart1");
    uart1 = device_find("uart1");

    if (uart1) {
        // device_open(uart1, DEVICE_OFLAG_DMA_TX | DEVICE_OFLAG_INT_RX);
        // device_set_callback(uart1, uart_irq_callback);
        // device_control(uart1, DEVICE_CTRL_SET_INT, (void *)(UART_RX_FIFO_IT | UART_RTO_IT));
    }

    dma_register(DMA0_CH2_INDEX, "ch2");
    dma_ch2 = device_find("ch2");

    if (dma_ch2) {
        DMA_DEV(dma_ch2)->direction = DMA_MEMORY_TO_PERIPH;
        DMA_DEV(dma_ch2)->transfer_mode = DMA_LLI_ONCE_MODE;
        DMA_DEV(dma_ch2)->src_req = DMA_REQUEST_NONE;
        DMA_DEV(dma_ch2)->dst_req = DMA_REQUEST_UART1_TX;
        DMA_DEV(dma_ch2)->src_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
        DMA_DEV(dma_ch2)->dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
        DMA_DEV(dma_ch2)->src_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dma_ch2)->dst_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dma_ch2)->src_width = DMA_TRANSFER_WIDTH_8BIT;
        DMA_DEV(dma_ch2)->dst_width = DMA_TRANSFER_WIDTH_8BIT;
        device_open(dma_ch2, 0);
    }
}

void uart1_config(uint32_t baudrate, uart_databits_t databits, uart_parity_t parity, uart_stopbits_t stopbits)
{
    device_close(uart1);
    UART_DEV(uart1)->baudrate = baudrate;
    UART_DEV(uart1)->stopbits = stopbits;
    UART_DEV(uart1)->parity = parity;
    UART_DEV(uart1)->databits = (databits - 5);
    device_open(uart1, DEVICE_OFLAG_DMA_TX | DEVICE_OFLAG_INT_RX);
    device_set_callback(uart1, uart_irq_callback);
    device_control(uart1, DEVICE_CTRL_SET_INT, (void *)(UART_RX_FIFO_IT | UART_RTO_IT));
    Ring_Buffer_Reset(&usb_rx_rb);
    Ring_Buffer_Reset(&uart1_rx_rb);
}

static uint8_t uart1_dtr;
static uint8_t uart1_rts;

void uart1_set_dtr_rts(uint8_t dtr, uint8_t rts)
{
    uart1_dtr = dtr;
    uart1_rts = rts;
}

void uart1_dtr_init(void)
{
    gpio_set_mode(uart1_dtr, GPIO_OUTPUT_MODE);
}
void uart1_rts_init(void)
{
    gpio_set_mode(uart1_rts, GPIO_OUTPUT_MODE);
}
void uart1_dtr_deinit(void)
{
    gpio_set_mode(uart1_dtr, GPIO_INPUT_MODE);
}
void uart1_rts_deinit(void)
{
    gpio_set_mode(uart1_rts, GPIO_INPUT_MODE);
}
void dtr_pin_set(uint8_t status)
{
    gpio_write(uart1_dtr, status);
}
void rts_pin_set(uint8_t status)
{
    gpio_write(uart1_rts, status);
}
void ringbuffer_lock()
{
    cpu_global_irq_disable();
}
void ringbuffer_unlock()
{
    cpu_global_irq_enable();
}

void uart_ringbuffer_init(void)
{
    /* init mem for ring_buffer */
    memset(usb_rx_mem, 0, USB_OUT_RINGBUFFER_SIZE);
    memset(uart_rx_mem, 0, UART_RX_RINGBUFFER_SIZE);

    /* init ring_buffer */
    Ring_Buffer_Init(&usb_rx_rb, usb_rx_mem, USB_OUT_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
    Ring_Buffer_Init(&uart1_rx_rb, uart_rx_mem, UART_RX_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
}

static dma_control_data_t uart_dma_ctrl_cfg = {
    .bits.fix_cnt = 0,
    .bits.dst_min_mode = 0,
    .bits.dst_add_mode = 0,
    .bits.SI = 1,
    .bits.DI = 0,
    .bits.SWidth = DMA_TRANSFER_WIDTH_8BIT,
    .bits.DWidth = DMA_TRANSFER_WIDTH_8BIT,
    .bits.SBSize = 0,
    .bits.DBSize = 0,
    .bits.I = 0,
    .bits.TransferSize = 4095
};
static dma_lli_ctrl_t uart_lli_list = {
    .src_addr = (uint32_t)src_buffer,
    .dst_addr = DMA_ADDR_UART1_TDR,
    .nextlli = 0
};

void uart_send_from_ringbuffer(void)
{
    if (Ring_Buffer_Get_Length(&usb_rx_rb)) {
        if (!dma_channel_check_busy(dma_ch2)) {
            uint32_t avalibleCnt = Ring_Buffer_Read(&usb_rx_rb, src_buffer, UART_TX_DMA_SIZE);

            if (avalibleCnt) {
                dma_channel_stop(dma_ch2);
                uart_dma_ctrl_cfg.bits.TransferSize = avalibleCnt;
                memcpy(&uart_lli_list.cfg, &uart_dma_ctrl_cfg, sizeof(dma_control_data_t));
                dma_channel_update(dma_ch2, (void *)((uint32_t)&uart_lli_list));
                dma_channel_start(dma_ch2);
            }
        }
    }
}

#else

#define USB_OUT_RINGBUFFER_SIZE (1024)
#define UART_RX_RINGBUFFER_SIZE (1024)
#define UART_TX_DMA_SIZE        (1024)

uint8_t usb_rx_mem[2][USB_OUT_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));
uint8_t uart_rx_mem[2][UART_RX_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));

uint8_t src_buffer[2][UART_TX_DMA_SIZE] __attribute__((section(".tcm_code")));

struct device *uart[2];
struct device *dma_ch[2];

Ring_Buffer_Type usb_rx_rb[2];
Ring_Buffer_Type uart_rx_rb[2];

void uart_irq_callback(struct device *dev, void *args, uint32_t size, uint32_t state)
{
    Ring_Buffer_Type *puart_rx_rb;

    if (!strncmp(dev->name, uart[0]->name, DEVICE_NAME_MAX)) {
        puart_rx_rb = &uart_rx_rb[0];
    } else {
        puart_rx_rb = &uart_rx_rb[1];
    }

    if (state == UART_EVENT_RX_FIFO) {
        if (size && size < Ring_Buffer_Get_Empty_Length(puart_rx_rb)) {
            Ring_Buffer_Write(puart_rx_rb, (uint8_t *)args, size);
        } else {
            MSG("RF OV\r\n");
        }
    } else if (state == UART_EVENT_RTO) {
        if (size && size < Ring_Buffer_Get_Empty_Length(puart_rx_rb)) {
            Ring_Buffer_Write(puart_rx_rb, (uint8_t *)args, size);
        } else {
            MSG("RTO OV\r\n");
        }
    } else if (state == UART_RX_FER_IT) {
        MSG("RX ERR\r\n");
    }
}
void uart_init(uint8_t idx)
{
    if (idx == 0) {
        uart_register(UART0_INDEX, "uart0");
        uart[0] = device_find("uart0");

        dma_register(DMA0_CH2_INDEX, "ch2");
        dma_ch[0] = device_find("ch2");

        if (dma_ch[0]) {
            DMA_DEV(dma_ch[0])->direction = DMA_MEMORY_TO_PERIPH;
            DMA_DEV(dma_ch[0])->transfer_mode = DMA_LLI_ONCE_MODE;
            DMA_DEV(dma_ch[0])->src_req = DMA_REQUEST_NONE;
            DMA_DEV(dma_ch[0])->dst_req = DMA_REQUEST_UART0_TX;
            DMA_DEV(dma_ch[0])->src_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
            DMA_DEV(dma_ch[0])->dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
            DMA_DEV(dma_ch[0])->src_burst_size = DMA_BURST_1BYTE;
            DMA_DEV(dma_ch[0])->dst_burst_size = DMA_BURST_1BYTE;
            DMA_DEV(dma_ch[0])->src_width = DMA_TRANSFER_WIDTH_8BIT;
            DMA_DEV(dma_ch[0])->dst_width = DMA_TRANSFER_WIDTH_8BIT;
            device_open(dma_ch[0], 0);
        }
    } else {
        uart_register(UART1_INDEX, "uart1");
        uart[1] = device_find("uart1");

        dma_register(DMA0_CH3_INDEX, "ch3");
        dma_ch[1] = device_find("ch3");

        if (dma_ch[1]) {
            DMA_DEV(dma_ch[1])->direction = DMA_MEMORY_TO_PERIPH;
            DMA_DEV(dma_ch[1])->transfer_mode = DMA_LLI_ONCE_MODE;
            DMA_DEV(dma_ch[1])->src_req = DMA_REQUEST_NONE;
            DMA_DEV(dma_ch[1])->dst_req = DMA_REQUEST_UART1_TX;
            DMA_DEV(dma_ch[1])->src_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
            DMA_DEV(dma_ch[1])->dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
            DMA_DEV(dma_ch[1])->src_burst_size = DMA_BURST_1BYTE;
            DMA_DEV(dma_ch[1])->dst_burst_size = DMA_BURST_1BYTE;
            DMA_DEV(dma_ch[1])->src_width = DMA_TRANSFER_WIDTH_8BIT;
            DMA_DEV(dma_ch[1])->dst_width = DMA_TRANSFER_WIDTH_8BIT;
            device_open(dma_ch[1], 0);
        }
    }
}

void uart_config(uint8_t idx, uint32_t baudrate, uart_databits_t databits, uart_parity_t parity, uart_stopbits_t stopbits)
{
    device_close(uart[idx]);
    UART_DEV(uart[idx])->baudrate = baudrate;
    UART_DEV(uart[idx])->stopbits = stopbits;
    UART_DEV(uart[idx])->parity = parity;
    UART_DEV(uart[idx])->databits = (databits - 5);
    device_open(uart[idx], DEVICE_OFLAG_DMA_TX | DEVICE_OFLAG_INT_RX);
    device_set_callback(uart[idx], uart_irq_callback);
    device_control(uart[idx], DEVICE_CTRL_SET_INT, (void *)(UART_RX_FIFO_IT | UART_RTO_IT));
    Ring_Buffer_Reset(&usb_rx_rb[idx]);
    Ring_Buffer_Reset(&uart_rx_rb[idx]);
}

void uart_set_dtr_rts(uint8_t dtr, uint8_t rts)
{
}

void uart_dtr_init(void)
{
}
void uart_rts_init(void)
{
}
void uart_dtr_deinit(void)
{
}
void uart_rts_deinit(void)
{
}
void dtr_pin_set(uint8_t status)
{
}
void rts_pin_set(uint8_t status)
{
}
void ringbuffer_lock()
{
    cpu_global_irq_disable();
}
void ringbuffer_unlock()
{
    cpu_global_irq_enable();
}

void uart_ringbuffer_init(void)
{
    /* init mem for ring_buffer */
    memset(usb_rx_mem[0], 0, USB_OUT_RINGBUFFER_SIZE);
    memset(usb_rx_mem[1], 0, USB_OUT_RINGBUFFER_SIZE);
    memset(uart_rx_mem[0], 0, UART_RX_RINGBUFFER_SIZE);
    memset(uart_rx_mem[1], 0, UART_RX_RINGBUFFER_SIZE);

    /* init ring_buffer */
    Ring_Buffer_Init(&usb_rx_rb[0], usb_rx_mem[0], USB_OUT_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
    Ring_Buffer_Init(&uart_rx_rb[0], uart_rx_mem[0], UART_RX_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);

    Ring_Buffer_Init(&usb_rx_rb[1], usb_rx_mem[1], USB_OUT_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
    Ring_Buffer_Init(&uart_rx_rb[1], uart_rx_mem[1], UART_RX_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
}

static dma_control_data_t uart_dma_ctrl_cfg = {
    .bits.fix_cnt = 0,
    .bits.dst_min_mode = 0,
    .bits.dst_add_mode = 0,
    .bits.SI = 1,
    .bits.DI = 0,
    .bits.SWidth = DMA_TRANSFER_WIDTH_8BIT,
    .bits.DWidth = DMA_TRANSFER_WIDTH_8BIT,
    .bits.SBSize = 0,
    .bits.DBSize = 0,
    .bits.I = 0,
    .bits.TransferSize = 4095
};
static dma_lli_ctrl_t uart_lli_list[2] = {
    {
        .src_addr = (uint32_t)src_buffer[0],
        .dst_addr = DMA_ADDR_UART0_TDR,
        .nextlli = 0
    },
    {
        .src_addr = (uint32_t)src_buffer[1],
        .dst_addr = DMA_ADDR_UART1_TDR,
        .nextlli = 0
    }
};

void uart_send_from_ringbuffer(uint8_t idx, Ring_Buffer_Type *rb)
{
    if (Ring_Buffer_Get_Length(rb)) {
        if (!dma_channel_check_busy(dma_ch[idx])) {
            uint32_t avalibleCnt = Ring_Buffer_Read(rb, src_buffer[idx], UART_TX_DMA_SIZE);

            if (avalibleCnt) {
                dma_channel_stop(dma_ch[idx]);
                uart_dma_ctrl_cfg.bits.TransferSize = avalibleCnt;
                memcpy(&uart_lli_list[idx].cfg, &uart_dma_ctrl_cfg, sizeof(dma_control_data_t));
                dma_channel_update(dma_ch[idx], (void *)((uint32_t)&uart_lli_list[idx]));
                dma_channel_start(dma_ch[idx]);
            }
        }
    }
}

#endif