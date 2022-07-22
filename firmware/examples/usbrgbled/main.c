/**
 * @file main.c
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
#include <math.h>
#include "bflb_platform.h"
#include "hal_usb.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "uart_interface.h"
#include "bl702_ef_ctrl.h"
#include "bl702_glb.h"
#include "hal_gpio.h"
#include "hal_pwm.h"

#define LED_R_PWM_CHAN          PWM_CH1_INDEX
#define LED_G_PWM_CHAN          PWM_CH0_INDEX
#define LED_B_PWM_CHAN          PWM_CH2_INDEX

#define CDC_IN_EP0  0x82
#define CDC_OUT_EP0 0x01
#define CDC_INT_EP0 0x83

#define USBD_VID           0x4000
#define USBD_PID           0x0789
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + 8 + 9 + 7)

static void usbd_cdc_acm_bulk_out(uint8_t ep);

uint8_t cdc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x02, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /* Interface Associate */
    0x08,                                                  /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION,             /* bDescriptorType */
    0,                                                     /* bFirstInterface */
    0x01,                                                  /* bInterfaceCount */
    USB_DEVICE_CLASS_VEND_SPECIFIC,                        /* bFunctionClass */
    0x00,                                                  /* bFunctionSubClass */
    0xFF,                                                  /* bFunctionProtocol */
    0x00,                                                  /* iFunction */
    0x09,                                                  /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE,                         /* bDescriptorType */
    (uint8_t)(0),                                          /* bInterfaceNumber */
    0x00,                                                  /* bAlternateSetting */
    0x01,                                                  /* bNumEndpoints */
    0xFF,                                                  /* bInterfaceClass */
    0x00,                                                  /* bInterfaceSubClass */
    0x00,                                                  /* bInterfaceProtocol */
    0x00,                                                  /* iInterface */
    0x07,                                                  /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                          /* bDescriptorType */
    CDC_OUT_EP0,                                           /* bEndpointAddress */
    0x02,                                                  /* bmAttributes */
    0x40, 0x00,                                            /* wMaxPacketSize */
    0x00,                                                   /* bInterval */
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'B', 0x00,                  /* wcChar0 */
    'o', 0x00,                  /* wcChar1 */
    'u', 0x00,                  /* wcChar2 */
    'f', 0x00,                  /* wcChar3 */
    'f', 0x00,                  /* wcChar4 */
    'a', 0x00,                  /* wcChar5 */
    'l', 0x00,                  /* wcChar6 */
    'o', 0x00,                  /* wcChar7 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x20,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'B', 0x00,                  /* wcChar0 */
    'o', 0x00,                  /* wcChar1 */
    'u', 0x00,                  /* wcChar2 */
    'f', 0x00,                  /* wcChar3 */
    'f', 0x00,                  /* wcChar4 */
    'a', 0x00,                  /* wcChar5 */
    'l', 0x00,                  /* wcChar6 */
    'o', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    'R', 0x00,                  /* wcChar9 */
    'G', 0x00,                  /* wcChar10 */
    'B', 0x00,                  /* wcChar11 */
    ' ', 0x00,                  /* wcChar13 */
    ' ', 0x00,                  /* wcChar14 */
    ' ', 0x00,                  /* wcChar15 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x30,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'R', 0x00,                  /* wcChar0 */
    'V', 0x00,                  /* wcChar1 */
    '-', 0x00,                  /* wcChar2 */
    'B', 0x00,                  /* wcChar3 */
    'L', 0x00,                  /* wcChar4 */
    '7', 0x00,                  /* wcChar5 */
    '0', 0x00,                  /* wcChar6 */
    '2', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    'U', 0x00,                  /* wcChar9 */
    'S', 0x00,                  /* wcChar10 */
    'B', 0x00,                  /* wcChar11 */
    ' ', 0x00,                  /* wcChar12 */
    'R', 0x00,                  /* wcChar13 */
    'G', 0x00,                  /* wcChar14 */
    'B', 0x00,                  /* wcChar15 */
    ' ', 0x00,                  /* wcChar16 */
    'L', 0x00,                  /* wcChar17 */
    'E', 0x00,                  /* wcChar18 */
    'D', 0x00,                  /* wcChar19 */
    ' ', 0x00,                  /* wcChar20 */
    ' ', 0x00,                  /* wcChar21 */
    ' ', 0x00,                  /* wcChar22 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

usbd_endpoint_t cdc_out_ep = {
    .ep_addr = CDC_OUT_EP0,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct device *usb_fs;

usbd_class_t cdc_class;
usbd_interface_t cdc_cmd_intf;
usbd_interface_t cdc_data_intf;

extern struct device *usb_dc_init(void);

static uint8_t gamma_correct(uint8_t value)
{
    return (uint8_t)(pow((float)value / 255.0f, 2.8f) * 255.0f + 0.5f);
}

static void create_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    pwm_dutycycle_config_t pwm_cfg = {
        0,
        32
    };
    struct device *led_r;
    struct device *led_g;
    struct device *led_b;

    led_r = device_find("led_r");
    led_g = device_find("led_g");
    led_b = device_find("led_b");

    pwm_cfg.threshold_high = gamma_correct(red);
    device_control(led_r, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);

    pwm_cfg.threshold_high = gamma_correct(green);
    device_control(led_g, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);

    pwm_cfg.threshold_high = gamma_correct(blue);
    device_control(led_b, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
}

static void usbd_cdc_acm_bulk_out(uint8_t ep)
{
    uint32_t actual_read_length = 0;
    uint8_t out_buffer[64];

    if (usbd_ep_read(ep, out_buffer, 64, &actual_read_length) < 0) {
        usbd_ep_set_stall(ep);
        return;
    }

    if (actual_read_length == 3) {
        create_rgb(out_buffer[0], out_buffer[1], out_buffer[2]);
    }

    usbd_ep_read(ep, NULL, 0, NULL);
}

int main(void)
{
    
    struct device *led_r;
    struct device *led_g;
    struct device *led_b;

    bflb_platform_print_set(1);
    bflb_platform_init(0);

    pwm_register(LED_R_PWM_CHAN, "led_r");
    pwm_register(LED_G_PWM_CHAN, "led_g");
    pwm_register(LED_B_PWM_CHAN, "led_b");

    led_r = device_find("led_r");
    led_g = device_find("led_g");
    led_b = device_find("led_b");

    PWM_DEV(led_r)->period = 255;
    PWM_DEV(led_r)->threshold_low = 0;
    PWM_DEV(led_r)->threshold_high = 255;
    device_open(led_r, DEVICE_OFLAG_STREAM_TX);
    pwm_channel_start(led_r);

    PWM_DEV(led_g)->period = 255;
    PWM_DEV(led_g)->threshold_low = 0;
    PWM_DEV(led_g)->threshold_high = 255;
    device_open(led_g, DEVICE_OFLAG_STREAM_TX);
    pwm_channel_start(led_g);

    PWM_DEV(led_b)->period = 255;
    PWM_DEV(led_b)->threshold_low = 0;
    PWM_DEV(led_b)->threshold_high = 255;
    device_open(led_b, DEVICE_OFLAG_STREAM_TX);
    pwm_channel_start(led_b);

    create_rgb(0, 0, 0);

    usbd_desc_register(cdc_descriptor);

    usbd_cdc_add_acm_interface_idx(&cdc_class, &cdc_cmd_intf, 0);
    usbd_cdc_add_acm_interface_idx(&cdc_class, &cdc_data_intf, 0);
    usbd_interface_add_endpoint(&cdc_data_intf, &cdc_out_ep);

    usb_fs = usb_dc_init();

    if (usb_fs) {
        device_control(usb_fs, DEVICE_CTRL_SET_INT, (void *)(USB_EP1_DATA_OUT_IT));
    }

    while (!usb_device_is_configured()) {
    }

    while(1);
}
