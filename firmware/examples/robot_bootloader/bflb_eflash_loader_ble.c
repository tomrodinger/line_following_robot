/**
  ******************************************************************************
  * @file    blsp_eflash_loader_ble.c
  * @version V1.2
  * @date
  * @brief   This file is the peripheral case header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Bouffalo Lab</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Bouffalo Lab nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "bflb_eflash_loader.h"
#include "bflb_platform.h"
#include "blsp_port.h"
#include "blsp_common.h"
#include "partition.h"
#include "hal_uart.h"
#include "drv_device.h"
#include "hal_boot2.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "bluetooth.h"
#include "conn.h"
#include "gatt.h"
#include "hci_core.h"
#include "hci_driver.h"
#include "ble_lib_api.h"
#include "bl702_sec_eng.h"
#include "hal_wdt.h"

static struct bt_conn *ble_bl_conn = NULL;
static struct bt_gatt_exchange_params exchg_mtu;
static SemaphoreHandle_t rx_sem;
static SemaphoreHandle_t tx_sem;
static bool is_indicate_enabled = false;

void bflb_eflash_loader_ble_if_enable_int(void)
{

}

void bflb_eflash_loader_ble_if_send(uint8_t *data, uint32_t len)
{

}

int32_t bflb_eflash_loader_ble_if_wait_tx_idle(uint32_t timeout)
{
    return 0;
}

static int ble_blf_recv(struct bt_conn *conn,
              const struct bt_gatt_attr *attr, const void *buf,
              u16_t len, u16_t offset, u8_t flags)
{
    uint8_t *ble_buf = (uint8_t *)buf;
    uint8_t *rcv_buf = g_eflash_loader_readbuf[0];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t pkg_length;

    /*If prepare write, it will return 0 */
    if (flags == BT_GATT_WRITE_FLAG_PREPARE) {
        return 0;
    }

    if ((len + g_rx_buf_len) >= BFLB_EFLASH_LOADER_BLE_READBUF_SIZE) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    arch_memcpy(&rcv_buf[g_rx_buf_len], ble_buf, len);
    g_rx_buf_len += len;

    if (g_rx_buf_len < 2) {
        g_rx_buf_len = 0;
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    pkg_length = rcv_buf[0] | (rcv_buf[1] << 8);

    if ((g_rx_buf_len - 2) == pkg_length) {
        g_rx_buf_len = g_rx_buf_len - 2;
        xSemaphoreGiveFromISR( rx_sem, &xHigherPriorityTaskWoken );
    } else if ((g_rx_buf_len - 2) > pkg_length) {
        g_rx_buf_len = 0;
        xSemaphoreGiveFromISR( rx_sem, &xHigherPriorityTaskWoken );
    }

    return len;
}

static void ble_cfg_changed(const struct bt_gatt_attr *attr, u16_t vblfue)
{
    if(vblfue == BT_GATT_CCC_INDICATE) {
        
        MSG("enable notify.\n");
        is_indicate_enabled = true;
  
    } else {
        MSG("disable notify.\n");
        is_indicate_enabled = false;
    }
}

static void ble_tx_mtu_size(struct bt_conn *conn, u8_t err,
                               struct bt_gatt_exchange_params *params)
{

}

static void bl_connected(struct bt_conn *conn, uint8_t err)
{
    int tx_octets = 0x00fb;
    int tx_time = 0x0848;
    struct bt_le_conn_param param;

    param.interval_max=0x28;
    param.interval_min=0x18;
    param.latency=0;
    param.timeout=400;
    g_rx_buf_len = 0;

     MSG("%s err %d\n", __FUNCTION__, err);

	if (err) {

	} else {
        ble_bl_conn = conn;
        bt_conn_le_param_update(conn, &param);

        if (!bt_le_set_data_len(ble_bl_conn, tx_octets, tx_time)) {
            exchg_mtu.func = ble_tx_mtu_size;
            //exchange mtu size after connected.
            bt_gatt_exchange_mtu(ble_bl_conn, &exchg_mtu);
        }

        if (!g_eflash_loader_readbuf[0])
        {
            g_eflash_loader_readbuf[0] = pvPortMalloc(BFLB_EFLASH_LOADER_BLE_READBUF_SIZE);
            arch_memset(g_eflash_loader_readbuf[0], 0, BFLB_EFLASH_LOADER_BLE_READBUF_SIZE);
        } else {
            vPortFree(g_eflash_loader_readbuf[0]);
            vPortFree(g_eflash_loader_readbuf[1]);

            g_eflash_loader_readbuf[0] = pvPortMalloc(BFLB_EFLASH_LOADER_BLE_READBUF_SIZE);
            arch_memset(g_eflash_loader_readbuf[0], 0, BFLB_EFLASH_LOADER_BLE_READBUF_SIZE);
        }

        xSemaphoreTake(rx_sem, 0);
        
        bflb_eflash_loader_if_set(BFLB_EFLASH_LOADER_IF_BLE);
	}
}

static void bl_disconnected(struct bt_conn *conn, uint8_t reason)
{
    MSG("%s reason %d\n", __FUNCTION__, reason);
    if (!g_eflash_loader_readbuf[0])
    {
        vPortFree(g_eflash_loader_readbuf[0]);
    }

    ble_bl_conn = NULL;
    is_indicate_enabled = false;
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	// BT_DATA(BT_DATA_NAME_COMPLETE, "bl702_robot", sizeof("bl702_robot")),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, "BL_702", 6),
    
};
static struct bt_gatt_attr blattrs[]= {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0xFFF0)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00070001, 0x0745, 0x4650, 0x8d93, 0xdf59be2fc10a)),
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
                            BT_GATT_PERM_READ,
                            NULL,
                            NULL,
                            NULL),

    BT_GATT_CCC(ble_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00070002, 0x0745, 0x4650, 0x8d93, 0xdf59be2fc10a)),
                            BT_GATT_CHRC_WRITE,
                            BT_GATT_PERM_WRITE,
                            NULL,
                            ble_blf_recv,
                            NULL)
};

static struct bt_gatt_service ble_bl_server = BT_GATT_SERVICE(blattrs);

static struct bt_conn_cb conn_callbacks = {
	.connected = bl_connected,
	.disconnected = bl_disconnected,
};

void bt_enable_cb(int err)
{
    bt_addr_le_t adv_addr;
    char str[100] = {0};
    
    if (!err) {
        bt_get_local_public_address(&adv_addr);
        sprintf(str, "robot_bl702_%02X%02X", adv_addr.a.val[0], adv_addr.a.val[1]);
        
        bt_set_name(str);

        bt_conn_cb_register(&conn_callbacks);
        bt_gatt_service_register(&ble_bl_server);
        bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    }
}

int32_t bflb_eflash_loader_ble_init()
{
    rx_sem = xSemaphoreCreateBinary();
    tx_sem = xSemaphoreCreateBinary();

    GLB_Set_EM_Sel(GLB_EM_8KB);
    ble_controller_init(configMAX_PRIORITIES - 1);
    // Initialize BLE Host stack
    hci_driver_init();

    bt_enable(bt_enable_cb);

    return BFLB_EFLASH_LOADER_SUCCESS;
}

int32_t bflb_eflash_loader_ble_handshake_poll(uint32_t timeout)
{
    if (ble_bl_conn) {
        return 0;
    }
    return -1;
}

uint32_t *bflb_eflash_loader_ble_recv(uint32_t *recv_len, uint32_t maxlen, uint32_t timeout)
{
    uint32_t *rcv_buf = NULL;
    uint32_t wait_new_conn_timeout = timeout;
    uint8_t cmd;
    struct device *wdg;

    wdg = device_find("wdg_rst");

    if (ble_bl_conn == NULL) {
        while (wait_new_conn_timeout) {
            vTaskDelay(pdMS_TO_TICKS(20));
            wait_new_conn_timeout -= 20;

            if (wdg) {
                device_control(wdg, DEVICE_CTRL_RST_WDT_COUNTER, NULL);
            }

            if (ble_bl_conn) {
                break;
            }
        }
    }

    if (ble_bl_conn) {
        while (timeout) {
            timeout = timeout - 20;

            if (wdg) {
                device_control(wdg, DEVICE_CTRL_RST_WDT_COUNTER, NULL);
            }

            if (xSemaphoreTake(rx_sem, pdMS_TO_TICKS(20)) == pdTRUE) {
                break;
            }
        }

        if (timeout) {
            if (g_rx_buf_len) {
                *recv_len = g_rx_buf_len;
                g_rx_buf_len = 0;
                rcv_buf = (uint32_t *)&g_eflash_loader_readbuf[0][2];
                cmd = *((uint8_t *)&g_eflash_loader_readbuf[0][2]);

                if ((cmd == BFLB_EFLASH_LOADER_CMD_FLASH_ERASE) ||
                        (cmd == BFLB_EFLASH_LOADER_CMD_RESET)) {
                    timeout = 200;
                    
                    while (timeout) {
                        timeout = timeout - 20;

                        if (wdg) {
                            device_control(wdg, DEVICE_CTRL_RST_WDT_COUNTER, NULL);
                        }

                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                }
            } else {
                bflb_eflash_loader_ble_send("NOK", sizeof("NOK"));
            }
        } else {
            g_rx_buf_len = 0;
        }
    }

    return rcv_buf;
}

static void indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        u8_t err)
{
    if (err != 0U) {
        // vOutputString("Indication fail");
        MSG("Indication fail %d\r\n", err);
    } else {
        MSG("Indication success\r\n");
        // vOutputString("Indication success");
    }

    xSemaphoreGive(tx_sem);
}

int32_t bflb_eflash_loader_ble_send(uint32_t *data, uint32_t len)
{
    struct bt_gatt_indicate_params params;
    struct device *wdg;
    uint16_t timeout = 5000;

    wdg = device_find("wdg_rst");

    MSG("send len %u\r\n", len);

    xSemaphoreTake(tx_sem, 0);

    if ((ble_bl_conn) && (is_indicate_enabled)) {

        memset(&params, 0, sizeof(struct bt_gatt_indicate_params));
        params.attr = &blattrs[2];
        params.data = data;
        params.len = len;
        params.func = indicate_cb;

        if (!bt_gatt_indicate(ble_bl_conn, &params)) {
            while (timeout) {
                timeout = timeout - 20;

                if (wdg) {
                    device_control(wdg, DEVICE_CTRL_RST_WDT_COUNTER, NULL);
                }

                if (xSemaphoreTake(tx_sem, pdMS_TO_TICKS(20)) == pdTRUE) {
                    break;
                }
            }

            if (timeout) {
                return 0;
            }
        }
    }
    
    return -1;
}

int32_t bflb_eflash_loader_ble_wait_tx_idle(uint32_t timeout)
{
    return BFLB_EFLASH_LOADER_SUCCESS;
}

int32_t bflb_eflash_loader_ble_change_rate(uint32_t oldval, uint32_t newval)
{
    return BFLB_EFLASH_LOADER_SUCCESS;
}

int32_t bflb_eflash_loader_ble_deinit()
{
    return BFLB_EFLASH_LOADER_SUCCESS;
}

void bflb_eflash_loader_ble_stop(void)
{
    if (ble_bl_conn) {
        bt_conn_disconnect(ble_bl_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
    bt_le_adv_stop();
    bt_disable();
    // ble_controller_deinit();
}
