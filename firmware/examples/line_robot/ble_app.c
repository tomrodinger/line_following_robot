#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "drv_device.h"
#include "bflb_platform.h"
#include "bl702_glb.h"
#include "hal_common.h"

#include "bluetooth.h"
#include "conn.h"
#include "gatt.h"
#include "hci_core.h"
#include "hci_driver.h"
#include "ble_lib_api.h"
#include "bl702_sec_eng.h"
#include "ring_buffer.h"
#include "gatt.h"
#include "motor.h"

#define TO_BLE_INTERVAL(x)  ((x) * 0.625)

static struct bt_conn *ble_bl_conn = NULL;
SemaphoreHandle_t tx_sem;
static struct bt_gatt_exchange_params exchg_mtu;
static bool is_ble_app_recv = false;

#define MAGIC_CODE  "BL702BOOT"

static int ble_app_recv(struct bt_conn *conn,
              const struct bt_gatt_attr *attr, const void *buf,
              u16_t len, u16_t offset, u8_t flags)
{
    is_ble_app_recv = true;

    if ((len == sizeof(MAGIC_CODE) - 1) && 
            (!strncmp(buf, MAGIC_CODE, sizeof(MAGIC_CODE) - 1))) {
        motor_run(STOP, 0);
        __disable_irq();
        GLB_SW_POR_Reset();
        while (1) {
            /*empty dead loop*/
        }
    }

    return 0;
}

static void ble_app_cfg_changed(const struct bt_gatt_attr *attr, u16_t vblfue)
{

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

	if (err) {

	} else {
        ble_bl_conn = conn;
        bt_conn_le_param_update(conn, &param);

        if (!bt_le_set_data_len(ble_bl_conn, tx_octets, tx_time)) {
            exchg_mtu.func = ble_tx_mtu_size;
            //exchange mtu size after connected.
            bt_gatt_exchange_mtu(ble_bl_conn, &exchg_mtu);
        }
        is_ble_app_recv = false;
	}
}

static void bl_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ble_bl_conn = NULL;
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	// BT_DATA(BT_DATA_NAME_COMPLETE, "bl702_robot", sizeof("bl702_robot")),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, "RV_702", 6),
    
};
static struct bt_gatt_attr blattrs[]= {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0xFFF0)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00070001, 0x0745, 0x4650, 0x8d93, 0xdf59be2fc10a)),
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            NULL,
                            NULL,
                            NULL),

    BT_GATT_CCC(ble_app_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00070002, 0x0745, 0x4650, 0x8d93, 0xdf59be2fc10a)),
                            BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                            BT_GATT_PERM_WRITE,
                            NULL,
                            ble_app_recv,
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
    struct bt_le_adv_param adv_param = {
        .options = BT_LE_ADV_OPT_CONNECTABLE | 
                    BT_LE_ADV_OPT_USE_NAME,
        .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
        .interval_max = BT_GAP_ADV_SLOW_INT_MAX
    };
    
    if (!err) {
        bt_get_local_public_address(&adv_addr);
        sprintf(str, "robot_bl702_%02X%02X", adv_addr.a.val[0], adv_addr.a.val[1]);
        
        bt_set_name(str);

        bt_conn_cb_register(&conn_callbacks);
        bt_gatt_service_register(&ble_bl_server);
        bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    }
}

void ble_app_init(void)
{
    tx_sem = xSemaphoreCreateBinary();

    GLB_Set_EM_Sel(GLB_EM_8KB);
    ble_controller_init(configMAX_PRIORITIES - 1);
    // Initialize BLE Host stack
    hci_driver_init();

    bt_enable(bt_enable_cb);
}

static void notify_cb(struct bt_conn *conn, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( tx_sem, &xHigherPriorityTaskWoken );
}

void ble_app_send(uint8_t *data, uint16_t len)
{
    struct bt_gatt_notify_params params;
    uint16_t offset = 0;
    uint16_t sent_length;
    uint8_t uuid_idx = 1;
    uint16_t mtu = bt_gatt_get_mtu(ble_bl_conn) - 10;

    if (len) {
        memset(&params, 0, sizeof(params));

        while (offset < len) {
            if ((len - offset) >= mtu) {
                sent_length = mtu;
            } else {
                sent_length = len - offset;
            }

            params.uuid = blattrs[uuid_idx].uuid;
            params.attr = &blattrs[uuid_idx];
            params.data = &data[offset];
            params.len = sent_length;
            params.func = notify_cb;

            bt_gatt_notify_cb(ble_bl_conn, &params);

            xSemaphoreTake( tx_sem, portMAX_DELAY);

            offset += sent_length;
        }
    }
}

bool ble_app_is_connected(void)
{
    return (ble_bl_conn != NULL);
}

bool ble_app_is_received(void)
{
    if (is_ble_app_recv) {
        is_ble_app_recv = false;
        return true;
    }

    return false;
}