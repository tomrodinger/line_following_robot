#ifndef BLE_APP_H
#define BLE_APP_H

void ble_app_init(void);
void ble_app_send(uint8_t *data, uint16_t len);
bool ble_app_is_connected(void);
bool ble_app_is_received(void);

#endif