from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.logger import Logger
from kivy.uix.floatlayout import FloatLayout
from kivy.factory import Factory
from kivy.properties import ObjectProperty
from kivy.uix.popup import Popup
from kivy.utils import platform
from able import BluetoothDispatcher, GATT_SUCCESS
from dataclasses import dataclass, field
from typing import List
from functools import partial
from kivy.clock import Clock

from able.android.dispatcher import (
    BluetoothDispatcher
)

from able.scan_settings import (
    ScanSettings,
    ScanSettingsBuilder
)
from able.filters import *
import time
import configparser
import hashlib
import binascii
import threading
from queue import Queue
import os

BFLB_EFLASH_LOADER_CMD_RESET=b'\x21'
BFLB_EFLASH_LOADER_CMD_FLASH_ERASE=b'\x30'
BFLB_EFLASH_LOADER_CMD_FLASH_WRITE=b'\x31'
BFLB_EFLASH_LOADER_CMD_FLASH_READ=b'\x32'
BFLB_EFLASH_LOADER_CMD_FLASH_BOOT=b'\x33'
BFLB_EFLASH_LOADER_CMD_FLASH_WRITE_CHECK=b'\x3A'
BFLB_EFLASH_LOADER_CMD_FLASH_SET_PARA=b'\x3B'
BFLB_EFLASH_LOADER_CMD_FLASH_CHIPERASE=b'\x3C'
BFLB_EFLASH_LOADER_CMD_FLASH_READSHA=b'\x3D'
BFLB_EFLASH_LOADER_CMD_FLASH_XIP_READSHA=b'\x3E'

FLASH_START_ADDRESS=0x2F000
FLASH_TOTAL_SIZE=0xca000
FLASH_END_ADDRESS=FLASH_START_ADDRESS + FLASH_TOTAL_SIZE
FLASH_PAGE_SIZE=4096
HANDSHAKE_CMD=b'\x55\x55\x55\x55'
FLASH_OFFSET=0x2000
BOOT_ENTRY=0x0000
MAGIC_CODE="BL702BOOT"

BLE_READ_CHARACTERISTIC_UUID = "00070001-0745-4650-8d93-df59be2fc10a"
BLE_WRITE_CHARACTERISTIC_UUID = "00070002-0745-4650-8d93-df59be2fc10a"

class LoadDialog(FloatLayout):
    load = ObjectProperty(None)
    cancel = ObjectProperty(None)

class BLE(BluetoothDispatcher):
    devices = []
    is_have_service = False
    is_reconnect = False
    is_write_done = False
    device = None
    rx_queue = Queue(maxsize = 1)
        
    def on_device(self, device, rssi, advertisement):
        if device.getName():
            Logger.info("Found device {}".format(device.getName()))
        if device.getName() and ("robot_bl702" in device.getName() or "bl702_robot" in device.getName()):
            self.device = device
            self.stop_scan()
            self.connect_gatt(device)

    def on_connection_state_change(self, status, state):
        Logger.info("On connect status {} state {}".format(status, state))
        if status == GATT_SUCCESS and state:
            self.discover_services()
        else:
            self.is_have_service = False
            if self.is_reconnect:
                self.close_gatt()
                self.connect_gatt(self.device)

    def on_services(self, status, services):
        if status == GATT_SUCCESS:
            # save discovered services object
            self.services = services
            self.is_have_service = True

    def on_characteristic_read(self, characteristic, status):
        Logger.info("on_characteristic_read: status=%s, characteristic=%s", status,
                    characteristic.getUuid().toString())
        if status == GATT_SUCCESS:
            Logger.info("Characteristic read: succeed")

    def on_characteristic_write(self, characteristic, status):
        Logger.info("Write {} with status {}".format(characteristic, status))
        if status == GATT_SUCCESS:
            self.is_write_done = True

    def on_characteristic_changed(self, characteristic):
        Logger.info("On characteristic changed {}".format(characteristic))
        self.rx_queue.put(characteristic.getStringValue(0))

    def clean_queue(self):
        if not self.rx_queue.empty():
            self.rx_queue.get()

    def find_device(self):
        self.is_have_service = False
        self.is_reconnect = True
        self.start_scan()

        while self.is_have_service == False:
            time.sleep(0.1)

def add_boot_header(data, config):
    # BOOT HEADER
    header = int(config.get('BOOTHEADER_CFG', 'magic_code'), 16).to_bytes(4, "little")
    header = header + int(config.get('BOOTHEADER_CFG', 'revision'), 16).to_bytes(4, "little")
    # Flash configuration
    flash_cfg = int(config.get('BOOTHEADER_CFG', 'io_mode'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'cont_read_support').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'sfctrl_clk_delay').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'sfctrl_clk_invert'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'reset_en_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'reset_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'exit_contread_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'exit_contread_cmd_size').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'jedecid_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'jedecid_cmd_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'qpi_jedecid_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qpi_jedecid_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'sector_size').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'mfg_id'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'page_size').to_bytes(2, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'chip_erase_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'sector_erase_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'blk32k_erase_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'blk64k_erase_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'write_enable_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'page_prog_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'qpage_prog_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qual_page_prog_addr_mode').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'fast_read_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'fast_read_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'qpi_fast_read_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qpi_fast_read_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'fast_read_do_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'fast_read_do_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'fast_read_dio_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'fast_read_dio_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'fast_read_qo_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'fast_read_qo_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'fast_read_qio_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'fast_read_qio_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'qpi_fast_read_qio_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qpi_fast_read_qio_dmy_clk').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'qpi_page_prog_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'write_vreg_enable_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'wel_reg_index').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qe_reg_index').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'busy_reg_index').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'wel_bit_pos').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qe_bit_pos').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'busy_bit_pos').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'wel_reg_write_len').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'wel_reg_read_len').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qe_reg_write_len').to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'qe_reg_read_len').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'release_power_down'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'busy_reg_read_len').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'reg_read_cmd0'), 16).to_bytes(2, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'reg_read_cmd1'), 16).to_bytes(2, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'reg_write_cmd0'), 16).to_bytes(2, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'reg_write_cmd1'), 16).to_bytes(2, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'enter_qpi_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'exit_qpi_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'cont_read_code'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'cont_read_exit_code'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'burst_wrap_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'burst_wrap_dmy_clk'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'burst_wrap_data_mode').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'burst_wrap_code'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'de_burst_wrap_cmd'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'de_burst_wrap_cmd_dmy_clk'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'de_burst_wrap_code_mode').to_bytes(1, "little")
    flash_cfg = flash_cfg + int(config.get('BOOTHEADER_CFG', 'de_burst_wrap_code'), 16).to_bytes(1, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'sector_erase_time').to_bytes(2, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'blk32k_erase_time').to_bytes(2, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'blk64k_erase_time').to_bytes(2, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'page_prog_time').to_bytes(2, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'chip_erase_time').to_bytes(2, "little")
    flash_cfg = flash_cfg + config.getint('BOOTHEADER_CFG', 'power_down_delay').to_bytes(2, "little")
    flash_cfg = int(config.get('BOOTHEADER_CFG', 'flashcfg_magic_code'), 16).to_bytes(4, "little") + flash_cfg + binascii.crc32(flash_cfg).to_bytes(4, "little")
    # Clock configuration
    clock_cfg = config.getint('BOOTHEADER_CFG', 'xtal_type').to_bytes(1, "little")
    clock_cfg = clock_cfg + config.getint('BOOTHEADER_CFG', 'pll_clk').to_bytes(1, "little")
    clock_cfg = clock_cfg + config.getint('BOOTHEADER_CFG', 'hclk_div').to_bytes(1, "little")
    clock_cfg = clock_cfg + config.getint('BOOTHEADER_CFG', 'bclk_div').to_bytes(1, "little")
    clock_cfg = clock_cfg + config.getint('BOOTHEADER_CFG', 'flash_clk_type').to_bytes(2, "little")
    clock_cfg = clock_cfg + config.getint('BOOTHEADER_CFG', 'flash_clk_div').to_bytes(2, "little")
    clock_cfg = int(config.get('BOOTHEADER_CFG', 'clkcfg_magic_code'), 16).to_bytes(4, "little") + clock_cfg + binascii.crc32(clock_cfg).to_bytes(4, "little")
    # Boot configuration
    boot_cfg = (config.getint('BOOTHEADER_CFG', 'sign') | (config.getint('BOOTHEADER_CFG', 'encrypt_type') << 2) |
                (config.getint('BOOTHEADER_CFG', 'key_sel') << 4) | (config.getint('BOOTHEADER_CFG', 'no_segment') << 8) |
                (config.getint('BOOTHEADER_CFG', 'cache_enable') << 9) | (config.getint('BOOTHEADER_CFG', 'notload_in_bootrom') << 10) |
                (config.getint('BOOTHEADER_CFG', 'aes_region_lock') << 11) | (int(config.get('BOOTHEADER_CFG', 'cache_way_disable'), 16) << 12) |
                (config.getint('BOOTHEADER_CFG', 'crc_ignore') << 16) | (config.getint('BOOTHEADER_CFG', 'hash_ignore') << 17) |
                (config.getint('BOOTHEADER_CFG', 'boot2_enable') << 19)).to_bytes(4, "little")
    # Image configuration
    image_cfg = len(data).to_bytes(4, "little")
    image_cfg = image_cfg + config.getint('BOOTHEADER_CFG', 'bootentry').to_bytes(4, "little")
    image_cfg = image_cfg + int(config.get('BOOTHEADER_CFG', 'img_start'), 16).to_bytes(4, "little")

    sha256 = hashlib.sha256()

    sha256.update(data)

    image_cfg = image_cfg + sha256.digest()
    image_cfg = image_cfg + int(config.get('BOOTHEADER_CFG', 'boot2_pt_table_0'), 16).to_bytes(4, "little")
    image_cfg = image_cfg + int(config.get('BOOTHEADER_CFG', 'boot2_pt_table_1'), 16).to_bytes(4, "little")

    header = header + flash_cfg + clock_cfg + boot_cfg + image_cfg
    header = header + binascii.crc32(header).to_bytes(4, "little")

    while len(header) < 0x1000:
        header = header + b'\xFF'

    return header + data

def create_payload(command, data):
    checksum = 0

    for item in data:
        checksum = checksum + item

    checksum = checksum + len(data) & 0xFF
    checksum = checksum + ((len(data) >> 8) & 0xFF)
    checksum = checksum & 0xFF

    payload = command + checksum.to_bytes(1, "little") + len(data).to_bytes(2, "little") + data

    return payload

def write_data(ble, buf):
    characteristic = ble.services.search(BLE_WRITE_CHARACTERISTIC_UUID)
    while len(buf) > 0:
        if len(buf) <= 240:
            data = int(0).to_bytes(1, "little") + buf[0 : 240]
        else:
            data = int(1).to_bytes(1, "little") + buf[0 : 240]
        
        ble.is_write_done = False
        ble.write_characteristic(characteristic, data)
        while ble.is_write_done == False:
            time.sleep(0.1)
        
        buf = buf[240: ]

def erase_flash_ble(ble, size):
    size = size + FLASH_START_ADDRESS

    data = FLASH_START_ADDRESS.to_bytes(4, "little") + size.to_bytes(4, "little")

    command = create_payload(BFLB_EFLASH_LOADER_CMD_FLASH_ERASE, data)

    write_data(ble, command)

def get_response_ble(ble, log):
    timeout = 3
    while ble.rx_queue.empty():
        timeout = timeout - 1
        if timeout == 0:
            break
        time.sleep(1)
    
    if timeout == 0:
        log.print_out("Did not receive response from BLE device\r\n")
        return -1
    else:
        response = ble.rx_queue.get()
    
    if len(response) != 2:
        log.print_out("Error: didn't receive enough bytes in the response\r\n")
        return -1

    log.print_out("Received a response: {}\r\n".format(response))

    if not("OK" in response):
        log.print_out("Error: response NACK or unknown response\r\n")
        return -1
    return 0

def program_one_page_ble(ble, data, log):
    data = int(0).to_bytes(4, "little") + data

    command = create_payload(BFLB_EFLASH_LOADER_CMD_FLASH_WRITE, data)

    # write the bytes in three shots with a time delay betwoen, otherwise there is a strange bug where bytes get dropped
    ble.clean_queue()
    write_data(ble, command)

    return (get_response_ble(ble, log))

def system_reset_command_ble(ble):
    data = int(0).to_bytes(1, "little")

    command = create_payload(BFLB_EFLASH_LOADER_CMD_RESET, data)

    write_data(ble, command)

def thread_main(log, ble, bin_path, ini_path, ui_bt):
    config = configparser.ConfigParser()
    bin_data = None

    try:
        if bin_path != "":
            with open(bin_path, "rb") as fh:
                bin_data = fh.read()
        else:
            log.print_out("Error: Please select .BIN file")
            return
        if ini_path != "":
            config.read(ini_path)
        else:
            log.print_out("Error: Please select .INI file")
            return

        # pad 0x00 until the length of the data is divisable by 16
        while len(bin_data) % 0x10 != 0:
            bin_data = bin_data + b'\x00'

        bin_data = add_boot_header(bin_data, config)

        # pad 0x00 until the length of the data is divisable by 256
        while len(bin_data) & 0xFF != 0:
            bin_data = bin_data + b'\xFF'

        if len(bin_data) > FLASH_TOTAL_SIZE:
            log.print_out("Error: the firmware is too big to fit in the flash, its size is {}".format(len(self.bin_data)))

        log.print_out("\r\nScanning...")

        ble.find_device()

        device = ble.device

        if device is None:
            log.print_out("\r\nCannot find device...")
            return
        else:
            log.print_out('\r\n' + "Found device {} with address {}".format(device.getName(), device.getAddress()))

        log.print_out("\r\nReset into bootloader...")

        characteristic = ble.services.search(BLE_WRITE_CHARACTERISTIC_UUID)
        data = MAGIC_CODE.encode()  
        ble.is_write_done = False
        ble.write_characteristic(characteristic, data)
        while ble.is_write_done == False:
            time.sleep(0.1)

        dis_timeout = 2000
        while ble.is_have_service == True:
            time.sleep(0.01)
            dis_timeout = dis_timeout - 1
            if dis_timeout == 0:
                break

        if dis_timeout != 0:
            while ble.is_have_service == False:
                time.sleep(0.01)
            log.print_out("\r\nConnected to bootloader and erase flash")
        else:
            log.print_out("\r\nRobot is already in bootloader mode")
        
        ble.is_reconnect = False
        erase_flash_ble(ble, len(bin_data))
        ble.close_gatt()
        time.sleep(20)

        log.print_out("\r\nErased flash, reconnect to update firmware")
        ble.find_device()

        read_characteristic = ble.services.search(BLE_READ_CHARACTERISTIC_UUID)
        ble.enable_notifications(read_characteristic, True)

        FLASH_PAGE_SIZE = 4096
        while len(bin_data) > 0:
            log.print_out("Size left: {}\r\n".format(len(bin_data)))
            # if len(data) < FLASH_PAGE_SIZE:
            #     data = data + bytearray([0]) * (FLASH_PAGE_SIZE - len(data))
            #     print("Size left after append:", len(data))
            # assert len(data) >= FLASH_PAGE_SIZE
            ret = program_one_page_ble(ble, bin_data[0 : FLASH_PAGE_SIZE], log)
            if ret != 0:
                log.print_out("Please reset robot and try again\r\n")
                break
            bin_data = bin_data[FLASH_PAGE_SIZE:]

        if ret == 0:
            ble.is_reconnect = False
            system_reset_command_ble(ble)
            log.print_out("Successfull\r\n")
        else:
            log.print_out("Failed\r\n")
        ui_bt.disabled = False
        ble.close_gatt()

    except Exception as e:
        log.print_out("\r\n{}".format(e))

class MyLogHandler():

    def __init__(self, label):
        self.label = label

    def print_to_label(self, text, *largs):
        self.label.text = self.label.text + text

    def print_out(self, text):
        Clock.schedule_once(partial(self.print_to_label, text))
        # self.label.text = self.label.text + text

class Main(FloatLayout):
    log = None
    ble = None

    def dismiss_popup(self):
        self._popup.dismiss()

    def load_command(self):
        try:
            content = LoadDialog(load=self.load, cancel=self.dismiss_popup)
            PATH = "."
            from android.permissions import request_permissions, Permission
            request_permissions([Permission.READ_EXTERNAL_STORAGE, Permission.WRITE_EXTERNAL_STORAGE])
            # app_folder = os.path.dirname(os.path.abspath(__file__))
            # PATH = "/storage/emulated/0" #app_folder
            content.ids.filechooser.path = "/storage/emulated/0"

            self._popup = Popup(title="Load file", content=content,
                                size_hint=(0.9, 0.9))
            self._popup.open()
        except Exception as e:
            self.ids.texterror.text = "{}".format(e)

    def load(self, path, filename):
        try:
            self.ids.textpath.text = filename[0]
            self.dismiss_popup()
        except Exception as e:
            self.ids.texterror.text = "{}".format(e)

    def load_command1(self):
        try:
            content = LoadDialog(load=self.load1, cancel=self.dismiss_popup)
            PATH = "."
            from android.permissions import request_permissions, Permission
            request_permissions([Permission.READ_EXTERNAL_STORAGE, Permission.WRITE_EXTERNAL_STORAGE])
            # app_folder = os.path.dirname(os.path.abspath(__file__))
            # PATH = "/storage/emulated/0" #app_folder
            content.ids.filechooser.path = "/storage/emulated/0"

            self._popup = Popup(title="Load file", content=content,
                                size_hint=(0.9, 0.9))
            self._popup.open()
        except Exception as e:
            self.ids.texterror.text = "{}".format(e)

    def load1(self, path, filename):
        try:
            # with open(os.path.join(path, filename[0])) as stream:
            #     self.textpath.text = stream.read()
            self.ids.textpath1.text = filename[0]
            self.dismiss_popup()
        except Exception as e:
            self.ids.texterror.text = "{}".format(e)

    def updateLog(self, text, *args):
        self.ids.texterror.text = self.ids.texterror.text + text

    def process_command(self):
        self.ids.program_bt.disabled = True
        self.ble = BLE()
        self.ids.texterror.text = ""
        if self.log is None:
            self.log = MyLogHandler(self.ids.texterror)
        threading.Thread(target = thread_main, args = (self.log, self.ble, self.ids.textpath.text, self.ids.textpath1.text, self.ids.program_bt, )).start()

class MainApp(App):
    pass

Factory.register('Main', cls=Main)
Factory.register('LoadDialog', cls=LoadDialog)

if __name__ == '__main__':
    MainApp().run()
