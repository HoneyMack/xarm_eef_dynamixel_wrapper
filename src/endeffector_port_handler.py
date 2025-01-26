# Made by: Takuya Okubo

import warnings
from dynamixel_sdk import PortHandler
from dynamixel_sdk.robotis_def import *
from xarm.wrapper import XArmAPI

LATENCY_TIMER = 16
DEFAULT_BAUDRATE = 1000000


class EndEffectorPortHandler(PortHandler):
    def __init__(self, xArmAPI: XArmAPI):
        self.is_open = False
        self.baudrate = DEFAULT_BAUDRATE
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0
        self.received_packet = []

        self.is_using = False
        self.xapi = xArmAPI

    def closePort(self):
        self.is_open = False

    def clearPort(self):
        # 何も意味がないことを示すための警告
        warnings.warn("clearPort is meaningless in this sub class")

    def setPortName(self, port_name):
        # 何も意味がないことを示すための警告
        warnings.warn("setPortName is meaningless in this sub class")

    def getBytesAvailable(self):
        return len(self.received_packet) > 0

    def readPort(self, length):
        if len(self.received_packet) >= length:
            ret_packet = self.received_packet[:length]
            self.received_packet = self.received_packet[length:]
            return ret_packet
        else:
            warnings.warn("Not enough data in buffer")
            return []

    def writePort(self, packet):
        code, ret = self.xapi.getset_tgpio_modbus_data(packet, min_res_len=10, is_transparent_transmission=True)
        # TODO: check error code
        self.received_packet.extend(ret)
        return len(packet)

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        # NOTE: Serialポートの時と違って，writePort時に受信データを受け取るので，ここでは受信データを待つ必要がないので，タイムアウトは0のまま
        self.packet_timeout = 0.0

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        # NOTE: Serialポートの時と違って，writePort時に受信データを受け取るので，ここでは受信データを待つ必要がないので，タイムアウトは0のまま
        self.packet_timeout = 0.0

    def setupPort(self, cflag_baud):
        if self.is_open:
            self.closePort()

        code = self.xapi.set_tgpio_modbus_timeout(LATENCY_TIMER * 2 + 2, is_transparent_transmission=True)
        code = self.xapi.set_tgpio_modbus_baudrate(self.baudrate)
        self.is_open = True

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True
