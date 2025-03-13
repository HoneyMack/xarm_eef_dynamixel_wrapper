# Made by: Takuya Okubo

import warnings
from dynamixel_sdk import PortHandler
from dynamixel_sdk.robotis_def import *
from xarm.wrapper import XArmAPI

from xarm_eef_dynamixel_wrapper.constants import DEFAULT_BAUDRATE, DEFAULT_LATENCY_TIMER

class EndEffectorPortHandler(PortHandler):
    def __init__(self, xArmAPI: XArmAPI, baudrate:int=DEFAULT_BAUDRATE, latency_timer:int=DEFAULT_LATENCY_TIMER):
        self.is_open = False
        self.baudrate = baudrate
        self.latency_timer = latency_timer
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0
        self.received_packet = []

        self.is_using = False
        self.xapi = xArmAPI

    def closePort(self):
        self.is_open = False

    def clearPort(self):
        # Warning to indicate that this method is meaningless
        warnings.warn("clearPort is meaningless in this sub class")

    def setPortName(self, port_name):
        # Warning to indicate that this method is meaningless
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
        # NOTE: Unlike with a serial port, since the received data is obtained during writePort, there is no need to wait for the received data here, so the timeout remains 0
        self.packet_timeout = 0.0

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        # NOTE: Unlike with a serial port, since the received data is obtained during writePort, there is no need to wait for the received data here, so the timeout remains 0
        self.packet_timeout = 0.0

    def setupPort(self, cflag_baud):
        if self.is_open:
            self.closePort()

        code = self.xapi.set_tgpio_modbus_timeout(self.latency_timer * 2 + 2, is_transparent_transmission=True)
        code = self.xapi.set_tgpio_modbus_baudrate(self.baudrate)
        self.is_open = True

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True
