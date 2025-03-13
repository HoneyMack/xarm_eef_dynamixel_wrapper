import dataclasses
import warnings
import time
from yaml_config import YamlConfig

from xarm.wrapper import XArmAPI
from dynamixel_sdk import PacketHandler
from xarm_eef_dynamixel_wrapper.endeffector_port_handler import EndEffectorPortHandler


@dataclasses.dataclass
class DxlConfig(YamlConfig):
    id: int
    baudrate: int
    addr_torque_enable: int
    addr_goal_position: int
    addr_present_position: int
    torque_enable: int
    torque_disable: int
    unit_pos: float  # [rad/step]
    unit_vel: float  # [rad/s/step]
    unit_effort: float  # [A/step]


@dataclasses.dataclass
class EEFConfig(YamlConfig):
    xarm_ip: str
    gripper_open: float  # [rad]
    gripper_close: float  # [rad]
    dxl_config: DxlConfig


@dataclasses.dataclass(frozen=True)
class EEFState:
    pos: float | None = None  # position of the end effector [rad] [0 ~ max]. 0: close, max: open
    vel: float | None = None  # velocity of the end effector [rad/s]
    effort: float | None = None  # effort of the end effector [A]
    norm_pos: float | None = None  # normalized position of the end effector [0, 1]. 0: close, 1: open
    norm_vel: float | None = None  # normalized velocity of the end effector.  -: close, +: open


class EEFAPI:
    def __init__(self, eef_config: EEFConfig):
        self.eef_config = eef_config
        self.dxl_config = eef_config.dxl_config

        self.robot = XArmAPI(self.eef_config.xarm_ip, is_radian=True)
        self.eef_portHandler = EndEffectorPortHandler(self.robot, baudrate=self.dxl_config.baudrate, latency_timer=1)
        self.eef_packetHandler = PacketHandler(2.0)  # 2.0 is protocol version
        self.is_working = False

        # set useful variables
        self.open_minus_close = self.eef_config.gripper_open - self.eef_config.gripper_close
        self.pos_max_rad = abs(self.eef_config.gripper_open - self.eef_config.gripper_close)

    def get_state(self) -> EEFState:
        pos = self._get_current_position() * self.dxl_config.unit_pos
        pos_offset = self.open_minus_close / abs(self.open_minus_close) * (pos - self.eef_config.gripper_close)
        norm_pos = (pos - self.eef_config.gripper_close) / self.open_minus_close
        # TODO: implement velocity and effort reading
        return EEFState(pos=pos_offset, vel=None, effort=None, norm_pos=norm_pos, norm_vel=None)

    def set_state(self, state: EEFState):

        if state.pos is not None:
            # check if the position is within the range
            pos = state.pos
            if not (0 <= state.pos <= self.pos_max_rad):
                warnings.warn(f"position {state.pos} is out of range. It will be clipped.")
                pos = max(0, min(self.pos_max_rad, state.pos))

            sign = 1 if self.open_minus_close >= 0 else -1
            pos_motor = pos * sign + self.eef_config.gripper_close

            pos_motor_pulse = int(pos_motor / self.dxl_config.unit_pos)
            self._set_position(pos_motor_pulse)
        elif state.norm_pos is not None:
            norm_pos = state.norm_pos
            if not (0 <= state.norm_pos <= 1):
                warnings.warn(f"normalized position {state.norm_pos} is out of range. It will be clipped.")
                norm_pos = max(0, min(1, state.norm_pos))

            pos_motor = self.eef_config.gripper_close + norm_pos * self.open_minus_close
            pos_motor_pulse = int(pos_motor / self.dxl_config.unit_pos)
            self._set_position(pos_motor_pulse)
        else:
            # TODO: implement velocity and effort control
            raise NotImplementedError("velocity, effort, and norm_vel are not implemented yet")
    
    def setup(self):
        self.clear_error_states()
        self.enable_torque()

    def clear_error_states(self):
        # setup gripper
        self.eef_portHandler.openPort()
        self.eef_portHandler.setBaudRate(self.dxl_config.baudrate)
        time.sleep(0.1)
        self.eef_packetHandler.reboot(self.eef_portHandler, self.dxl_config.id)
        time.sleep(0.5)
        self.is_working = True

    def enable_torque(self):
        self.eef_packetHandler.write1ByteTxRx(
            self.eef_portHandler, self.dxl_config.id, self.dxl_config.addr_torque_enable, self.dxl_config.torque_enable
        )

    def disable_torque(self):
        self.eef_packetHandler.write1ByteTxRx(
            self.eef_portHandler, self.dxl_config.id, self.dxl_config.addr_torque_enable, self.dxl_config.torque_disable
        )

    def shutdown(self):
        self.disable_torque()
        self.eef_portHandler.closePort()
        self.robot.disconnect()
        self.is_working = False

    def __del__(self):
        if self.is_working:
            self.shutdown()

    def _get_current_position(self) -> float:
        dxl_present_position, dxl_comm_result, dxl_error = self.eef_packetHandler.read4ByteTxRx(
            self.eef_portHandler, self.dxl_config.id, self.dxl_config.addr_present_position
        )
        while dxl_comm_result != 0 or dxl_error != 0:
            print("%s" % self.eef_packetHandler.getTxRxResult(dxl_comm_result))
            print("%s" % self.eef_packetHandler.getRxPacketError(dxl_error))
            time.sleep(0.001)  # Prevent the loop from running too fast
            dxl_present_position, dxl_comm_result, dxl_error = self.eef_packetHandler.read4ByteTxRx(
                self.eef_portHandler, self.dxl_config.id, self.dxl_config.addr_present_position
            )
            if dxl_error == 128:
                print("Hardware Error detected. rebooting dynamixel")
                self.clear_error_states()
        return dxl_present_position

    def _set_position(self, pos_pulse: int) -> None:
        dxl_comm_result, dxl_error = self.eef_packetHandler.write4ByteTxRx(
            self.eef_portHandler, self.dxl_config.id, self.dxl_config.addr_goal_position, pos_pulse
        )
        # TODO: add error handling codes


if __name__ == "__main__":
    import pathlib
    
    # load config from yaml
    path = pathlib.Path(__file__).parent.parent / "configs" / "open_parallel_gripper.yaml"
    eef_config = EEFConfig.load(path)
        
    
    eef_api = EEFAPI(eef_config)
    # setup
    eef_api.setup()
    
    # control the eef with normalized position [0, 1]. 0: close, 1: open
    time.sleep(1)
    print(eef_api.get_state())
    eef_api.set_state(EEFState(norm_pos=0.0))
    time.sleep(1)
    print(eef_api.get_state())
    eef_api.set_state(EEFState(norm_pos=0.5))
    time.sleep(1)
    print(eef_api.get_state())
    eef_api.set_state(EEFState(norm_pos=1.0))
    time.sleep(1)
    print(eef_api.get_state())
    
    # control the eef with position [rad]. 0: close, max: open
    eef_api.set_state(EEFState(pos=0))
    time.sleep(1)
    print(eef_api.get_state())
    eef_api.set_state(EEFState(pos=2))
    time.sleep(1)
    print(eef_api.get_state())
    eef_api.set_state(EEFState(pos=4))
    time.sleep(1)
    print(eef_api.get_state())
    
    # shutdown
    eef_api.shutdown()
    print("end")