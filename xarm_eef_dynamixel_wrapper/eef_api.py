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
        time.sleep(0.2)
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
    
    # manually set the config
    # eef_config = EEFConfig(
    #     xarm_ip="192.168.10.251",
    #     gripper_open=(3755 - 100) * 0.001534,  # 3755 [step] -> 0.001534 [rad/step]
    #     gripper_close=(911 + 100) * 0.001534,  # 911 [step] -> 0.001534 [rad/step]
    #     dxl_config=DxlConfig(
    #         id=1,
    #         baudrate=1000000,
    #         addr_torque_enable=64,
    #         addr_goal_position=116,
    #         addr_present_position=132,
    #         torque_enable=1,
    #         torque_disable=0,
    #         unit_pos=0.001534,  # 0.001534 [rad/step]
    #         unit_vel=0.240,  # 0.240 [rad/s/step]
    #         unit_effort=0.001,  # 1 [mA/step]
    #     ),
    # )
    # # write current config to yaml
    # eef_config.save(path)
    # print(f"config saved to {path}")
    
    
    eef_api = EEFAPI(eef_config)
    # setup
    # eef_api.setup()
    eef_api.enable_torque()
    
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
    
# class Lite6Robot(Robot):
#     # EEFにOpenParallelGripper(XL330, ID:1)を使った場合の構成
#     GRIPPER_OPEN = 3755 - 100
#     GRIPPER_CLOSE = 911 + 100
#     DXL_ID = 1
#     DYNAMIXEL_BAUDRATE = 1000000
#     ADDR_TORQUE_ENABLE = 64
#     ADDR_GOAL_POSITION = 116
#     ADDR_PRESENT_POSITION = 132
#     TORQUE_ENABLE = 1  # Value for enabling the torque
#     TORQUE_DISABLE = 0  # Value for disabling the torque
#     #  MAX_DELTA = 0.2
#     DEFAULT_MAX_DELTA = 0.05

#     def num_dofs(self) -> int:
#         return 7

#     def get_joint_state(self) -> np.ndarray:
#         state = self.get_state()
#         gripper = state.gripper_pos()
#         all_dofs = np.concatenate([state.joints(), np.array([gripper])])
#         return all_dofs

#     def command_joint_state(self, joint_state: np.ndarray) -> None:
#         if len(joint_state) == 6:
#             self.set_command(joint_state, None)
#         elif len(joint_state) == 7:
#             self.set_command(joint_state[:6], joint_state[6])
#         else:
#             raise ValueError(f"Invalid joint state: {joint_state}, len={len(joint_state)}")

#     def stop(self):
#         self.running = False
#         if self.command_thread is not None:
#             self.command_thread.join()

#         if self.robot is not None:
#             self.eef_packetHandler.write1ByteTxRx(
#                 self.eef_portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
#             )  # Torque Disable
#             self.eef_portHandler.closePort()
#             self.robot.disconnect()

#     def __init__(
#         self,
#         ip: str = "192.168.10.251",
#         real: bool = True,
#         control_frequency: float = 200.0,
#         max_delta: float = DEFAULT_MAX_DELTA,
#     ):
#         print(ip)
#         self.real = real
#         self.max_delta = max_delta
#         if real:
#             from xarm.wrapper import XArmAPI
#             from dynamixel_sdk import PacketHandler
#             from xarm_eef_dynamixel_wrapper.endeffector_port_handler import EndEffectorPortHandler

#             self.robot = XArmAPI(ip, is_radian=True)
#             self.eef_portHandler = EndEffectorPortHandler(self.robot, baudrate=self.DYNAMIXEL_BAUDRATE, latency_timer=1)
#             self.eef_packetHandler = PacketHandler(2.0)  # 2.0 is protocol version
#         else:
#             self.robot = None

#         self._control_frequency = control_frequency
#         self._clear_error_states()
#         self._set_gripper_position(self.GRIPPER_OPEN)

#         self.last_state_lock = threading.Lock()
#         self.target_command_lock = threading.Lock()
#         self.last_state = self._update_last_state()
#         self.target_command = {
#             "joints": self.last_state.joints(),
#             "gripper": 0,
#         }
#         self.running = True
#         self.command_thread = None
#         if real:
#             self.command_thread = threading.Thread(target=self._robot_thread)
#             self.command_thread.start()

#     def get_state(self) -> RobotState:
#         with self.last_state_lock:
#             return self.last_state

#     def set_command(self, joints: np.ndarray, gripper: Optional[float] = None) -> None:
#         with self.target_command_lock:
#             self.target_command = {
#                 "joints": joints,
#                 "gripper": gripper,
#             }

#     def _clear_error_states(self):
#         if self.robot is None:
#             return
#         self.robot.clean_error()
#         self.robot.clean_warn()
#         self.robot.motion_enable(True)
#         time.sleep(1)
#         self.robot.set_mode(1)
#         time.sleep(1)
#         self.robot.set_collision_sensitivity(0)
#         time.sleep(1)
#         self.robot.set_state(state=0)
#         time.sleep(1)

#         # setup gripper
#         self.eef_portHandler.openPort()
#         self.eef_portHandler.setBaudRate(self.DYNAMIXEL_BAUDRATE)
#         time.sleep(1)
#         self.eef_packetHandler.reboot(self.eef_portHandler, self.DXL_ID)  # reboot dynamixel
#         time.sleep(1)
#         self.eef_packetHandler.write1ByteTxRx(
#             self.eef_portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
#         )  # Torque Enable
#         time.sleep(1)

#     def _get_gripper_pos(self) -> float:
#         if self.robot is None:
#             return 0.0

#         # dynamixelの角度を取得
#         dxl_present_position, dxl_comm_result, dxl_error = self.eef_packetHandler.read4ByteTxRx(
#             self.eef_portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION
#         )
#         while dxl_comm_result != 0 or dxl_error != 0:
#             print("%s" % self.eef_packetHandler.getTxRxResult(dxl_comm_result))
#             print("%s" % self.eef_packetHandler.getRxPacketError(dxl_error))
#             time.sleep(0.001)
#             dxl_present_position, dxl_comm_result, dxl_error = self.eef_packetHandler.read4ByteTxRx(
#                 self.eef_portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION
#             )
#             if dxl_error == 128:  # 128 is error code for hardware error
#                 print("Hardware Error detected. rebooting dynamixel")
#                 self._clear_error_states()

#         normalized_gripper_pos = (dxl_present_position - self.GRIPPER_OPEN) / (self.GRIPPER_CLOSE - self.GRIPPER_OPEN)
#         return normalized_gripper_pos

#     def _set_gripper_position(self, pos: int) -> None:
#         if self.robot is None:
#             return
#         dxl_comm_result, dxl_error = self.eef_packetHandler.write4ByteTxRx(
#             self.eef_portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, pos
#         )

#     def _robot_thread(self):
#         rate = Rate(duration=1 / self._control_frequency)  # command and update rate for robot
#         step_times = []
#         count = 0

#         while self.running:
#             s_t = time.time()
#             # update last state
#             self.last_state = self._update_last_state()
#             with self.target_command_lock:
#                 joint_delta = np.array(self.target_command["joints"] - self.last_state.joints())
#                 gripper_command = self.target_command["gripper"]

#             norm = np.linalg.norm(joint_delta)

#             # threshold delta to be at most 0.01 in norm space
#             if norm > self.max_delta:
#                 delta = joint_delta / norm * self.max_delta
#             else:
#                 delta = joint_delta

#             # command position
#             self._set_position(
#                 self.last_state.joints() + delta,
#             )

#             if gripper_command is not None:
#                 set_point = gripper_command
#                 self._set_gripper_position(
#                     int(self.GRIPPER_OPEN + set_point * (self.GRIPPER_CLOSE - self.GRIPPER_OPEN))
#                 )
#             self.last_state = self._update_last_state()

#             rate.sleep()
#             step_times.append(time.time() - s_t)
#             count += 1
#             if count % 1000 == 0:
#                 # Mean, Std, Min, Max, only show 3 decimal places and string pad with 10 spaces
#                 frequency = 1 / np.mean(step_times)
#                 # print(f"Step time - mean: {np.mean(step_times):10.3f}, std: {np.std(step_times):10.3f}, min: {np.min(step_times):10.3f}, max: {np.max(step_times):10.3f}")
#                 print(
#                     f"Low  Level Frequency - mean: {frequency:10.3f}, std: {np.std(frequency):10.3f}, min: {np.min(frequency):10.3f}, max: {np.max(frequency):10.3f}"
#                 )
#                 step_times = []

#     def _update_last_state(self) -> RobotState:
#         with self.last_state_lock:
#             if self.robot is None:
#                 return RobotState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, np.zeros(3))

#             gripper_pos = self._get_gripper_pos()

#             code, servo_angle = self.robot.get_servo_angle(is_radian=True)
#             while code != 0:
#                 print(f"Error code {code} in get_servo_angle().")
#                 self._clear_error_states()
#                 code, servo_angle = self.robot.get_servo_angle(is_radian=True)

#             code, cart_pos = self.robot.get_position_aa(is_radian=True)
#             while code != 0:
#                 print(f"Error code {code} in get_position().")
#                 self._clear_error_states()
#                 code, cart_pos = self.robot.get_position_aa(is_radian=True)

#             cart_pos = np.array(cart_pos)
#             aa = cart_pos[3:]
#             cart_pos[:3] /= 1000

#             return RobotState.from_robot(
#                 cart_pos,
#                 servo_angle,
#                 gripper_pos,
#                 aa,
#             )

#     def _set_position(
#         self,
#         joints: np.ndarray,
#     ) -> None:
#         if self.robot is None:
#             return
#         # threhold xyz to be in  min max
#         ret = self.robot.set_servo_angle_j(joints, wait=False, is_radian=True)
#         if ret in [1, 9]:
#             self._clear_error_states()

#     def get_observations(self) -> Dict[str, np.ndarray]:
#         state = self.get_state()
#         pos_quat = np.concatenate([state.cartesian_pos(), state.quat()])
#         joints = self.get_joint_state()
#         return {
#             "joint_positions": joints,  # rotational joint + gripper state
#             "joint_velocities": joints,
#             "ee_pos_quat": pos_quat,
#             "gripper_position": np.array(state.gripper_pos()),
#         }


# def main():
#     ip = "192.168.10.251"
#     robot = Lite6Robot(ip)
#     import time

#     time.sleep(1)
#     print(robot.get_state())

#     time.sleep(1)
#     print(robot.get_state())
#     print("end")
#     robot.stop()


# if __name__ == "__main__":
#     main()
