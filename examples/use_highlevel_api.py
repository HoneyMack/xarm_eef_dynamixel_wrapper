import time
import pathlib

from xarm_eef_dynamixel_wrapper.eef_api import EEFConfig, EEFState, EEFAPI


if __name__ == "__main__":

    # Step 0. load config from yaml: you can also manually create EEFConfig object. but it is recommended to load from yaml file.
    path = pathlib.Path(__file__).parent.parent / "configs" / "open_parallel_gripper.yaml"
    eef_config = EEFConfig.load(path)

    # Step 1. setup
    eef_api = EEFAPI(eef_config)
    eef_api.setup()

    # Step 2. CONTROL!
    # Option 1. control the eef with normalized position [0, 1]. 0: close, 1: open
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

    # Option 2. control the eef with position [rad]. 0: close, max: open
    eef_api.set_state(EEFState(pos=0))
    time.sleep(1)
    print(eef_api.get_state())
    eef_api.set_state(EEFState(pos=2))
    time.sleep(1)
    print(eef_api.get_state())
    eef_api.set_state(EEFState(pos=4))
    time.sleep(1)
    print(eef_api.get_state())

    # Step 3. shutdown (last step)
    eef_api.shutdown()
    print("end")
