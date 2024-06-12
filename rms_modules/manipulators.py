
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from rms_modules.base import *
import subprocess
import sys

def get_vx_bot():
    f_data = load_rms_config()
    vxmodel = f_data["robot_model"]

    if vxmodel == "vx250":
        bot = get_vx250()
    elif vxmodel == "vx300":
        bot = get_vx300()
    else:
        print(" >>> [Error: check robot_model in config file]")
        sys.exit(1)

    return bot

def get_vx_node_status():
    vx250_node = "/vx250/xs_sdk"
    vx300_node = "/vx300/xs_sdk"

    nodes = subprocess.run("ros2 node list", shell=True, capture_output=True)
    nodes_bytes = nodes.stdout
    nodes_str = nodes_bytes.decode("UTF-8")

    return vx250_node in nodes_str, vx300_node in nodes_str

def get_vx250():
    bot = InterbotixManipulatorXS(
            robot_model = "vx250",
            group_name = "arm",
            gripper_name = "gripper"
        )
    return bot

def get_vx300():
    bot = InterbotixManipulatorXS(
            robot_model = "vx300",
            group_name = "arm",
            gripper_name = "gripper"
        )
    return bot

def go2sleep():
    bot = get_vx_bot()
    bot.arm.go_to_home_pose()   # go to home position
    bot.arm.go_to_sleep_pose()  # go to sleep position
    bot.shutdown()

