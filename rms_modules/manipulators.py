import datetime
import json
import os.path
from paramiko import SSHClient, AutoAddPolicy
import re
from scp import SCPClient
import subprocess

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class VXManipulator:
    def __init__(self, category):
        """
        Initialize the class instance with the specified parameters.
        """

        self.robot_model, self.host, self.user, self.password = load_config_json(f"{category}.json")
        self.ssh = None

    def get_client(self):
        """
        """

        if self.ssh is None:
            ssh = SSHClient()
            ssh.load_system_host_keys()
            ssh.set_missing_host_key_policy(AutoAddPolicy())
            ssh.connect(
                hostname=self.host,
                username=self.user,
                password=self.password
            )
            self.ssh = ssh

    def execute_client_command(self, command):
        """
        """

        try:
            self.ssh.exec_command(command)
        except:
            print("failed")
            return
        
    def download_from_client(self, remote_path, local_path):
        """
        Download all files from a remote path to a local path.

        @param remote_path: The path to the repository on the remote machine.
        @param local_path: The path where the repository will be downloaded to on the local machine.
        """

        scp = SCPClient(self.ssh.get_transport())
        scp.get(remote_path=remote_path, local_path=local_path, recursive=True)
        scp.close()

    def close_client(self):
        """
        """

        try:
            self.ssh.close()
        except:
            return



def load_config_json(name):
    """
    Load the configuration parameters from the corresponding JSON file.

    @param name: The name of the JSON file in the config subdir.
    @return robot_model: The model, host address, username, and password of the manipulator.
    """

    path = os.path.join(os.path.expanduser("~/rms_ros/src/rms_ros/config"), name)
    with open(path, "r") as file:
        data = json.load(file)
    robot_model = data["robot_model"]
    host = data["host"]
    username = data["username"]
    password = data["password"]
    return robot_model, host, username, password


def get_vx_bot(robot_model):
    """

    @param robot_model: The model of the manipulator i.e., vx250 or vx300s.
    @return: The InterbotixManipulatorsXS instance of the manipulator.
    """

    bot = InterbotixManipulatorXS(
        robot_model=robot_model,
        group_name="arm",
        gripper_name="gripper"
    )
    return bot


def get_vx_node_status(manipulator):
    """
    Get the status of a manipulator's ROS nodes.

    @param manipulator: A VXManipulator class instance.
    @return: 1 if the manipulator's nodes are running, 0 else.
    """

    node = f"/{manipulator.robot_model}/xs_sdk"
    nodes = subprocess.run("ros2 node list", shell=True, capture_output=True)
    node_bytes = nodes.stdout
    node_list = node_bytes.decode("UTF-8")
    return node in node_list


def check_for_file(path):
    """
    Check if a file path exists.

    @param path: The path to the file.
    @return: 1 if the file exists, 0 else.
    """

    return os.path.exists(path)


def get_primary_write_dir(configuration):
    """
    Get the PCD write directory for the primary manipulator.

    @param configuration: The RMS configuration i.e., 1, 2, or 3.
    @return: The path to the PCD write directory.
    """

    date = datetime.datetime.now().strftime("%Y-%m-%d")
    scan_dir = os.path.expanduser(f"~/rms_ros/src/rms_ros/scans/{date}/")
    time = datetime.datetime.now().strftime("%H:%M:%S")
    if not os.path.exists(scan_dir):
        path = os.path.join(scan_dir, f"C{configuration}_0001_{time}")
        os.makedirs(path)
        return path
    if configuration == 1:
        pattern = re.compile(r"^C1_(\d{4})_\d{2}:\d{2}:\d{2}$")
    elif configuration == 2:
        pattern = re.compile(r"^C2_(\d{4})_\d{2}:\d{2}:\d{2}$")
    elif configuration == 3:
        pattern = re.compile(r"^C3_(\d{4})_\d{2}:\d{2}:\d{2}$")
    numbers = []
    for item in os.listdir(scan_dir):
        if os.path.isdir(os.path.join(scan_dir, item)):
            match = pattern.match(item)
            if match:
                numbers.append(match.group(1))
    if numbers:
        index = int(max(numbers)) + 1
        path = os.path.join(scan_dir, f"C{configuration}_{index:04d}_{time}")
        os.makedirs(path)
        return path
    else:
        path = os.path.join(scan_dir, f"C{configuration}_0001_{time}")
        os.makedirs(path)
        return path


def get_secondary_write_dir(secondary):
    """
    Get the PCD write directory for the secondary manipulator.

    @return: The path to the PCD write directory.
    """

    return f"/home/{secondary.user}/rms_ros/src/rms_ros/scans/"
