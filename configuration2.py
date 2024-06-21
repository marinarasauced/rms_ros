import os.path
import subprocess

from rms_modules.manipulators import VXManipulator, get_vx_node_status, check_for_file, get_primary_write_dir, get_secondary_write_dir
from rms_modules.pointclouds import write_pcd_from_stl


def main():
    """
    Run an RMS configuration one trial.

    This executable runs the RMS single scan configuration. If the manipulator is online, it will cycle through all
    viewpoints, sample point clouds, return to the home position, go to sleep, register all scans, and then compare them
    to the model. Before running this executable, ensure that the part name is specified correctly and that it has been
    converted to a PCD file from its STL source file.

    """

    # Inputs:
    part_id = "test_object_b_scaled"

    # Check whether manipulator(s) are online:
    primary = VXManipulator("primary")
    secondary = VXManipulator("secondary")
    if not get_vx_node_status(primary):
        print(f"{primary.robot_model} is offline, please launch the interbotix control node")
        return
    if not get_vx_node_status(secondary):
        print(f"{secondary.robot_model} is offline, please launch the interbotix control node")
        return

    # Check whether PCD file exists and if not, check if STL file exists:
    path2pcd = os.path.join(os.path.expanduser("~/rms_ros/src/rms_ros/models/pcd/"), f"{part_id}.pcd")
    path2stl = os.path.join(os.path.expanduser("~/rms_ros/src/rms_ros/models/stl/"), f"{part_id}.stl")
    if not check_for_file(path2pcd):
        print(f"unable to find {part_id}.pcd, please check if it is in the models/pcd/ subdirectory")
        if check_for_file(path2stl):
            write_pcd_from_stl(path2stl, path2pcd)
        else:
            print(f"unable to find {part_id}.stl, please check if it is in the models/stl/ subdirectory")
            return
        
    # Connect to secondary:
    

    # Collect point clouds:
    primary_dir = get_primary_write_dir(2)
    secondary_dir = get_secondary_write_dir(secondary)
    primary_path_to_pcd_collection = os.path.expanduser(f"/home/{primary.user}/rms_ros/src/rms_ros/rms_nodes/src/pcd_collection.py")
    secondary_path_to_pcd_collection = os.path.expanduser(f"/home/{secondary.user}/rms_ros/src/rms_ros/rms_nodes/src/pcd_collection.py")
    primary_cmd = [f"python3",
                    f"{primary_path_to_pcd_collection}",
                    f"--robot_model",
                    f"{primary.robot_model}",
                    f"--path",
                    f"{primary_dir}"
                    ]
    secondary_cmd = f"source /opt/ros/humble/setup.bash && source ~/rms_ros/install/setup.bash && source ~/interbotix_ws/install/setup.bash && python3 {secondary_path_to_pcd_collection} --robot_model {secondary.robot_model} --path {secondary_dir}"
    secondary.get_client()
    secondary.execute_client_command(secondary_cmd)
    subprocess.run(primary_cmd)

    input("press enter")

    # Register point clouds:
    path_to_pcd_registration = os.path.expanduser("~/rms_ros/src/rms_ros/rms_nodes/src/pcd_registration.py")
    subprocess.run([f"python3",
                    f"{path_to_pcd_registration}",
                    f"--path",
                    f"{primary_dir}"
                    ])


if __name__ == "__main__":
    main()
