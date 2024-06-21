#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import os.path
import sys
import tf2_ros
import time

sys.path.append(os.path.expanduser("~/rms_ros/src/rms_ros/"))

from rms_modules.manipulators import get_vx_bot
from rms_modules.pointclouds import read_pcd, unpack_rgb
from sensor_msgs.msg import PointCloud2


class PCDCollection(Node):
    def __init__(self, robot_model, path):
        """
        Initialize the PCDCollection class instance.

        @param robot_model: The model of the robot i.e., vx250 or vx300s.
        @param path: The write path of the PCD files.
        """

        super().__init__(f"{robot_model}_pcd_collection")

        # Input parameters:
        self.declare_parameters(namespace="", parameters=[
            ("robot_model", robot_model),
            ("path", path),
        ])
        self.robot_model = robot_model
        self.write_dir = path

        self.tf_name = f"{self.robot_model}_tfs.txt"
        self.tf_path = os.path.join(self.write_dir, self.tf_name)

        if not os.path.exists(self.write_dir):
            os.makedirs(self.write_dir)
            open(os.path.join(os.path.expanduser("~/rms_ros/src/rms_ros/scans"), f"{self.robot_model}_tfs.txt"), "a").close()

        # ROS publishers and subscribers:
        self.counter = 1
        self.data = None
        self.status = False
        self.subscription = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self.callback_pcd,
            10
        )

        #
        self.viewpoints_path = os.path.join(os.path.expanduser("~/rms_ros/src/rms_ros/config"),
                                            f"{self.robot_model}_viewpoints.txt")
        self.viewpoints = self.load_viewpoints()
        self.bot = get_vx_bot(self.robot_model)

        #
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback_pcd(self, msg):
        """
        Callback function for ROS2 PointCloud2 messages.

        @param msg: ROS2 PointCloud2 message.
        """

        self.data = msg
        self.sample_pcd()

    def sample_pcd(self):
        """
        Sample a point cloud.
        """

        if self.data and self.status:
            scan_name = f"{self.robot_model}_{self.counter:03d}.pcd"
            scan_path = os.path.join(self.write_dir, scan_name)
            self.write_pcd(scan_path)
            


    def get_transform(self):
        """
        Get the current transform from world to the camera frame.

        @return: Transform from world to the camera frame.
        """

        try:
            tf_future = self.tf_buffer.wait_for_transform_async(
                target_frame=f"{self.robot_model}/base_link",
                source_frame=f"{self.robot_model}/camera_depth_frame",
                time=rclpy.time.Time()
            )
            rclpy.spin_until_future_complete(self, tf_future)

            # Check if the future was successfully completed
            if tf_future.done():
                transform = self.tf_buffer.lookup_transform(
                    f"{self.robot_model}/base_link",
                    f"{self.robot_model}/camera_depth_frame",
                    rclpy.time.Time()
                )
                return transform
            else:
                print("Transform not available yet, retrying...")
        except (tf2_ros.TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            print(f"Transform exception occurred: {ex}, retrying...")


    def write_pcd(self, path):
        """
        Write the measured point cloud as a PCD file.

        @param path: The path at which the PCD file is written.
        """

        gen = read_pcd(self.data, skip_nans=True)
        ints = list(gen)
        xyz = np.array([[x[0], x[1], x[2]] for x in ints])
        rgb = np.array([unpack_rgb(x[3]) for x in ints])
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(xyz)
        o3d_cloud.colors = o3d.utility.Vector3dVector(rgb / 255.0)
        o3d.io.write_point_cloud(path, o3d_cloud)

    def write_tf(self, transform):
        """
        Write the measured transform from world to the camera frame to a new line in the transformations file.

        @param transform: The current transformation matrix from world to the camera frame.
        """

        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        rotation = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        tf_matrix = np.eye(4, 4)
        tf_matrix[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(rotation)
        tf_matrix[:3, 3] = translation
        formatted_tf_matrix = np.array([[f"{x:.10f}" for x in row] for row in tf_matrix])
        with open(self.tf_path, "a") as file:
            file.write(f"{self.counter:03d}: {formatted_tf_matrix}\n")

    def load_viewpoints(self):
        """
        Load all viewpoints from the respective TXT file.

        @return: A numpy array containing the viewpoints.
        """

        with open(self.viewpoints_path, "r") as file:
            lines = file.readlines()
        data = []
        for line in lines:
            values = [float(x) for x in line.strip().split(",")]
            data.append(values)
        return np.array(data)

    def move_to_viewpoint(self, x, y, z):
        """
        Move the manipulator to the current viewpoint.
        """

        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z)
        duration = 2  # cannot be 1 for correct tfs
        tic = time.time()
        while time.time() < tic + duration:
            pass

    def wait_for_pcd(self):
        """
        Wait for a PointCloud2 message to be received or timeout after five seconds.
        """

        self.data = None
        self.status = True
        tic = time.time()
        while self.data is None:
            toc = time.time()
            rclpy.spin_once(self, timeout_sec=0.1)
            if toc - tic > 5.0:
                break

    def run(self):
        """
        Run the node.

        This function will cycle the current manipulator through all viewpoints and collect a point cloud sample at each
        viewpoint. After a point cloud is sampled at all viewpoints, the manipulator returns to the home position (to
        prevent collisions with any baseplate features) and then the sleep position.
        """

        self.get_transform()
        for view in self.viewpoints:
            self.move_to_viewpoint(x=view[1], y=view[2], z=view[3])
            self.wait_for_pcd()
            self.write_tf(self.get_transform())
            self.counter += 1
        self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()
        try:
            self.bot.shutdown()
        except:
            pass


def main():
    """
    Collect PCD files.
    """

    parser = argparse.ArgumentParser(description="iterate through viewpoints and collect point clouds")
    parser.add_argument("--robot_model", type=str, required=True, help="robot model i.e., vx250 or vx300s")
    parser.add_argument("--path", type=str, required=True, help="path to pcd scan directory")
    args = parser.parse_args()

    ros_args = sys.argv[1:]
    ros_args.extend(["--ros-args", "--param", f"robot_model:={args.robot_model}", "--param", f"path:={args.path}"])

    rclpy.init(args=ros_args)
    node = PCDCollection(robot_model=args.robot_model, path=args.path)
    node.run()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
