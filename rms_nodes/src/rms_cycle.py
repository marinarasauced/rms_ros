
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import tf2_ros

import argparse
import numpy as np
import os
import open3d as o3d
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from rms_modules.base import *
from rms_modules.manipulators import get_vx_bot
from rms_modules.pointclouds import *


class RMSCycle(Node):
    def __init__(self, configuration, write_directory):
        super().__init__("pcd_collecting")
        
        self.subscription = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self.callback_pointcloud,
            10
        )

        self.declare_parameter("configuration", configuration)
        self.configuration = self.get_parameter("configuration").get_parameter_value().integer_value
        self.declare_parameter("write_directory", write_directory)
        self.write_dir = self.get_parameter("write_directory").get_parameter_value().string_value

        self.path2config = get_path2config()
        self.path2scans = get_path2scans()
        self.robot_model = unpack_rms_config()

        self.bot = get_vx_bot()
        self.data = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.counter = 1

        self.trans_path = os.path.join(self.write_dir, f"{self.robot_model}_transformations.txt")
        self.trans_file = open(self.trans_path, 'a')

        self.load_waypoints()

    def callback_pointcloud(self, msg):
        self.data = msg
        self.capture_pointcloud()

    def capture_pointcloud(self):
        if self.data:
            try:
                transform = None
                while transform is None:
                    transform = self.get_transform()
                    rclpy.spin_once(self)
                
                self.get_logger().info(f"translation: (x: {transform.transform.translation.x:.3f}, "
                                    f"y: {transform.transform.translation.y:.3f}, "
                                    f"z: {transform.transform.translation.z:.3f})")
                
                self.get_logger().info(f"rotation: (x: {transform.transform.rotation.x:.3f}, "
                                    f"y: {transform.transform.rotation.y:.3f}, "
                                    f"z: {transform.transform.rotation.z:.3f}, "
                                    f"w: {transform.transform.rotation.w:.3f})")
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"could not transform: {e}")
                return
            
            log_doing(f"saving pointcloud {self.counter}")

            scan_path = self.get_write_path()
            self.write_pointcloud(self.data, scan_path)
            self.write_transform(transform)
            self.counter += 1

    def get_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'world', rclpy.time.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"could not transform: {e}")
            return None
        
    def get_write_path(self):
        scan_path = os.path.join(self.write_dir, f"{self.robot_model}-{self.counter:03d}.pcd")
        return scan_path
    
    def load_waypoints(self):
        with open(os.path.join(self.path2config, "waypoints.txt")) as file: 
            lines = file.readlines()

        data = []
        for line in lines:
            values = [float(x) for x in line.strip().split(',')]
            data.append(values)

        self.waypoints = np.array(data)

    def move_to_waypoint(self, x, y, z):
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z)
        tic = time.time()
        toc = tic + 1
        while time.time() < toc:
            pass

    def wait_for_pointcloud(self):
        self.data = None
        tic = time.time()
        while self.data is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - tic > 5:  # timeout after 5 seconds
                log_doing("timeout waiting for pointcloud data")
                break

    def write_pointcloud(self, data, path):
        gen = read_pointcloud(data, skip_nans=True)
        int_data = list(gen)
        xyz = np.array([[x[0], x[1], x[2]] for x in int_data])
        rgb = np.array([unpack_rgb(x[3]) for x in int_data])

        o3dcloud = o3d.geometry.PointCloud()
        o3dcloud.points = o3d.utility.Vector3dVector(xyz)
        o3dcloud.colors = o3d.utility.Vector3dVector(rgb / 255.0)
        o3d.io.write_point_cloud(path, o3dcloud)

    def write_transform(self, transform: TransformStamped):
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        rotation = np.array([
            -transform.transform.rotation.x,
            -transform.transform.rotation.y,
            -transform.transform.rotation.z,
            -transform.transform.rotation.w
        ])
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(rotation)
        transformation_matrix[:3, 3] = translation
        formatted_array = np.array([[f"{x:.10f}" for x in row] for row in transformation_matrix])
        with open(self.trans_path, "a") as f:
            f.write(f"{self.counter:03d}: {formatted_array}\n")

        
    def run(self):
        for point in self.waypoints:
            self.move_to_waypoint(x=point[1], y=point[2], z=point[3])
            self.wait_for_pointcloud()
        self.trans_file.close()

########################################################

def main(args=None):
    parser = argparse.ArgumentParser(description='collect pcd views')
    parser.add_argument('--configuration', type=int, default=1, help='RMS configuration index', required=True)
    parser.add_argument('--write_directory', type=str, default="", help='path to save pcd and tf files', required=True)  
    args = parser.parse_args()

    rclpy_args = sys.argv[:1]
    rclpy_args.extend(['--ros-args', '--param', f'configuration:={args.configuration}', '--param', f'write_directory:={args.write_directory}'])

    rclpy.init(args=rclpy_args)
    node = RMSCycle(configuration=args.configuration, write_directory=args.write_directory)
    node.run()
    rclpy.shutdown()

if __name__=="__main__":
    main()

