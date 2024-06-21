#!/usr/bin/env python3

import argparse
import glob
import rclpy
import numpy as np
import open3d as o3d
import os.path
import sys


class PCDRegistration(rclpy.node.Node):
    def __init__(self, path):
        """
        Initialize the PCDCollection class instance.

        @param path: The read and write path of the PCD files.
        """

        super().__init__(f"pcd_registration")

        # Input parameters:

        self.declare_parameters(namespace="", parameters=[
            ("path", path),
        ])
        self.dir = path
        self.vx250_files, self.vx300s_files = self.get_pcd_file_paths()
        self.pcd_ = self.get_pcd_files()
        self.tf_ = self.get_pcd_transformations()

    def get_pcd_file_paths(self):
        """
        Get the file paths to all PCD scans from a directory.

        @return: A list of all paths to all vx250 and vx300s PCD scans from the current trial.
        """

        vx250_search = os.path.join(self.dir, "vx250-*.pcd")
        vx300s_search = os.path.join(self.dir, "vx300s-*.pcd")
        vx250_files = []
        vx300s_files = []
        vx250_files.extend(glob.glob(vx250_search))
        vx300s_files.extend(glob.glob(vx300s_search))
        if not vx250_files and not vx300s_files:
            print("cannot located PCD files")
            return 0
        return vx250_files, vx300s_files

    def get_pcd_files(self):
        """
        Get a list of all PCD files contents.

        @return: A list of all content from the previously identified PCD files.
        """

        paths = self.vx250_files + self.vx300s_files
        return [o3d.io.read_point_cloud(pcd) for pcd in paths]

    def get_pcd_transformations(self):
        """
        Get the transformations from the camera frame to world at the instances of PCD collection.

        @return: A list of all transformations.
        """

        vx250_search = os.path.join(self.dir, "vx250_tfs.txt")
        vx300s_search = os.path.join(self.dir, "vx300s_tfs.txt")
        vx250_files = []
        vx300s_files = []
        vx250_files.extend(glob.glob(vx250_search))
        vx300s_files.extend(glob.glob(vx300s_search))
        tf_ = []
        for path in vx250_files + vx300s_files:
            with open(path, "r") as file:
                lines = file.readlines()
                idx = 0
                while idx < len(lines):
                    if ":" in lines[idx]:
                        try:
                            matrix_str = lines[idx].strip() + " " + lines[idx + 1].strip() + " " + lines[
                                idx + 2].strip() + " " + lines[idx + 3].strip()
                            start = matrix_str.find("[[")
                            end = matrix_str.find("]]")
                            elements = matrix_str[start:end + 2].replace("'", "").replace("[", "").replace("]",
                                                                                                           "").split()
                            matrix_arr = np.array(elements, dtype=float).reshape(4, 4)
                            tf_.append(matrix_arr)
                        except IndexError:
                            print("cannot read transformations")
                        idx += 1
                    else:
                        idx += 1
        return tf_

    def transform_pcds(self):
        """
        Apply transformations to all PCD files.
        """

        for pcd, tf in zip(self.pcd_, self.tf_):
            if not isinstance(tf, np.ndarray) or tf.shape != (4, 4):
                print("tf matrix is not formatted properly")
            pcd.transform(tf)

    def register_pcds(self, pcd, scan):
        """
        Register a PCD file into the current scan PCD.

        @param pcd: The current PCD file to be added to the scan.
        @param scan: The current combination of previous PCD files.
        @return: A transformation matrix to align pcd to scan.
        """

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iterations=50)
        pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
        scan_down = scan.voxel_down_sample(voxel_size=0.05)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd_down, scan_down, 0.1, np.eye(4, 4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(criteria=criteria),
        )
        return reg_p2p.transformation

    def run(self):
        """
        Run the node.

        This function will read all PCD files from the input directory, apply the corresponding transformations,
        register, stitch, and filter the PCD files, and then save the result to the input directory.
        """

        self.transform_pcds()
        scan = o3d.geometry.PointCloud()
        for idx, pcd in enumerate(self.pcd_):
            if idx == 0:
                scan += pcd
            else:
                tf = self.register_pcds(pcd, scan)
                pcd.transform(tf)
                scan += pcd


def main():
    """


    """

    parser = argparse.ArgumentParser(description="iterate through viewpoints and collect point clouds")
    parser.add_argument("--path", type=str, required=True, help="path to pcd scan directory")
    args = parser.parse_args()

    ros_args = sys.argv[1:]
    ros_args.extend(["--ros-args", "--param", f"path:={args.path}"])

    rclpy.init(args=ros_args)
    node = PCDRegistration(path=args.path)
    node.run()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
