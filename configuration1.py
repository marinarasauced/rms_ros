import os.path
import sys

from rms_modules.manipulators import VXManipulator


def main():
    """
    Run an RMS configuration one trial.

    This executable runs the RMS single scan configuration. If the manipulator is online, it will cycle through all
    viewpoints, sample point clouds, return to the home position, go to sleep, register all scans, and then compare them
    to the model. Before running this executable, ensure that the part name is specified correctly and that it has been
    converted to a PCD file from its STL source file.

    """

    part_id = "test_object_b_scaled"
    primary = VXManipulator("primary")
    # secondary = VXManipulator("secondary")


if __name__ == "__main__":
    main()
