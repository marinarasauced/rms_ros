
import os.path
import sys

from rms_modules.manipulators import *








def main():
    """
    Run a RMS configuration one trial.

    This executable runs the RMS single scan configuration. If the manipulator is online, it will cycle through all viewpoints, sample pointclouds, return to the home position, go to sleep, register all scans, and then compare them to the model.

    """

    primary = VXManipulator("primary")
    print("test")
     

if __name__=="__main__":
    main()