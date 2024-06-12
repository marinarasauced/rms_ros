
import argparse
import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from rms_modules.base import *
from rms_modules.manipulators import *
from rms_modules.models import *
from rms_modules.run_case1 import run_case1
from rms_modules.run_case2 import run_case2
from rms_modules.run_case3 import run_case3

def main():
    parser = argparse.ArgumentParser(description="run shell commands")
    parser.add_argument("--rms_configuration", type=int, default=0, help="rms configuration | 1: single scan; 2: double scan; 3: single grip single scan", required=True)
    parser.add_argument("--part_name", type=str, default="model", help="name of part being scanned & its reference model", required=True)
    args = parser.parse_args()

    if args.rms_configuration == 1:
        run_case1(args)
    elif args.rms_configuration == 2:
        run_case2(args)
    elif args.rms_configuration == 3:
        run_case3(args)
    else:
        print("invalid RMS case, shutting down")


if __name__=="__main__":
    main()
