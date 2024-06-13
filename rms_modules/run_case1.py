
from rms_modules.base import *
from rms_modules.manipulators import *
from rms_modules.models import *
from rms_modules.pointclouds import get_write_directory, register_pointclouds

def run_case1(args):

    #
    print("\n#####################################################\n")
    print(f"Starting RMS Configuration One")
    print("\n#####################################################\n")

    robot_model = unpack_rms_config()
    part_name = args.part_name

    # Check RMS manipulators status:
    log_task("checking RMS manipulators status")
    vx250status, vx300status = get_vx_node_status()
    if robot_model == "vx250":
        if not vx250status:
            log_error("vx250 is offline")
        else:
            log_done("vx250 is online")
    elif robot_model == "vx300":
        if not vx300status:
            log_error("vx300 is offline")
        else:
            log_done("vx300 is online")

    # Check RMS files status:
    log_task("checking for RMS files")
    if not get_rms_files_status():
        log_error("missing RMS waypoints.txt")
    else:
        log_done("found RMS waypoints.txt")

    # Check model.pcd status:
    log_task(f"checking for {part_name}.pcd")
    if not get_model_pcd_status(part_name):
        log_doing(f"{part_name}.pcd not found, checking for {part_name}.stl")
        if not get_model_stl_status(part_name):
            log_error(f"no {part_name}.pcd or {part_name}.stl found in rms_ros/config/models/")
        else:
            log_done(f"found {part_name}.stl")
            log_task(f"writing {part_name}.pcd")
            generate_pcd_from_stl(part_name)
            log_done(f"wrote {part_name}.pcd")
    else:
        log_done(f"found {part_name}.pcd")

    # Commands list:
    log_task("generating write directory")
    write_directory = get_write_directory(1)
    log_done(f"generated write directory at {write_directory}")

    log_task("collecting pointclouds at waypoints")
    subprocess.run(f"python3 ~/rms_ros/src/rms_ros/rms_nodes/src/rms_cycle.py --configuration 1 --write_directory {write_directory}", shell=True)
    log_done("collected pointclouds")

    log_task("going to sleep")
    go2sleep()
    log_done("went to sleep")

    log_task("registering pointclouds")
    register_pointclouds(write_directory)
    log_done("wrote registered pointcloud")