
import json
import os.path
import subprocess
import sys

def get_path2config():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/"))

def get_path2models():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/models/"))

def get_path2scans():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../scans/"))

def get_rms_files_status():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/waypoints.txt"))

def load_rms_config():
    path2config = get_path2config()
    f_ = open(os.path.join(path2config, "config.json"))
    f_data = json.load(f_)

    return f_data

def unpack_rms_config():
    f_data = load_rms_config()

    return f_data["robot_model"]

def log_task(str):
    print(" >>> [Task:  " + str + "]")

def log_done(str):
    print(" >>> [Done:  " + str + "]\n")

def log_doing(str):
    print(" >>> [Doing: " + str + "]")

def log_error(str):
    print(" >>> [Error: " + str + ", shutting down]\n")
    sys.exit(1)

def run_commands(commands):
    for command in commands:
        log_doing(f"{command}")
        subprocess.run(command, shell=True)