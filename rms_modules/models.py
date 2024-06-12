
from rms_modules.base import *
import open3d as o3d
import os.path

def get_model_pcd_status(part_name):
    path2models = get_path2models()
    pcd_path = os.path.join(path2models, f"pcd/{part_name}.pcd")

    return os.path.exists(pcd_path)

def get_model_stl_status(part_name):
    path2models = get_path2models()
    pcd_path = os.path.join(path2models, f"stl/{part_name}.stl")

    return os.path.exists(pcd_path)

def generate_pcd_from_stl(part_name):
    path2models = get_path2models()
    read_path = os.path.join(path2models, f"stl/{part_name}.stl")
    write_path = os.path.join(path2models, f"pcd/{part_name}.pcd")
    stl = o3d.io.read_triangle_mesh(read_path)
    pcd = stl.sample_points_uniformly(1000000)
    o3d.io.write_point_cloud(write_path, pcd)
    