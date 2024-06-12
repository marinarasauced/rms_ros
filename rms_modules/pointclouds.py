
from sensor_msgs.msg import PointCloud2, PointField

import ctypes
import datetime
import glob
import open3d as o3d
import math
import numpy as np
import struct
import sys

from rms_modules.base import *

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_pointcloud(cloud, field_names=None, skip_nans=False, uvs=[]):
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

#
def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

#
def unpack_rgb(packed):
    s = struct.pack('>f', packed)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
    
    return [r, g, b]

def get_write_directory(configuration):
    path2scans = get_path2scans()
    curr_date = datetime.datetime.today().strftime("%Y-%m-%d")
    curr_date_path = os.path.join(path2scans, curr_date)
    if not os.path.exists(curr_date_path):
        os.makedirs(curr_date_path)
    folder_max = 0
    for folder in os.listdir(curr_date_path):
        if int(folder[:1]) == configuration and folder[:4].isdigit() and folder[5:6].isdigit() and folder[8:9].isdigit() and folder[11:12].isdigit():
            folder_curr = folder[2:4]
            folder_max = max(folder_max, int(folder_curr))
    folder_max += 1

    next_index = str(folder_max).zfill(3)

    curr_time = datetime.datetime.now().strftime("%H:%M:%S")
    next_folder = f"{configuration}{next_index}-{curr_time}"

    write_dir = os.path.abspath(os.path.join(curr_date_path, next_folder))
    if not os.path.exists(write_dir):
        os.makedirs(write_dir)
            
    return write_dir

def get_pcd_file_paths(scan_path):
    vx250_search = os.path.join(scan_path, "vx250-*.pcd")
    vx300_search = os.path.join(scan_path, "vx300-*.pcd")
    vx250_files = []
    vx300_files = []
    vx250_files.extend(glob.glob(vx250_search))
    vx300_files.extend(glob.glob(vx300_search))

    if not vx250_files and not vx300_files:
        log_error("cannot located pcd scans")

    return vx250_files, vx300_files

def read_pcd_files(pc_paths):
    return [o3d.io.read_point_cloud(pc) for pc in pc_paths]

def downsample_pointclouds(pcs, voxel_size):
    return [pc.voxel_down_sample(voxel_size) for pc in pcs]

def filter_pointclouds_by_z(pcs, z_thresh):
    filtered_pcs = []
    for pc in pcs:
        pc_array = np.array(pc.data)
        filtered_indices = np.where(np.abs(pc_array[:, 2]) < z_thresh)
        filtered_pcs.append(pc.select_by_index(filtered_indices[0]))
    return filtered_pcs

def register_pointclouds(scan_path):
    vx250_files, vx300_files = get_pcd_file_paths(scan_path)
    print(vx250_files)
    print(vx300_files)

    pointclouds = read_pcd_files(vx250_files + vx300_files)

    voxel_size = 0.005
    pointclouds = downsample_pointclouds(pointclouds, voxel_size)

    z_thresh = 1
    pointclouds = filter_pointclouds_by_z(pointclouds, z_thresh)
