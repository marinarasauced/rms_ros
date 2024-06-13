
from sensor_msgs.msg import PointCloud2, PointField

import copy
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
        if folder.startswith(str(configuration)):
            try:
                # Extract the numerical index part from the folder name
                folder_index = int(folder[1:4])
                folder_max = max(folder_max, folder_index)
            except ValueError:
                # Ignore folders that do not match the expected pattern
                continue

    next_index = str(folder_max + 1).zfill(3)

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

def read_pcd_transformations(scan_path):
    vx250_search = os.path.join(scan_path, "vx250_transformations.txt")
    vx300_search = os.path.join(scan_path, "vx300_transformations.txt")
    vx250_files = []
    vx300_files = []
    vx250_files.extend(glob.glob(vx250_search))
    vx300_files.extend(glob.glob(vx300_search))

    transformations_path = vx250_files + vx300_files
    transformations = []
    for path in transformations_path:
        with open(path, 'r') as file:
            lines = file.readlines()
            iter = 0
            while iter < len(lines):
                if ':' in lines[iter]:
                    try:
                        matrix_str = lines[iter+0].strip() + ' ' + lines[iter+1].strip() + ' ' + lines[iter+2].strip() + ' ' + lines[iter+3].strip()
                        start_idx = matrix_str.find('[[')
                        end_idx = matrix_str.find(']]')
                        elements = matrix_str[start_idx:end_idx + 2].replace("'", "").replace('[', '').replace(']', '').split()
                        matrix_array = np.array(elements, dtype=float).reshape(4, 4)
                        transformations.append(matrix_array)
                    except IndexError:
                        log_error("cannot read transformations")
                    iter += 1
                else:
                    iter += 1
    return transformations

def read_pcd_files(pc_paths):
    return [o3d.io.read_point_cloud(pc) for pc in pc_paths]

def downsample_pointclouds(pcs, voxel_size):
    return [pc.voxel_down_sample(voxel_size) for pc in pcs]

def filter_pointclouds_by_z(pcs, z_thresh):
    filtered_pcs = []
    for pc in pcs:
        pc_array = np.asarray(pc.points)
        filtered_indices = np.where(np.abs(pc_array[:, 2]) < z_thresh)
        filtered_pcs.append(pc.select_by_index(filtered_indices[0]))
    return filtered_pcs

def filter_pointclouds_by_removing_outliers(pcs):
    filtered_pcs = []
    for pc in pcs:
        _, ind = pc.remove_statistical_outlier(nb_neighbors=100, std_ratio=0.2)
        pc = pc.select_by_index(ind)
        filtered_pcs.append(pc)
    return filtered_pcs

def transform_pointclouds(pcs, tfs):
    for pc, tf in zip(pcs, tfs):
        if not isinstance(tf, np.ndarray) or tf.shape != (4, 4):
            log_error("tf matrix is not formatted properly")
        pc.transform(tf)
    return pcs

def estimate_normals(pcs, r):
    for pc in pcs:
        pc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamRadius(radius=r))
    return pcs

def global_registration(source, target, voxel_size):
    distance_threshold = voxel_size*2
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=100))

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
        4,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.00001)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))

    return result.transformation

def register_point_clouds_point_to_plane(source, target, threshold=0.001):
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=10000, relative_fitness=1e-6, relative_rmse=1e-6)
    loss = o3d.pipelines.registration.TukeyLoss(k=0.1)

    reg_gicp = o3d.pipelines.registration.registration_generalized_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(), criteria)
    print(f"Fitness: {reg_gicp.fitness}")
    print(f"Inlier RMSE: {reg_gicp.inlier_rmse}")
    # print(f"Number of Iterations: {reg_gicp.iteration_num}")
    return reg_gicp.transformation

def multiscale_gicp(source, target, voxel_sizes, threshold):
    current_transformation = np.identity(4)
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=10000, relative_fitness=1e-6, relative_rmse=1e-6)
    for voxel_size in voxel_sizes:
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)
        
        source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        
        result_icp = o3d.pipelines.registration.registration_generalized_icp(
            source_down, target_down, threshold, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),criteria)
        
        current_transformation = result_icp.transformation
        print(f"GICP at voxel size {voxel_size} - Fitness: {result_icp.fitness}, Inlier RMSE: {result_icp.inlier_rmse}")
    
    return current_transformation

def segment_floor(point_cloud, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                                     ransac_n=ransac_n,
                                                     num_iterations=num_iterations)
    floor_cloud = point_cloud.select_by_index(inliers)
    object_cloud = point_cloud.select_by_index(inliers, invert=True)
    return floor_cloud, object_cloud

def smooth_point_cloud(point_cloud, search_radius=0.1, num_iterations=10):
    smoothed_cloud = point_cloud.filter_smooth_simple(search_radius)
    return smoothed_cloud

def extract_largest_cluster(point_cloud, eps=0.02, min_points=10000):
    labels = np.array(point_cloud.cluster_dbscan(eps=eps, min_points=min_points))
    max_label = labels.max()
    largest_cluster = point_cloud.select_by_index(np.where(labels == max_label)[0])
    return largest_cluster


def prepare_pcds_stitch(voxel_size, model, scan):

        # Load pointclouds with identity initial alignment guess:
        tf_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        model.transform(tf_init)

        # Preprocess pointclouds:
        model_down, model_fpfh = preprocess_pcd_stitch(model, voxel_size)
        scan_down, scan_fpfh = preprocess_pcd_stitch(scan, voxel_size)

        return model, scan, model_down, scan_down, model_fpfh, scan_fpfh


def preprocess_pcd_stitch(pcd, voxel_size):

        # Downsample pcd to voxel size:
        pcd_down = pcd.voxel_down_sample(voxel_size)

        # Compute normal with search radius:
        radius_normal = voxel_size * 2
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        # Compute FPFH feature with search radius:
        radius_feature = voxel_size * 5
        pcd_fpfh =  o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
        return pcd_down, pcd_fpfh

def register_pcds_stitch(model_down, scan_down, model_fpfh, scan_fpfh, voxel_size):

        # Registration parameters:
        distance_threshold = voxel_size * 1.5

        # RANSAC registration:
        result =  o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            model_down, scan_down, model_fpfh, scan_fpfh,  True, distance_threshold,
             o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            4, [
                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
            ],  o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))

        return result

def register_pointclouds(scan_path):
    vx250_files, vx300_files = get_pcd_file_paths(scan_path)
    pointclouds = read_pcd_files(vx250_files + vx300_files)

    z_thresh = 1
    pointclouds = filter_pointclouds_by_z(pointclouds, z_thresh)
    pointclouds = filter_pointclouds_by_removing_outliers(pointclouds)

    voxel_size = 0.008

    transformations = read_pcd_transformations(scan_path)
    pointclouds = transform_pointclouds(pointclouds, transformations)
    pointclouds = estimate_normals(pointclouds, 0.1)

    pointcloud = pointclouds[0]
    for pc in pointclouds[1:]:
        log_doing("registering pointclouds")
        model, scan, model_down, scan_down, model_fpfh, scan_fpfh = prepare_pcds_stitch(voxel_size, pointcloud, pc)
        tf_ransac = register_pcds_stitch(scan_down, model_down, scan_fpfh, model_fpfh, voxel_size)
#        o3d.visualization.draw_geometries([model_down + scan_down])
#        o3d.visualization.draw_geometries([model_down + scan_down.transform(tf_ransac.transformation)])
#        transformed_model = copy.deepcopy(model)
        transformed_scan = copy.deepcopy(scan)
        transformed_scan.transform(tf_ransac.transformation)
#        o3d.visualization.draw_geometries([transformed_model + transformed_scan])
        pointcloud += transformed_scan
#        o3d.visualization.draw_geometries([pointcloud])

    o3d.visualization.draw_geometries([pointcloud])
    o3d.io.write_point_cloud(os.path.join(scan_path, "scan.pcd"), pointcloud)