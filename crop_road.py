import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("kota_circuit2.ply")

if not pcd.colors:
    raise ValueError("Point cloud does not have color information.")

road_color_bounds = np.array([[0.62, 0.62, 0.62], [0.9, 0.9, 0.9]])

road_mask = np.all((np.asarray(pcd.colors) >= road_color_bounds[0]) & (np.asarray(pcd.colors) <= road_color_bounds[1]), axis=1)
road_pcd = pcd.select_by_index(np.where(road_mask)[0], invert=False)

mean_z = np.mean(np.asarray(road_pcd.points)[:, 2])

z_tolerance = 11

elevation_mask = np.abs(np.asarray(road_pcd.points)[:, 2] - mean_z) <= z_tolerance

final_road_pcd = road_pcd.select_by_index(np.where(elevation_mask)[0], invert=False)

o3d.io.write_point_cloud("final_road_level.ply", final_road_pcd)

o3d.visualization.draw_geometries([final_road_pcd], window_name="Final Road Level Point Cloud")
