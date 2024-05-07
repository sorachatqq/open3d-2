import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("kota_circuit2.ply")
print("Initial number of points:", len(pcd.points))

center = pcd.get_center()
print("Center of the point cloud:", center)

aabb = pcd.get_axis_aligned_bounding_box()
min_bound = aabb.min_bound
max_bound = aabb.max_bound
print("Min bounds of the point cloud:", min_bound)
print("Max bounds of the point cloud:", max_bound)

new_center = (min_bound + max_bound) / 2
print("Adjusted center of the bounding box:", new_center)

half_side_length = 100

bounding_box = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=new_center - half_side_length,
    max_bound=new_center + half_side_length
)
cropped_pcd = pcd.crop(bounding_box)
print("Number of points after cropping:", len(cropped_pcd.points))

if len(cropped_pcd.points) > 0:
    cropped_pcd.estimate_normals()
    o3d.io.write_point_cloud("cropped_Kota_circuit.ply", cropped_pcd, write_ascii=True)
else:
    print("Cropped point cloud is still empty, consider adjusting the bounding box parameters.")
