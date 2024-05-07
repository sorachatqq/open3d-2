import open3d as o3d

# Load the mesh
mesh = o3d.io.read_triangle_mesh("public/kota_circuit2.ply")
if not mesh.has_vertices() or not mesh.has_triangles():
    raise ValueError("Invalid mesh: missing vertices or triangles.")

# Crop the mesh
center = mesh.get_center()
side_length = 50
min_bound = center - side_length
max_bound = center + side_length
crop_volume = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
cropped_mesh = mesh.crop(crop_volume)

# Compute vertex normals
cropped_mesh.compute_vertex_normals()

# Write the cropped mesh to a file
o3d.io.write_triangle_mesh("cropped_mesh.ply", cropped_mesh, write_ascii=True)

# Select points based on conditions
selected_points = []
for i in range(len(cropped_mesh.vertices)):
    vertex = cropped_mesh.vertices[i]
    if vertex[2] >= 0.9 and -0.2 <= vertex[0] <= 0.2 and -0.2 <= vertex[1] <= 0.2:
        selected_points.append(i)
selected_mesh = cropped_mesh.select_by_index(selected_points)

# Write the selected points mesh to a file
o3d.io.write_triangle_mesh("selected_points.ply", selected_mesh, write_ascii=True)

# Read and visualize the selected points mesh
selected_mesh = o3d.io.read_triangle_mesh("selected_points.ply")
o3d.visualization.draw_geometries([selected_mesh])
