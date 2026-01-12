import open3d as o3d
import numpy as np

# Load the mesh
mesh = o3d.io.read_triangle_mesh("slider.stl")

# Make sure normals exist
mesh.compute_vertex_normals()
mesh.scale(0.001, center=(0,0,0))   

# Sample points from the surface
pc = mesh.sample_points_uniformly(number_of_points=500000)

# (Optional) Voxel downsample for RViz performance
pc = pc.voxel_down_sample(voxel_size=0.002)  # 2mm

# Save as PCD
o3d.io.write_point_cloud("slider.pcd", pc)

