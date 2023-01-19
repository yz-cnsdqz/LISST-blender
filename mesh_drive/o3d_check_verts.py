import open3d as o3d
import numpy as np

file = r"C:\Users\hshang\Downloads\CoreView_390_verts.npy"

verts = np.load(file)
print(verts.shape)

vis = o3d.visualization.Visualizer()
vis.create_window()

# geometry is the point cloud used in your animaiton
geometry = o3d.geometry.PointCloud()


for i in range(len(verts)):
    # now modify the points of your geometry
    # you can use whatever method suits you best, this is just an example
    geometry.points = o3d.utility.Vector3dVector(verts[i])
    if i == 0:    
        vis.add_geometry(geometry)
    else:
        vis.update_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()