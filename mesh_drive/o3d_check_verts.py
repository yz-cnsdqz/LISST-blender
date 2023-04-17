import open3d as o3d
import numpy as np
name = 313
file = f"C:\\Users\\hshang\\Downloads\\verts\\file1\\CoreView_{name}_faces.npy"
# file = f'C:\\Users\\hshang\\Downloads\\verts\\mocap_zju\\CoreView_{name}_verts.npy'
# file = r'C:\Users\hshang\Downloads\KeypointNeRF\data\zju_mocap\CoreView_394\joints3d\1.npy'



# file = r"C:\Users\hshang\Downloads\params\verts\vertices.npy"

verts = np.load(file, allow_pickle=True)
print(f"we are testing the file {name}")
print(verts.shape)

# vis = o3d.visualization.Visualizer()
# vis.create_window(width=800, height=600) 

# # geometry is the point cloud used in your animaiton
# geometry = o3d.geometry.PointCloud()

# for i in range(len(verts)):
#     # now modify the points of your geometry
#     # you can use whatever method suits you best, this is just an example
#     geometry.points = o3d.utility.Vector3dVector(verts[i])
#     if i == 0:    
#         vis.add_geometry(geometry)
#     else:
#         vis.update_geometry(geometry)
#     vis.poll_events()
#     vis.update_renderer()