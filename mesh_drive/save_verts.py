"""This script outputs the point cloud of the body mesh in each frame
"""
import numpy as np
import bpy
# before running this script, select the body mesh first!!!
verts_all = []

for frame in range(bpy.data.scenes['Scene'].frame_start, bpy.data.scenes['Scene'].frame_end+1):
    bpy.data.scenes['Scene'].frame_set(frame)
    dg = bpy.context.evaluated_depsgraph_get()
    obj = bpy.context.object.evaluated_get(dg)
    mesh = obj.to_mesh(preserve_all_data_layers=True, depsgraph=dg)
    verts = mesh.vertices
    verts_locs = np.array([obj.matrix_world @ x.co for x in verts])
    verts_all.append(verts_locs)
    
verts_all = np.stack(verts_all)

print(verts_all.shape)

#outfilename = '/home/yzhang/Desktop/verts_all.npy'
outfilename = r'C:\Users\hshang\Downloads\CoreView_394_verts.npy'

np.save(outfilename, verts_all)