import numpy as np
import bpy

verts_all = []
faces_all = []

armature_obj = bpy.context.active_object
if armature_obj.type != 'ARMATURE':
    raise ValueError("Active object is not an armature!")
anim_data = armature_obj.animation_data
start_frame = end_frame = None
for fcurve in anim_data.action.fcurves:
    for keyframe in fcurve.keyframe_points:
        if start_frame is None or keyframe.co[0] < start_frame:
            start_frame = int(keyframe.co[0])
        if end_frame is None or keyframe.co[0] > end_frame:
            end_frame = int(keyframe.co[0])

bpy.context.scene.frame_start = start_frame
bpy.context.scene.frame_end = end_frame
armature_name = armature_obj.name

for obj in bpy.context.scene.objects:
    for mod in obj.modifiers:
        if mod.type == 'ARMATURE' and mod.object == armature_obj:
            mesh_obj = obj
            break

if 'mesh_obj' in locals():
    # mesh_obj = mesh_obj.copy()
    # mesh_obj.data = mesh_obj.data.copy()
    # bpy.context.collection.objects.link(mesh_obj)
    # bpy.context.view_layer.objects.active = mesh_obj
    # bpy.ops.object.select_all(action='DESELECT')
    # mesh_obj.select_set(True)
    # bpy.ops.object.modifier_apply({"object": mesh_obj}, modifier=mod.name)
    bpy.context.view_layer.objects.active = mesh_obj
    for frame in range(bpy.data.scenes['Scene'].frame_start, bpy.data.scenes['Scene'].frame_end+1):
        bpy.data.scenes['Scene'].frame_set(frame)
        dg = bpy.context.evaluated_depsgraph_get()
        obj = bpy.context.object.evaluated_get(dg)
        mesh = obj.to_mesh(preserve_all_data_layers=True, depsgraph=dg)
        verts = mesh.vertices
        verts_locs = np.array([obj.matrix_world @ x.co for x in verts])
        verts_all.append(verts_locs)

        faces = mesh.polygons
        faces_indices = np.array([face.vertices[:] for face in faces])
        faces_all.append(faces_indices)

    verts_all = np.stack(verts_all)
    faces_all = np.stack(faces_all)

    print(verts_all.shape)
    print(faces_all.shape)

    verts_outfilename = f'C:\\Users\\hshang\\Downloads\\verts\\file1\\{armature_name}_verts.npy'
    faces_outfilename = f'C:\\Users\\hshang\\Downloads\\verts\\file1\\{armature_name}_faces.npy'

    np.save(verts_outfilename, verts_all)
    np.save(faces_outfilename, faces_all)
else:
    raise ValueError("No mesh object found that is parented to the armature.")