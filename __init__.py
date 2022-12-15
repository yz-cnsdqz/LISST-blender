# This script is developped based on the SMPL-X add-on: https://gitlab.tuebingen.mpg.de/jtesch/smplx_blender_addon/-/blob/master/__init__.py

# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####



import os, sys, pickle
import bpy
import sys
import numpy as np
import math
from mathutils import Vector, Matrix, Quaternion
from bpy_extras.io_utils import ImportHelper,ExportHelper # ImportHelper/ExportHelper is a helper class, defines filename and invoke() function which calls the file selector.
from mathutils import Vector, Quaternion
from bpy.props import ( BoolProperty, EnumProperty, FloatProperty, IntProperty, PointerProperty, StringProperty )
from bpy.types import ( PropertyGroup )


bl_info = {
    "name": "LISST-Blender",
    "author": "Haoliang Shang and Yan Zhang, ETH Zurich",
    "version": (2022, 12, 15),
    "blender": (2, 80, 0),
    "location": "Viewport > Right panel",
    "description": "LISST-Blender",
    "wiki_url": "",
    "category": "LISST"}


'''Armature properties'''
JOINT_NAMES = [
'root', 
'lhipjoint', 
'lfemur', 
'ltibia', 
'lfoot', 
'ltoes', 
'rhipjoint', 
'rfemur', 
'rtibia', 
'rfoot', 
'rtoes', 
'lowerback', 
'upperback', 
'thorax', 
'lowerneck', 
'upperneck', 
'head', 
'lclavicle', 
'lhumerus', 
'lradius', 
'lwrist', 
'lhand', 
'lfingers', 
'lthumb', 
'rclavicle', 
'rhumerus', 
'rradius', 
'rwrist', 
'rhand', 
'rfingers', 
'rthumb'
]




CHILDREN_TABLE = {
'root': ['lhipjoint', 'rhipjoint', 'lowerback'],
 'lhipjoint': ['lfemur'], 
 'lfemur': ['ltibia'], 
 'ltibia': ['lfoot'], 
 'lfoot': ['ltoes'],
 'ltoes': [], 
 'rhipjoint': ['rfemur'], 
 'rfemur': ['rtibia'], 
 'rtibia': ['rfoot'], 
 'rfoot': ['rtoes'], 
 'rtoes': [], 
 'lowerback': ['upperback'], 
 'upperback': ['thorax'], 
 'thorax': ['lowerneck', 'lclavicle', 'rclavicle'], 
 'lowerneck': ['upperneck'], 
 'upperneck': ['head'], 
 'head': [], 
 'lclavicle': ['lhumerus'], 
 'lhumerus': ['lradius'], 
 'lradius': ['lwrist'], 
 'lwrist': ['lhand', 'lthumb'], 
 'lhand': ['lfingers'], 
 'lfingers': [], 
 'lthumb': [], 
 'rclavicle': ['rhumerus'], 
 'rhumerus': ['rradius'], 
 'rradius': ['rwrist'], 
 'rwrist': ['rhand', 'rthumb'], 
 'rhand': ['rfingers'], 
 'rfingers': [], 
 'rthumb': []
 }


BONE_NAMES = {
('root', 'lhipjoint'): 'left_hip', 
('root', 'rhipjoint'): 'right_hip', 
('root', 'lowerback'): 'spine1',
('lhipjoint', 'lfemur'): 'left_knee', 
('lfemur', 'ltibia'): 'left_ankle', 
('ltibia', 'lfoot'): 'left_foot', 
('lfoot', 'ltoes'): 'left_toes', 
('rhipjoint', 'rfemur'): 'right_knee', 
('rfemur', 'rtibia'): 'right_ankle', 
('rtibia', 'rfoot'): 'right_foot', 
('rfoot', 'rtoes'): 'right_toes', 
('lowerback', 'upperback'): 'spine2', 
('upperback', 'thorax'): 'spine3', 
('thorax', 'lowerneck'): 'neck', 
('thorax', 'lclavicle'): 'left_collar', 
('thorax', 'rclavicle'): 'right_collar', 
('lowerneck', 'upperneck'): 'head1', 
('upperneck', 'head'): 'head2', 
('lclavicle', 'lhumerus'): 'left_shoulder', 
('lhumerus', 'lradius'): 'left_elbow', 
('lradius', 'lwrist'): 'left_wrist', 
('lwrist', 'lhand'): 'left_hand', 
('lwrist', 'lthumb'): 'left_thumb', 
('lhand', 'lfingers'): 'left_fingers', 
('rclavicle', 'rhumerus'): 'right_shoulder', 
('rhumerus', 'rradius'): 'right_elbow', 
('rradius', 'rwrist'): 'right_wrist', 
('rwrist', 'rhand'): 'right_hand', 
('rwrist', 'rthumb'): 'right_thumb', 
('rhand', 'rfingers'): 'right_fingers'
}

#used for rescaling
BONE_CHILD_NAMES = {
'left_hip':'lhipjoint' , 
'right_hip': 'rhipjoint', 
'spine1': 'lowerback',
'left_knee': 'lfemur', 
'left_ankle': 'ltibia', 
'left_foot': 'lfoot', 
'left_toes': 'ltoes', 
'right_knee': 'rfemur', 
'right_ankle': 'rtibia', 
'right_foot': 'rfoot', 
'right_toes': 'rtoes', 
'spine2': 'upperback', 
'spine3': 'thorax', 
'neck': 'lowerneck', 
'left_collar': 'lclavicle', 
'right_collar': 'rclavicle', 
'head1': 'upperneck', 
'head2': 'head', 
'left_shoulder': 'lhumerus', 
'left_elbow': 'lradius', 
'left_wrist': 'lwrist', 
'left_hand': 'lhand', 
'left_thumb': 'lthumb', 
'left_fingers': 'lfingers', 
'right_shoulder': 'rhumerus', 
'right_elbow': 'rradius', 
'right_wrist': 'rwrist', 
'right_hand': 'rhand', 
'right_thumb': 'rthumb', 
'right_fingers': 'rfingers'
}




ROT_Z_180 = Matrix.Rotation(math.radians(180), 4, 'Z')
ROT_X_NEG90 = Matrix.Rotation(math.radians(-90), 4, 'X')
ROT_TO_BLENDER = ROT_X_NEG90 @ ROT_Z_180



# load shape space
path = os.path.dirname(os.path.realpath(__file__))
shape_space = np.load(os.path.join(path, "data", "LISST_shape_space.pkl"), allow_pickle=True)
JOINT_DEFAULT_ORIENTATION = shape_space['template_joint_directions']
BDOY_MEAN_SHAPE = shape_space['meanshape']
SHAPE_BASIS=shape_space['shape_basis']
SHAPE_EIGVALS = shape_space['eigvals']




def rescale_bones(bone_lengths, mesh_armature):
    """rescale the armature to the correct bone_lengths

    Args:
        bone_lengths (numpy.ndarray): bone length data, the "motiondata['J_shape']"
        mesh_armature (bpy_types.Object): the target armature, should have a mesh skinned to it
    """
    
    #no hands, fingers, toes
    INVOLVED_BONES = ['left_hip', 'right_hip', 'left_knee', 'left_ankle', 'left_foot', 'right_knee', 'right_ankle', 'right_foot', 'spine2', 'spine3', 'neck', 
        'left_collar', 'right_collar', 'head1', 'head2', 'left_shoulder', 'left_elbow', 'left_wrist', 
        'right_shoulder', 'right_elbow', 'right_wrist', ]
    #clear inherit scale
    for bone in mesh_armature.data.bones:
        if bone.name in INVOLVED_BONES:
            bone.inherit_scale = 'NONE'
        else: continue
    
    #rescale
    for mesh_bone in mesh_armature.pose.bones:
        if mesh_bone.name in INVOLVED_BONES:
            target_length = bone_lengths[JOINT_NAMES.index(BONE_CHILD_NAMES[mesh_bone.name])]
            scale = target_length / mesh_bone.length * 100
            mesh_bone.scale = Vector((scale,scale,scale))
        else: continue





###############################################################################



class LISSTAddMesh(bpy.types.Operator):
    bl_idname = "scene.lisst_add_mesh"
    bl_label = "Add"
    bl_description = ("Add LISST base mesh model to scene")
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        try:
            # Enable button only if in Object Mode
            if (context.active_object is None) or (context.active_object.mode == 'OBJECT'):
                return True
            else: 
                return False
        except: return False

    def execute(self, context):
        
        path = os.path.dirname(os.path.realpath(__file__))

        # import the fbx file of the LISST base mesh, which has the mean shape skeleton
        model_file = os.path.join(path, "data", "LISST_canonical_mesh.fbx")
        bpy.ops.import_scene.fbx(filepath=model_file)
        
        # Select imported armature
        object_name = context.selected_objects[0].name
        bpy.ops.object.select_all(action='DESELECT')
        context.view_layer.objects.active = bpy.data.objects[object_name]
        bpy.data.objects[object_name].select_set(True)

        return {'FINISHED'}



class LISSTRandomShape(bpy.types.Operator):
    bl_idname = "object.lisst_random_shape"
    bl_label = "Random"
    bl_description = ("generate random bone lengths from the shape PCA shape, and rescale the body mesh")
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        try:
            # Enable button only if armature is active object
            return context.object.type == 'ARMATURE'
        except: return False

    def execute(self, context):
        # get the armature
        obj = bpy.context.object
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # random draw a bone length
        zdim = 15
        z = np.random.randn(zdim)
        eigvals = SHAPE_EIGVALS[:zdim]
        z = z * eigvals
        bone_lengths = BDOY_MEAN_SHAPE + np.einsum('ij,j->i',SHAPE_BASIS[:,:zdim], z)
        rescale_bones(bone_lengths, obj)

        return {'FINISHED'}




class LISSTResetShape(bpy.types.Operator):
    bl_idname = "object.lisst_reset_shape"
    bl_label = "Reset"
    bl_description = ("reset the body mesh to the mean shape bone lengths")
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        try:
            # Enable button only if armature is active object
            return context.object.type == 'ARMATURE'
        except: return False

    def execute(self, context):
        # get the armature
        obj = bpy.context.object
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # set the bone length
        rescale_bones(BDOY_MEAN_SHAPE, obj)

        return {'FINISHED'}



class LISST_PT_Model(bpy.types.Panel):
    bl_label = "LISST Model"
    bl_category = "LISST"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):

        layout = self.layout
        col = layout.column(align=True)
        
        row = col.row(align=True)
        col.prop(context.window_manager.lisst_tool, "lisst_add_mesh")
        col.operator("scene.lisst_add_mesh", text="Add")


class LISST_PT_Shape(bpy.types.Panel):
    bl_label = "Shape"
    bl_category = "LISST"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)

        row = col.row(align=True)
        split = row.split(factor=0.75, align=True)
        split.operator("object.lisst_random_shape")
        split.operator("object.lisst_reset_shape")
        col.separator()

        

class SMPLX_PT_Pose(bpy.types.Panel):
    bl_label = "Pose"
    bl_category = "SMPL-X"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)

        col.prop(context.window_manager.lisst_tool, "smplx_corrective_poseshapes")
        col.separator()
        col.operator("object.smplx_set_poseshapes")

        col.separator()
        col.label(text="Hand Pose:")
        row = col.row(align=True)
        split = row.split(factor=0.75, align=True)
        split.prop(context.window_manager.lisst_tool, "smplx_handpose")
        split.operator("object.smplx_set_handpose", text="Set")

        col.separator()
        col.operator("object.smplx_write_pose")
        col.separator()
        col.operator("object.smplx_load_pose")



class SMPLX_PT_Animation(bpy.types.Panel):
    bl_label = "Animation"
    bl_category = "SMPL-X"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.operator("object.smplx_add_animation")



class SMPLX_PT_Export(bpy.types.Panel):
    bl_label = "Export"
    bl_category = "SMPL-X"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)

        col.operator("object.smplx_export_alembic")
        col.separator()

        col.operator("object.smplx_export_fbx")
        col.separator()

#        export_button = col.operator("export_scene.obj", text="Export OBJ [m]", icon='EXPORT')
#        export_button.global_scale = 1.0
#        export_button.use_selection = True
#        col.separator()

        row = col.row(align=True)
        row.operator("ed.undo", icon='LOOP_BACK')
        row.operator("ed.redo", icon='LOOP_FORWARDS')
        col.separator()

        (year, month, day) = bl_info["version"]
        col.label(text="Version: %s-%s-%s" % (year, month, day))




def register():
    from bpy.utils import register_class
    for cls in classes:
        bpy.utils.register_class(cls)

    # Store properties under WindowManager (not Scene) so that they are not saved in .blend files and always show default values after loading
    bpy.types.WindowManager.lisst_tool = PointerProperty(type=PG_SMPLXProperties)

def unregister():
    from bpy.utils import unregister_class
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.WindowManager.lisst_tool


if __name__ == "__main__":
    register()
