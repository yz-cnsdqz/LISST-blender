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
BODY_MEAN_SHAPE = shape_space['meanshape']
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
            bone_idx = JOINT_NAMES.index(BONE_CHILD_NAMES[mesh_bone.name])
            target_length = bone_lengths[JOINT_NAMES.index(BONE_CHILD_NAMES[mesh_bone.name])]
            # scale = target_length / mesh_bone.length * 100
            scale = target_length / BODY_MEAN_SHAPE[bone_idx]
            mesh_bone.scale = Vector((scale,scale,scale))
        else: continue



def set_rest_pose(armature):
    """set the current pose of the armature as the rest pose
    **** from the addon "simple retarget tool" ****

    Args:
        armature (bpy_types.Object): the target armature
    """

    bpy.ops.object.mode_set(mode='POSE')
    def apply_armature():

        for mod in bpy.context.object.modifiers:

            if mod.type == 'ARMATURE':
                bpy.ops.object.select_all(action='DESELECT')
                mod_name = mod.name
                bpy.ops.object.modifier_copy(modifier=mod_name)
                bpy.ops.object.modifier_apply(modifier=mod_name)
                bpy.context.object.modifiers.active.name = mod_name

    bpy.context.view_layer.objects.active = armature

    for obj in bpy.data.objects:
        if obj.type == 'MESH' and obj.parent == armature:
                
            objectToSelect = bpy.data.objects[obj.name]
            objectToSelect.select_set(True)    
            bpy.context.view_layer.objects.active = objectToSelect
            sourceobj = objectToSelect
            
            if sourceobj.data.shape_keys is None:

                apply_armature()

            else:

                bpy.ops.object.duplicate(linked=False)
                duplicateobj = bpy.context.view_layer.objects.active
                duplicateobj.name="dups"
                sourceobj.shape_key_clear()
                bpy.context.view_layer.objects.active = sourceobj
                
                
                apply_armature()
                
                duplicateobj.select_set(True)        
                
                for idx in range(1, len(duplicateobj.data.shape_keys.key_blocks)):
                    duplicateobj.active_shape_key_index = idx
                    print("Copying Shape Key - ", duplicateobj.active_shape_key.name)
                    bpy.ops.object.shape_key_transfer()
                
                sourceobj.show_only_shape_key = False
                bpy.data.objects.remove(duplicateobj, do_unlink=True)
                
                
                        
    bpy.context.view_layer.objects.active = armature
    bpy.ops.pose.armature_apply(selected=False)
    bpy.ops.object.mode_set(mode='OBJECT')




def create_animation_forward_kinematics(armature, motiondata, duration=60, framerate=30):
    """create the animation using fk

    Args:
        armature (bpy_types.Object): the target armature
        motiondata (dict): the motion data, pickle load from the result.pkl file
        duration (int, optional): duration of the animation. Defaults to 60.
    """
    
    scene = bpy.data.scenes['Scene']
    scene.render.fps = framerate
    joint_rot_data = motiondata['J_rotmat']
    joint_loc_data = motiondata['J_locs_3d']
    scene.frame_start = 0
    scene.frame_end = 10+min(len(joint_rot_data), duration)

    for frame in range(min(len(joint_rot_data), duration)):
        scene.frame_set(frame)
        joint_rotmats = joint_rot_data[frame]
        joint_locs = joint_loc_data[frame]
       
        #set rootbone loc
        rootbone = armature.pose.bones['root']
        rootloc = np.eye(4) 
        #last row and last col = loc
        rootloc[:-1, -1] = joint_locs[0]
        rootloc[:-1, :-1] = joint_rotmats[0]
        rootbone.matrix = Matrix(rootloc)
        bpy.context.view_layer.update()

        for parent, children in CHILDREN_TABLE.items():
            
            for child in children:
                bone_name = BONE_NAMES[(parent, child)]
                if bone_name in ['left_thumb', 'right_thumb', 'left_fingers', 'right_fingers', 'left_hand', 'right_hand', 'right_toes', 'left_toes']:
                    continue
                parent_joint_index = JOINT_NAMES.index(parent)
                # child_joint_index = JOINT_NAMES.index(child)
                if parent == 'lradius':
                   child_joint_index = JOINT_NAMES.index('lhand')
                elif parent == 'rradius':
                    child_joint_index = JOINT_NAMES.index('rhand')
                else:
                    child_joint_index = JOINT_NAMES.index(child)
                
                ## canonical transform
                transf1 = np.eye(4)
                transf1[:-1, :-1] = np.array(armature.pose.bones[bone_name].bone.matrix_local)[:-1,:-1]
                
                ## transform w.r.t. the armature obj coordinate
                transf = np.eye(4)
                transf[:-1,:-1] = joint_rotmats[child_joint_index]
                transf[:-1, -1] = joint_locs[parent_joint_index]
                # if bone_name in ['left_hip', 'right_hip']:
                #     transf[:-1, -1] = joint_locs[0]
                # else:
                #     transf[:-1, -1] = joint_locs[parent_joint_index]
                M = (
                Matrix(transf) @ 
                Matrix(transf1)
                ) 
                armature.pose.bones[bone_name].matrix = M #armature.pose.bones[bone_name].matrix
                
                ##refresh the context
                bpy.context.view_layer.update()
                
        armature.keyframe_insert('location', frame=frame)
        armature.keyframe_insert('rotation_quaternion', frame=frame)
        bones = armature.pose.bones
        for bone in bones:
            if bone in ['left_thumb', 'right_thumb', 'left_fingers', 'right_fingers', 'left_hand', 'right_hand', 'right_toes', 'left_toes']:
                continue
            bone.keyframe_insert('rotation_quaternion', frame=frame)
            bone.keyframe_insert('location', frame=frame)
            bone.keyframe_insert('scale', frame=frame)






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
        bpy.ops.import_scene.fbx(filepath=model_file, force_connect_children=True, use_anim=False)
        
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
        z = 30*np.random.randn(zdim)
        eigvals = SHAPE_EIGVALS[:zdim]
        z = z * eigvals
        bone_lengths = BODY_MEAN_SHAPE + np.einsum('ij,j->i',SHAPE_BASIS[:,:zdim], z)
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
        rescale_bones(BODY_MEAN_SHAPE, obj)

        return {'FINISHED'}


class LISSTAddAnimation(bpy.types.Operator, ImportHelper):
    bl_idname = "object.lisst_add_animation"
    bl_label = "Add Animation"
    bl_description = ("Load LISST motion file and create animated LISST body")
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(
        default="*.pkl",
        options={'HIDDEN'}
    )

    target_framerate: IntProperty(
        name="Target framerate [fps]",
        description="Target framerate for animation in frames-per-second. Lower values will speed up import time.",
        default=30,
        min = 1,
        max = 120
    )

    @classmethod
    def poll(cls, context):
        try:
            # Always enable button
            return True
        except: return False


    def execute(self, context):

        target_framerate = self.target_framerate

        # Load .npz file
        print("Loading: " + self.filepath)
        with open(self.filepath, "rb") as f:
            motiondata = pickle.load(f, encoding="latin1")
        
        if ("r_locs" not in motiondata) or ("J_shape" not in motiondata) or ("J_locs_3d" not in motiondata)or ("J_rotmat" not in motiondata):
                self.report({"ERROR"}, "Invalid LISST motion data file, one/more motion data key(s) missing")
                return {"CANCELLED"}
        duration = motiondata['r_locs'].shape[0]    
        
        # add a new body
        if (context.active_object is not None):
            bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.scene.lisst_add_mesh()
        obj = context.view_layer.objects.active        
        if obj.type == 'ARMATURE':
            armature = obj
            obj = bpy.context.object.children[0]
        else:
            armature = obj.parent
       

        # re-scale the bone length
        rescale_bones(motiondata['J_shape'], armature)
        set_rest_pose(armature)
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.mode_set(mode='OBJECT')
        armature.select_set(state = True, view_layer = bpy.context.view_layer)
        bpy.context.view_layer.objects.active = armature
        bpy.ops.object.rotation_clear()
        
        bpy.ops.object.transform_apply()
        create_animation_forward_kinematics(armature, motiondata, duration, self.target_framerate)
        #optional: align orientation
        # bpy.ops.transform.rotate(value = 3.141592654, orient_axis='Z')
        # bpy.ops.object.transform_apply()
        if "cam_pose" in motiondata:
            cam_poses = motiondata['cam_pose']
            scene = bpy.context.scene
            cam = bpy.data.objects['Camera'] # Adjust if your camera has a different name

            # Set animation length
            scene.frame_end = len(cam_poses)

            # Iterate over each pose matrix
            for frame, pose in enumerate(cam_poses, start=1):
                # Convert numpy array to Blender Matrix
                matrix = Matrix(pose.tolist())
                
                # In Blender, camera points towards its -Z axis and the up direction is the Y axis
                # Depending on your camera poses you might need to adjust for this
                rotate = Matrix.Rotation(np.pi / 2, 4, 'X')
                matrix = matrix @ rotate
                
                # Set the matrix to the camera
                cam.matrix_world = matrix

                # Keyframe each matrix transformation for the animation
                cam.keyframe_insert(data_path='matrix_world', frame=frame)

        return {'FINISHED'}






class LISSTAddAnimationBatch(bpy.types.Operator, ImportHelper):
    bl_idname = "object.lisst_add_animation_batch"
    bl_label = "Add Animation Batch"
    bl_description = ("Load LISST motion file and create a batch of LISST animations")
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(
        default="*.pkl",
        options={'HIDDEN'}
    )

    target_framerate: IntProperty(
        name="Target framerate [fps]",
        description="Target framerate for animation in frames-per-second. Lower values will speed up import time.",
        default=30,
        min = 1,
        max = 120
    )

    @classmethod
    def poll(cls, context):
        try:
            # Always enable button
            return True
        except: return False


    def execute(self, context):

        target_framerate = self.target_framerate

        # Load .npz file
        print("Loading: " + self.filepath)
        with open(self.filepath, "rb") as f:
            motiondata_all = pickle.load(f, encoding="latin1")
        
        if ("r_locs" not in motiondata_all) or ("J_shape" not in motiondata_all) or ("J_locs_3d" not in motiondata_all)or ("J_rotmat" not in motiondata_all):
                self.report({"ERROR"}, "Invalid LISST motion data file, one/more motion data key(s) missing")
                return {"CANCELLED"}
        duration = motiondata_all['r_locs'].shape[0]    
        n_batches = motiondata_all['r_locs'].shape[1]
        
        for b in range(n_batches):
            print('-- processing animation {:03d}'.format(b))
            # add a new body
            if (context.active_object is not None):
                bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.scene.lisst_add_mesh()
            obj = context.view_layer.objects.active        
            if obj.type == 'ARMATURE':
                armature = obj
                obj = bpy.context.object.children[0]
            else:
                armature = obj.parent
        
            # get one batch
            motiondata = {}
            for key, val in motiondata_all.items():
                if key == 'J_shape':
                    motiondata[key] = val[b]
                elif key == 'J_locs_2d':
                    continue
                else:
                    motiondata[key] = val[:,b]
        
            # re-scale the bone length
            rescale_bones(motiondata['J_shape'], armature)
            set_rest_pose(armature)
            bpy.ops.object.select_all(action='DESELECT')
            bpy.ops.object.mode_set(mode='OBJECT')
            armature.select_set(state = True, view_layer = bpy.context.view_layer)
            bpy.context.view_layer.objects.active = armature
            bpy.ops.object.rotation_clear()
            
            bpy.ops.object.transform_apply()
            create_animation_forward_kinematics(armature, motiondata, duration, self.target_framerate)
            #optional: align orientation
            # bpy.ops.transform.rotate(value = 3.141592654, orient_axis='Z')
            # bpy.ops.object.transform_apply()
        

        return {'FINISHED'}






###################### for GUI ##################################

# Property groups for UI
class PG_LISSTProperties(PropertyGroup):

    lisst_mesh: EnumProperty(
        name = "Model",
        description = "LISST base mesh",
        items = [ ("neutral", "Neutral", "") ]
    )


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

        
class LISST_PT_Animation(bpy.types.Panel):
    bl_label = "Animation"
    bl_category = "LISST"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.operator("object.lisst_add_animation")
        col.separator()
        col.operator("object.lisst_add_animation_batch")
        col.separator()



classes = [
    PG_LISSTProperties,
    LISSTAddMesh,
    LISSTRandomShape,
    LISSTResetShape,
    LISSTAddAnimation,
    LISSTAddAnimationBatch,
    LISST_PT_Model,
    LISST_PT_Shape,
    LISST_PT_Animation,
]




def register():
    from bpy.utils import register_class
    for cls in classes:
        bpy.utils.register_class(cls)

    # Store properties under WindowManager (not Scene) so that they are not saved in .blend files and always show default values after loading
    bpy.types.WindowManager.lisst_tool = PointerProperty(type=PG_LISSTProperties)

def unregister():
    from bpy.utils import unregister_class
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.WindowManager.lisst_tool


if __name__ == "__main__":
    register()
