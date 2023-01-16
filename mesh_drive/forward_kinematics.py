import sys
sys.path.append("C:\LISST-blender\mesh_drive")
import pickle
import os
import bpy
import sys
import numpy as np
import math
from mathutils import Vector, Matrix, Quaternion
from armature_properties import *



# Input pkl path
# INPUT_FILE_PATH = "C:\\Users\\Lukas\\Projects\\lisst-motion-visualization\\input\\motion_0.pkl"
# INPUT_FILE_PATH = "/home/yzhang/workspaces/LISST/results/src/LISST_SHAPER_v0/results/mocap_zju_2/results.pkl"

# INPUT_FILE_PATH = "/home/yzhang/workspaces/LISST/results/src/LISST_SHAPER_v2/results/mocapgo_tmp2/results.pkl"
# INPUT_FILE_PATH = r"C:\Users\zhang\Downloads\results.pkl"
INPUT_FILE_PATH = r"C:\Users\hshang\Downloads\results.pkl"
# Animation properties
FPS_TARGET = 30



'''global placeholders'''
canonical_joint_locations = {}
canonical_bone_matrixes = {}
joint_parent_dict = {}
relative_rotation_around_parent = {}

ROT_Z_180 = Matrix.Rotation(math.radians(180), 4, 'Z')
ROT_X_NEG90 = Matrix.Rotation(math.radians(-90), 4, 'X')
ROT_TO_BLENDER = ROT_X_NEG90 @ ROT_Z_180

'''create/reset the armature'''
def create_armature(bone_lengths, name):
    """create a blender amature that has the specified bone_lengths and name

    Args:
        bone_lengths (numpy.ndarray): bone length data, the "motiondata['J_shape']"
        name (string): name

    Returns:
        bpy_types.Object: the armature
    """
    armature = bpy.data.armatures.new("Armature_"+name)
    armature_object = bpy.data.objects.new(name, armature)
    bpy.context.scene.collection.objects.link(armature_object)
    bpy.context.view_layer.objects.active = armature_object
    armature_object.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    rootbone = armature.edit_bones.new('root')
    rootbone.head = (0, 0, 0)
    rootbone.tail = rootbone.head + Vector([0, 0, 0.01])
    joint_parent_dict['root'] = rootbone
    canonical_joint_locations['root'] = Vector([0, 0, 0])
    joint_orientations = dict(zip(JOINT_NAMES, JOINT_DEFAULT_ORIENTATION))
    
    for parent, children in CHILDREN_TABLE.items():
        for child in children:
            bone_name = BONE_NAMES[(parent, child)]
            bone_length_current = bone_lengths[JOINT_NAMES.index(child)]
            bone = armature.edit_bones.new(bone_name)
            child_joint_location = canonical_joint_locations[parent] + Vector(joint_orientations[child]) * bone_length_current
            bone.head = tuple(canonical_joint_locations[parent])    
            bone.tail = child_joint_location
            bone.parent = joint_parent_dict[parent]
            canonical_joint_locations[child] = child_joint_location
            joint_parent_dict[child] = bone
            canonical_bone_matrixes[bone_name] = bone.matrix
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.scene.cursor.location = canonical_joint_locations['root']
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.context.active_object.select_set(False)
    return armature_object
    
def reset_bones(armature):
    """reset the armature to its canonical pose

    Args:
        armature (bpy_types.Object): the target armature
    """
    for parent, children in CHILDREN_TABLE.items():
        for child in children: 
            bone_name = BONE_NAMES[(parent, child)]
            armature.pose.bones[bone_name].matrix = canonical_bone_matrixes[bone_name]
            bpy.context.view_layer.update()


'''inverse kinematics
- Here we only rotate the bone to fit the joint locations.
- the bone roll is not constrained.
- the armature root is at the pelvis
'''
def get_current_joint_location(armature, joint_name, parent_joint):
    """get the position of the joint

    Args:
        armature (bpy_types.Object): the target armature
        joint_name (string): the target joint name
        parent_joint (string): parent joint name

    Returns:
        Vector: location as a Vector
    """
    bone_name = BONE_NAMES[(parent_joint, joint_name)]
    return Vector(armature.pose.bones[bone_name].tail)

def create_animation_inverse_kinematics(armature, motiondata, duration=100):
    """create the animation using ik

    Args:
        armature (bpy_types.Object): the target armature
        motiondata (dict): the motion data, pickle load from the result.pkl file
        duration (int, optional): duration of the animation. Defaults to 100.
    """
    scene = bpy.data.scenes['Scene']
    scene.render.fps = FPS_TARGET
    joint_rot_data = motiondata['J_rotmat']
    joint_loc_data = motiondata['J_locs']
    scene.frame_end = 10+len(joint_rot_data)

    for frame in range(min(len(joint_rot_data), duration)):
        armature.location = Vector(motiondata['J_locs'][frame, 0])
        
        for parent, children in CHILDREN_TABLE.items():
            for child in children: 
                
                child_current = get_current_joint_location(armature, child, parent)
                child_target = Vector(joint_loc_data[frame,  JOINT_NAMES.index(child)] - motiondata['J_locs'][frame,  0])
                
                parent_current = Vector(joint_loc_data[frame,  JOINT_NAMES.index(parent)] - motiondata['J_locs'][frame,  0])
                link_current = child_current - parent_current
                link_target = child_target - parent_current
                
                
                rd = link_current.rotation_difference(link_target)
                M = (
                Matrix.Translation(parent_current) @
                rd.to_matrix().to_4x4() @
                Matrix.Translation(-parent_current)
                ) 
                
                bone_name = BONE_NAMES[(parent, child)]
                armature.pose.bones[bone_name].matrix = M @ armature.pose.bones[bone_name].matrix
                bpy.context.view_layer.update()

        armature.keyframe_insert('location', frame=frame)
        armature.keyframe_insert('rotation_quaternion', frame=frame)
        bones = armature.pose.bones
        for bone in bones:
            bone.keyframe_insert('rotation_quaternion', frame=frame)
           
            
'''forward kinematics
- all bone transforms are constrained.
- the armature root is still in the world coordinate, to this end
- the FK result does not precisely aign with the IK result, but very close.
'''
def numpy2Matrix(src):
    '''
    src: 3x3 matrix
    '''
    mat = Matrix([src[0], src[1], src[2]])
    return mat

#perform rescaling in pose mode
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





def create_animation_forward_kinematics(armature, motiondata, duration=60):
    """create the animation using fk

    Args:
        armature (bpy_types.Object): the target armature
        motiondata (dict): the motion data, pickle load from the result.pkl file
        duration (int, optional): duration of the animation. Defaults to 60.
    """
    
    scene = bpy.data.scenes['Scene']
    scene.render.fps = FPS_TARGET
    joint_rot_data = motiondata['J_rotmat']
    joint_loc_data = motiondata['J_locs']
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
        rootbone.matrix = Matrix(rootloc)
        bpy.context.view_layer.update()


        for parent, children in CHILDREN_TABLE.items():
            
            for child in children:
                bone_name = BONE_NAMES[(parent, child)]
                if bone_name in ['left_thumb', 'right_thumb', 'left_fingers', 'right_fingers', 'left_hand', 'right_hand', 'right_toes', 'left_toes']:
                    continue
                parent_joint_index = JOINT_NAMES.index(parent)
                child_joint_index = JOINT_NAMES.index(child)
                
                ## canonical transform
               
                transf1 = np.eye(4)
                transf1[:-1, :-1] = np.array(armature.pose.bones[bone_name].bone.matrix_local)[:-1,:-1]
                #transf1[:-1, :-1] = np.array(canonical_bone_matrixes[bone_name])[:-1,:-1]
                
                ## transform w.r.t. the armature obj coordinate
                transf = np.eye(4)
                transf[:-1,:-1] = joint_rotmats[child_joint_index]
                
                if bone_name in ['left_hip', 'right_hip']:
                    transf[:-1, -1] = joint_locs[0]
                else:
                    transf[:-1, -1] = joint_locs[parent_joint_index]
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

def copy_pb_matrices(source_pb, target_pb):
    """copy matrices, used for constructing proper ik targets

    Args:
        source_pb (_type_): _description_
        target_pb (_type_): _description_
    """
    scene = bpy.data.scenes['Scene']
    for frame in range(scene.frame_start,scene.frame_end+1):
        scene.frame_set(frame)
        target_pb.matrix = source_pb.matrix.copy()
        target_pb.keyframe_insert('rotation_quaternion', frame=frame)
        target_pb.keyframe_insert('location', frame=frame)
        target_pb.keyframe_insert('scale', frame=frame)
    

def update_inputs(inertializer, foot, ik_target, ground_level):
    
    inertializer['input_ik_target_position'] = ik_target.matrix.translation.copy()
    inertializer['input_contact_point'] = foot.tail.copy()
    foot_z = inertializer['input_contact_point'][2]
    
    inertializer['input_contact_state'] = (foot_z <= ground_level)
    return inertializer

def update_position(ik_target, frame, inertializer):
    
    if inertializer['contact_lock']:
        ik_target.matrix.translation = inertializer['ik_target_position']
        bpy.context.view_layer.update()
        ik_target.keyframe_insert('location', frame=frame)
        print('lock contact at')
        print(frame)
        # print(inertializer['contact_position'])
        # print(ik_target.matrix)
    # else:
    #     foot.matrix.translation = inertializer['input_contact_position']    


def update_inertializer(inertializer, ik_target, unlock_radius):
    #unlock the contact state if the step is non-trivial
    unlock_contact = inertializer['contact_lock'] and (inertializer['ik_target_position'] - inertializer['input_ik_target_position']).length > unlock_radius

    if unlock_contact:
        print("unlock")
    
    if inertializer['contact_state'] == False and inertializer['input_contact_state']== True:
        inertializer['contact_lock'] = True
        inertializer['ik_target_position'] = ik_target.matrix.translation.copy()
        
    
    elif (inertializer['contact_lock'] and inertializer['contact_state'] and not inertializer['input_contact_state']) or unlock_contact:
        inertializer['contact_lock'] = False
    return inertializer
    


def unconnect_bones(armature, bone_names):
    for pb in bone_names:
        bpy.ops.object.mode_set(mode='POSE')#pose mode
        bpy.ops.pose.select_all(action = 'DESELECT')
        foot = armature.pose.bones[pb]
        #Set as active 
        bpy.context.object.data.bones.active = foot.bone
        #Select in viewport
        foot.bone.select = True

        bpy.ops.object.mode_set(mode='EDIT')

        # bpy.ops.armature.parent_clear(type='CLEAR')
        # bpy.context.active_bone.parent = None
        bpy.context.active_bone.use_connect = False

        bpy.ops.object.mode_set(mode='POSE')
        bpy.ops.pose.select_all(action = 'DESELECT')



def fix_sliding(armature, ground_level, unlock_radius):
    """This function loops  over the existing frames and updates the input contact state per frame, updates the contact

    Args:
        armature (_type_): _description_
        ground_level (_type_): _description_
        bpy.data.objects['Armature.001'].pose.bones['mixamorig:LeftFoot']
    """

    #step1: unconnect feet
    # unconnect_bones(armature, ['mixamorig:LeftFoot', 'mixamorig:RightFoot'])

    #step2: inertializer
    left_foot = armature.pose.bones['mixamorig:LeftFoot']
    right_foot = armature.pose.bones['mixamorig:RightFoot']
    left_ik_target = armature.pose.bones['mixamorig:LeftLegIK']
    right_ik_target = armature.pose.bones['mixamorig:RightLegIK']
    scene = bpy.data.scenes['Scene']
    #initialize inertializers
    left_inertializer = {
        'contact_state' : False,
        'contact_lock' : False,
        'ik_target_position' : left_ik_target.matrix.translation.copy(),
        'contact_point' : left_foot.tail.copy(),
        'input_contact_point' : Vector((0,0,0)),
        'input_ik_target_position' : Vector((0,0,0)),
        'input_contact_state' : False
        }
    right_inertializer = {
        'contact_state' : False,
        'contact_lock' : False,
        'ik_target_position' : right_ik_target.matrix.translation.copy(),
        'contact_point' : right_foot.tail.copy(),#the tail of the foot bone is where the contact happens
        'input_contact_point' : Vector((0,0,0)),
        'input_ik_target_position' : Vector((0,0,0)),
        'input_contact_state' : False
        }
    #initialize ik constraint

    
    for frame in range(scene.frame_start,scene.frame_end+1):
        scene.frame_set(frame)

        left_inertializer = update_inputs(left_inertializer, left_foot,left_ik_target, ground_level)
        left_inertializer = update_inertializer(left_inertializer, left_ik_target, unlock_radius)
        left_inertializer['contact_state'] = left_inertializer['input_contact_state']
        update_position(left_ik_target, frame, left_inertializer)

        right_inertializer = update_inputs(right_inertializer, right_foot, right_ik_target, ground_level)
        right_inertializer = update_inertializer(right_inertializer, right_ik_target, unlock_radius)
        right_inertializer['contact_state'] = right_inertializer['input_contact_state']
        update_position(right_ik_target, frame, right_inertializer)

        

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


    
def change_mixamo_rest_pose(mixamo_armature):
    """change the imported mixamo armature to our rest pose, use this before retargeting!

    Args:
        mixamo_armature (bpy_types.Object): the target mixamo armature
    """
    arm = mixamo_armature
    #resize
    arm.rotation_euler.x =0
    arm.scale = (0.004,0.004,0.004)

    #deselect all
    bpy.ops.pose.select_all(action='DESELECT')
    bpy.context.object.data.bones.active = None

    sboneToSelect =  arm.pose.bones['mixamorig:LeftUpLeg'].bone
    bpy.context.object.data.bones.active = sboneToSelect
    #left upper leg rotate 20 degrees in z direction
    bpy.ops.transform.rotate(value=-0.35, orient_axis='Z', orient_type = 'LOCAL')
    bpy.ops.transform.translate(value=(0.0, 0.0, 0.03), orient_type='GLOBAL')





    bpy.ops.pose.select_all(action='DESELECT')
    bpy.context.object.data.bones.active = None

    sboneToSelect =  arm.pose.bones['mixamorig:RightUpLeg'].bone
    bpy.context.object.data.bones.active = sboneToSelect
    #right upper leg rotate -20 degrees in z direction
    bpy.ops.transform.rotate(value=0.35, orient_axis='Z', orient_type = 'LOCAL')
    bpy.ops.transform.translate(value=(0.0, 0.0, 0.03), orient_type='GLOBAL')

    bpy.ops.pose.select_all(action='DESELECT')
    bpy.context.object.data.bones.active = None



if __name__ == '__main__':
    if os.path.exists(INPUT_FILE_PATH):
        print("Importing animation")
        with open(INPUT_FILE_PATH, "rb") as f:
            motiondata = pickle.load(f, encoding="latin1")
            
        duration=100
        """demo1: create armature and create fk animation
        """
        # armature1 = create_armature(motiondata['J_shape'], "forward_kinematics_body")
        # create_animation_forward_kinematics(armature1, motiondata, duration)
        
        """demo2: get the imported armature with mesh, rescale, set new rest pose, create fk animation
        """
        armature2 = bpy.data.objects['Armature.002']
        copy_pb_matrices(armature2.pose.bones['mixamorig:LeftFoot'],armature2.pose.bones['mixamorig:LeftLegIK'])
        copy_pb_matrices(armature2.pose.bones['mixamorig:RightFoot'],armature2.pose.bones['mixamorig:RightLegIK'])
        fix_sliding(armature2, 0, 0.045)
        # rescale_bones(motiondata['J_shape'], armature2)
        # set_rest_pose(armature2)
        # create_animation_forward_kinematics(armature2, motiondata, duration)

        """demo3: set the mixamo armature to our desired rest pose, then use the rokoko studio plugin for retargeting
        """
        # armature3 = bpy.data.objects['Armature.001'] #here is the name of the mixamo armature
        # change_mixamo_rest_pose(armature3)
        # set_rest_pose(armature3)
        

        """demo4: create armature and create ik animation
        """
        # armature4 = create_armature(motiondata['J_shape'], "inverse_kinematics_body")
        # create_animation_inverse_kinematics(armature4, motiondata, duration)
        
        """demo5: (does not work currently) fix sliding
        """
        # armature5 = bpy.data.objects['Armature.001']
        # fix_sliding(armature5, n = 0.245)
    else:
        print("Input file not found")

    