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


def fix_sliding(armature, n = 0.245):
    """fix the feet sliding issue by checking their contact with the floor, currently not working

    Args:
        armature (bpy_types.Object): the target armature
        n (float, optional): the distance between the foot pose bone and the (x,y,0) plane. Defaults to 0.245.
    """
    scene = bpy.data.scenes['Scene']
    left_foot = armature.pose.bones['mixamorig:LeftFoot']
    right_foot = armature.pose.bones['mixamorig:RightFoot']
    #currently in contact?
    l_contact = False
    r_contact = False
    #is it the first contacting frame?
    l_first_contact = False
    r_first_contact = False
    #there are previous contacting frame(s)?
    l_pre_contact = False
    r_pre_contact = False

    for frame in range(scene.frame_start,scene.frame_end+1):
        scene.frame_set(frame)
        lf_z = left_foot.matrix.translation[2]
        rf_z = right_foot.matrix.translation[2]
        l_contact = lf_z < n
        r_contact = rf_z < n

        if not l_pre_contact and l_contact:
            l_first_contact = True
        if not r_pre_contact and r_contact:
            r_first_contact = True
        
        if l_first_contact:
            lloc=np.array([left_foot.matrix.translation[0],left_foot.matrix.translation[1],n])
            l_pre_contact = True
            l_first_contact = False
        if r_first_contact:
            rloc=np.array([right_foot.matrix.translation[0],right_foot.matrix.translation[1],n])
            r_pre_contact = True
            l_first_contact = False
        if l_contact:
            lmat = np.eye(4)
            lmat[:-1, :-1] = np.array(left_foot.matrix)[:-1,:-1]
            lmat[:-1,-1] = lloc

            #this is currently where the problem is: this assignment does not work
            left_foot.matrix = Matrix(lmat)
            
            left_foot.keyframe_insert('rotation_quaternion', frame=frame)
            left_foot.keyframe_insert('location', frame=frame)
            bpy.context.view_layer.update()
        if r_contact:
            rmat = np.eye(4)
            rmat[:-1, :-1] = np.array(right_foot.matrix)[:-1,:-1]
            rmat[:-1,-1] = rloc

            #same here
            right_foot.matrix = Matrix(rmat)
            
            right_foot.keyframe_insert('rotation_quaternion', frame=frame)
            right_foot.keyframe_insert('location', frame=frame)
            bpy.context.view_layer.update()
        if not l_contact:
            l_pre_contact = False
        if not r_contact:
            r_pre_contact = False


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

        if (obj.type == 'MESH'
            ):
                
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
        # armature2 = bpy.data.objects['Armature.001']
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

    