import pickle
import os
import bpy
import sys
import numpy as np
import math
from mathutils import Vector, Matrix, Quaternion
from armature_properties import *


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
    armature = bpy.data.armatures.new("Armature_"+name)
    armature_object = bpy.data.objects.new(name, armature)
    bpy.context.scene.collection.objects.link(armature_object)
    bpy.context.view_layer.objects.active = armature_object
    armature_object.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    # rootbone = armature.edit_bones.new('root')
    # rootbone.head = (0, 0, 0)
    # rootbone.tail = rootbone.head + Vector([0, 0.01, 0])
    joint_parent_dict['root'] = None
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
    bone_name = BONE_NAMES[(parent_joint, joint_name)]
    return Vector(armature.pose.bones[bone_name].tail)

def create_animation_inverse_kinematics(armature, motiondata, duration=100):
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


def create_animation_forward_kinematics(armature, motiondata, duration=60):
    
    scene = bpy.data.scenes['Scene']
    scene.render.fps = FPS_TARGET
    joint_rot_data = motiondata['J_rotmat']
    joint_loc_data = motiondata['J_locs']
    scene.frame_end = 10+len(joint_rot_data)

    for frame in range(min(len(joint_rot_data), duration)):
        scene.frame_set(frame)
        joint_rotmats = joint_rot_data[frame]
        joint_locs = joint_loc_data[frame]
        # armature.location = Vector(motiondata['J_locs'][frame,  0])
        # armature.rotation_mode = 'QUATERNION'
        # armature.rotation_quaternion = (numpy2Matrix(joint_rotmats[0]).to_4x4()).to_quaternion()
        # armature.select_set(True)
        # bpy.ops.object.mode_set(mode='POSE')

        for parent, children in CHILDREN_TABLE.items():
            
            for child in children:
                bone_name = BONE_NAMES[(parent, child)]
                parent_joint_index = JOINT_NAMES.index(parent)
                child_joint_index = JOINT_NAMES.index(child)
                
                ## canonical transform
                transf1 = np.eye(4)
                # transf1[:-1, :-1] = np.array(armature.pose.bones[bone_name].bone.matrix_local)[:-1,:-1]
                transf1[:-1, :-1] = np.array(canonical_bone_matrixes[bone_name])[:-1,:-1]
                
                ## transform w.r.t. the armature obj coordinate
                transf = np.eye(4)
                transf[:-1,:-1] = joint_rotmats[child_joint_index]
                transf[:-1, -1] = joint_locs[parent_joint_index]
                
                M = (
                Matrix(transf) @ 
                Matrix(transf1)
                ) 
                armature.pose.bones[bone_name].matrix = M #armature.pose.bones[bone_name].matrix
                
                ## refresh the context
                bpy.context.view_layer.update()
                
        armature.keyframe_insert('location', frame=frame)
        armature.keyframe_insert('rotation_quaternion', frame=frame)
        bones = armature.pose.bones
        for bone in bones:
            bone.keyframe_insert('rotation_quaternion', frame=frame)
            bone.keyframe_insert('location', frame=frame)


if __name__ == '__main__':
            
    armature0 = create_armature(BDOY_MEAN_SHAPE[0], "forward_kinematics_body")

    