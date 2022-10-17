import pickle
import os
import bpy
import sys
import numpy as np
import math

from mathutils import Vector, Matrix, Quaternion
# Input pkl path
# INPUT_FILE_PATH = "C:\\Users\\Lukas\\Projects\\lisst-motion-visualization\\input\\motion_0.pkl"
#INPUT_FILE_PATH = "/home/yzhang/workspaces/LISST/results/src/LISST_SHAPER_v0/results/mocap_zju_2/results.pkl"
INPUT_FILE_PATH = r"C:\Users\zhang\Downloads\results.pkl"
# Animation properties
FPS_TARGET = 30

# Armature properties
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

joint_parent_dict = {}
relative_rotation_around_parent = {}

JOINT_DEFAULT_ORIENTATION = np.array([[ 0.0000e+00,  0.0000e+00,  0.0000e+00],
[ 6.2366e-01, -7.1724e-01,  3.1083e-01],
[ 3.4202e-01, -9.3969e-01,  0.0000e+00],
[ 3.4202e-01, -9.3969e-01,  0.0000e+00],
[ 6.7389e-02, -1.8515e-01,  9.8040e-01],
[ 1.5357e-11, -4.2199e-11,  1.0000e+00],
[-4.8902e-01, -8.0035e-01,  3.4685e-01],
[-3.4202e-01, -9.3969e-01,  0.0000e+00],
[-3.4202e-01, -9.3969e-01,  0.0000e+00],
[-8.5932e-02, -2.3610e-01,  9.6792e-01],
[-1.5354e-11, -4.2199e-11,  1.0000e+00],
[-7.6255e-03,  9.9898e-01, -4.4555e-02],
[ 4.2236e-02,  9.9887e-01, -2.1916e-02],
[ 4.4246e-02,  9.9902e-01,  7.4760e-05],
[ 3.1217e-03,  9.8464e-01,  1.7456e-01],
[-3.8303e-02,  9.8526e-01, -1.6675e-01],
[-1.2955e-02,  9.9706e-01, -7.5539e-02],
[ 9.1286e-01,  3.7052e-01, -1.7149e-01],
[ 1.0000e+00, -4.4896e-11,  3.6436e-27],
[ 1.0000e+00, -4.4897e-11,  5.9720e-27],
[ 1.0000e+00, -4.4896e-11,  9.8329e-27],
[ 1.0000e+00, -4.4902e-11,  1.9655e-26],
[ 1.0000e+00, -4.4897e-11,  3.9328e-26],
[ 7.0711e-01, -6.3493e-11,  7.0711e-01],
[-9.0092e-01,  3.9536e-01, -1.7898e-01],
[-1.0000e+00, -4.4897e-11,  1.0388e-27],
[-1.0000e+00, -4.4895e-11,  5.0273e-27],
[-1.0000e+00, -4.4901e-11,  9.8283e-27],
[-1.0000e+00, -4.4889e-11,  1.9677e-26],
[-1.0000e+00, -4.4909e-11,  3.9283e-26],
[-7.0711e-01, -6.3488e-11,  7.0711e-01]])

canonical_joint_locations = {}

canonical_bone_matrixes = {}

ROT_Z_180 = Matrix.Rotation(math.radians(180), 4, 'Z')
ROT_X_NEG90 = Matrix.Rotation(math.radians(-90), 4, 'X')
ROT_TO_BLENDER = ROT_X_NEG90 @ ROT_Z_180

def create_yup_armature(bone_lengths, name):
    armature = bpy.data.armatures.new("Armature_"+name)
    armature_object = bpy.data.objects.new(name, armature)
    bpy.context.scene.collection.objects.link(armature_object)
    bpy.context.view_layer.objects.active = armature_object
    armature_object.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')

    rootbone = armature.edit_bones.new('root')
    rootbone.head = (0, 0, 0)
    rootbone.tail = rootbone.head + Vector([0, 0.001, 0])
    joint_parent_dict['root'] = rootbone
    canonical_joint_locations['root'] = rootbone.head

    joint_orientations = dict(zip(JOINT_NAMES, JOINT_DEFAULT_ORIENTATION))

    for parent, children in CHILDREN_TABLE.items():
        for child in children:
            bone_name = BONE_NAMES[(parent, child)]
            bone_length_current = bone_lengths[JOINT_NAMES.index(child)]
            bone = armature.edit_bones.new(bone_name)
            
            child_joint_location = canonical_joint_locations[parent] + Vector(joint_orientations[child]) * bone_length_current
            
            bone.head = tuple(canonical_joint_locations[parent])    
            bone.tail = bone.head + Vector([0,bone_length_current,0])#child_joint_location
            
            bone.parent = joint_parent_dict[parent]
            
            canonical_joint_locations[child] = child_joint_location
            joint_parent_dict[child] = bone
            canonical_bone_matrixes[bone_name] = bone.matrix
    
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.scene.cursor.location = rootbone.head
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.context.active_object.select_set(False)
    
    return armature_object
    
def create_armature(bone_lengths, name):
    armature = bpy.data.armatures.new("Armature_"+name)
    armature_object = bpy.data.objects.new(name, armature)
    bpy.context.scene.collection.objects.link(armature_object)
    bpy.context.view_layer.objects.active = armature_object
    armature_object.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    
    rootbone = armature.edit_bones.new('root')
    rootbone.head = (0, 0, 0)
    rootbone.tail = rootbone.head + Vector([0, 0.001, 0])
    joint_parent_dict['root'] = rootbone
    canonical_joint_locations['root'] = rootbone.head
    
    joint_orientations = dict(zip(JOINT_NAMES, JOINT_DEFAULT_ORIENTATION))
    
    for parent, children in CHILDREN_TABLE.items():
        for child in children:
            bone_name = BONE_NAMES[(parent, child)]
            bone_length_current = bone_lengths[JOINT_NAMES.index(child)]
            bone = armature.edit_bones.new(bone_name)
            
            child_joint_location = canonical_joint_locations[parent] + ROT_TO_BLENDER @ Vector(joint_orientations[child]) * bone_length_current
            
            bone.head = tuple(canonical_joint_locations[parent])    
            bone.tail = child_joint_location
            
            bone.parent = joint_parent_dict[parent]
            
            canonical_joint_locations[child] = child_joint_location
            joint_parent_dict[child] = bone
            canonical_bone_matrixes[bone_name] = bone.matrix
    
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.scene.cursor.location = rootbone.head
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.context.active_object.select_set(False)
    
    return armature_object
    




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
           
            
    
def reset_bones(armature):
    for parent, children in CHILDREN_TABLE.items():
            for child in children: 
                 bone_name = BONE_NAMES[(parent, child)]
                 armature.pose.bones[bone_name].matrix = canonical_bone_matrixes[bone_name]
                 bpy.context.view_layer.update()
                
def get_current_joint_location(armature, joint_name, parent_joint):
    bone_name = BONE_NAMES[(parent_joint, joint_name)]
    return Vector(armature.pose.bones[bone_name].tail)
    

def numpy2Matrix(src):
    '''
    src: 3x3 matrix
    '''
    mat = Matrix([src[0], src[1], src[2]])
    return mat




def create_animation_forward_kinematics(armature, motiondata, duration=100):
    
    scene = bpy.data.scenes['Scene']
    scene.render.fps = FPS_TARGET
    joint_rot_data = motiondata['J_rotmat']
    scene.frame_end = 10+len(joint_rot_data)

    for frame in range(min(len(joint_rot_data), duration)):
        scene.frame_set(frame)
        reset_bones(armature)
        
        joint_rotmats = joint_rot_data[frame]
        
        armature.location = Vector(motiondata['J_locs'][frame,  0])
        armature.rotation_mode = 'QUATERNION'
        armature.rotation_quaternion = (numpy2Matrix(joint_rotmats[0]).to_4x4()).to_quaternion()
        for parent, children in CHILDREN_TABLE.items():
            for child in children:
                bone_name = BONE_NAMES[(parent, child)]
                
                parent_joint_index = JOINT_NAMES.index(parent)
                child_joint_index = JOINT_NAMES.index(child)
                
                parent_rot = numpy2Matrix(joint_rotmats[parent_joint_index])

                child_rot = numpy2Matrix(joint_rotmats[child_joint_index])
                
                bone_rot =  child_rot.to_4x4() @ parent_rot.to_4x4().transposed()
                # bone_rot =  parent_rot.to_4x4().transposed() @ child_rot.to_4x4()
                head_location = armature.pose.bones[bone_name].head 
                
                M = (
                Matrix.Translation(motiondata['J_locs'][frame,  parent_joint_index]) @
                bone_rot.to_4x4() @
                Matrix.Translation(-head_location)
                ) 
                armature.pose.bones[bone_name].matrix = M #@ armature.pose.bones[bone_name].matrix
                
                child_parent_bone_rot =   bone_rot

        bpy.context.view_layer.update()
                

        
        armature.keyframe_insert('location', frame=frame)
        armature.keyframe_insert('rotation_quaternion', frame=frame)
        bones = armature.pose.bones
        for bone in bones:
            bone.keyframe_insert('rotation_quaternion', frame=frame)
            
#this is the direct comparable version of Yan's current visualization which applies transforms to all joints
def create_animation_yup_forward_kinematics(armature, motiondata, duration=100):
    scene = bpy.data.scenes['Scene']
    scene.render.fps = FPS_TARGET
    joint_rot_data = motiondata['J_rotmat']
    scene.frame_end = 10+len(joint_rot_data)
        
    for frame in range(min(len(joint_rot_data), duration)):
        scene.frame_set(frame)
        reset_bones(armature)
        joint_rotmats = joint_rot_data[frame]
        
        armature.location = Vector(motiondata['J_locs'][frame,  0])
        armature.rotation_mode = 'QUATERNION'
        armature.rotation_quaternion = (numpy2Matrix(joint_rotmats[0]).to_4x4()).to_quaternion()
        for parent, children in CHILDREN_TABLE.items():
            for child in children:
                bone_name = BONE_NAMES[(parent, child)]
                
                parent_joint_index = JOINT_NAMES.index(parent)
                child_joint_index = JOINT_NAMES.index(child)
                
                parent_rot = numpy2Matrix(joint_rotmats[0])

                child_rot = numpy2Matrix(joint_rotmats[child_joint_index])
                
                bone_rot =  child_rot.to_4x4() @ parent_rot.to_4x4().transposed()
                # bone_rot = Matrix.Identity(4)
                # bone_rot =  parent_rot.to_4x4().transposed() @ child_rot.to_4x4()
                head_location = armature.pose.bones[bone_name].head 
                print(head_location)
                
                M = (
                # Matrix.Translation(motiondata['J_locs'][frame,  parent_joint_index]) @
                bone_rot.to_4x4() @
                Matrix.Translation(head_location)
                ) 
                armature.pose.bones[bone_name].matrix = M @ armature.pose.bones[bone_name].matrix
                
                child_parent_bone_rot =   bone_rot

        bpy.context.view_layer.update()
                

        
        armature.keyframe_insert('location', frame=frame)
        armature.keyframe_insert('rotation_quaternion', frame=frame)
        bones = armature.pose.bones
        for bone in bones:
            bone.keyframe_insert('rotation_quaternion', frame=frame)
    

if __name__ == '__main__':
    if os.path.exists(INPUT_FILE_PATH):
        print("Importing animation")
        with open(INPUT_FILE_PATH, "rb") as f:
            motiondata = pickle.load(f, encoding="latin1")
            
        
        # motiondata['J_rotmat'] = np.expand_dims(motiondata['J_rotmat'], axis=1)
        # motiondata['J_locs'] = np.expand_dims(motiondata['J_locs'], axis=1)
        # motiondata['J_shape'] = np.expand_dims(motiondata['J_shape'], axis=0)
        
        
        armature = create_yup_armature(motiondata['J_shape'], "forward_kinematics_body")
        create_animation_yup_forward_kinematics(armature, motiondata)
        
        armature2 = create_armature(motiondata['J_shape'], "inverse_kinematics_body")
        create_animation_inverse_kinematics(armature2, motiondata)

    else:
        print("Input file not found")

    