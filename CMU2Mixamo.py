import numpy as np
import bpy
from mathutils import Vector, Matrix, Quaternion



# CMU Armature properties
CMU_JOINT_NAMES = [
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

CMU_CHILDREN_TABLE = {
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


CMU_JOINT_DEFAULT_ORIENTATION = np.array(
[[ 0.0000e+00,  0.0000e+00,  0.0000e+00],
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



# mixamo armature properties
def getJointNames(children_table):
    out = []
    for key, val in children_table.items():
        if key not in out:
            out.append(key)
        for vv in val:
            out.append(vv)
    
    return out


# the body model, pad with 'mixamorig:' in the front. Check blender for details.
MIXAMO_CHILDREN_TABLE_BODY = {
'Hips': ['Spine', 'LeftUpLeg', 'RightUpLeg'],
'Spine': ['Spine1'],
'Spine1': ['Spine2'],
'Spine2': ['Neck', 'LeftShoulder', 'RightShoulder'],
'Neck': ['Head'],
'Head': ['HeadTop_End'],
'LeftShoulder': ['LeftArm'],
'LeftArm': ['LeftForeArm'],
'LeftForeArm': ['LeftHand'],
'RightShoulder': ['RightArm'],
'RightArm': ['RightForeArm'],
'RightForeArm': ['RightHand'],
'LeftUpLeg': ['LeftLeg'],
'LeftLeg': ['LeftFoot'],
'LeftFoot': ['LeftToeBase'],
'LeftToeBase': ['LeftToe_End'],
'RightUpLeg': ['RightLeg'],
'RightLeg': ['RightFoot'],
'RightFoot': ['RightToeBase'],
'RightToeBase': ['RightToe_End'],
 }

MIXAMO_JOINT_NAMES_BODY = getJointNames(MIXAMO_CHILDREN_TABLE_BODY)


# the hand hierarchy. Check blender for visualization. 
CHILDREN_TABLE_LEFTHAND = {
'LeftHand': ['LeftHandMiddle1', 'LeftHandThumb1', 'LeftHandIndex1', 'LeftHandRing1', 'LeftHandPinky1'],
'LeftHandMiddle1': ['LeftHandMiddle2'],
'LeftHandMiddle2': ['LeftHandMiddle3'],
'LeftHandMiddle3': ['LeftHandMiddle4'],
'LeftHandThumb1': ['LeftHandThumb2'],
'LeftHandThumb2': ['LeftHandThumb3'],
'LeftHandThumb3': ['LeftHandThumb4'],
'LeftHandIndex1': ['LeftHandIndex2'],
'LeftHandIndex2': ['LeftHandIndex3'],
'LeftHandIndex3': ['LeftHandIndex4'],
'LeftHandRing1': ['LeftHandRing2'],
'LeftHandRing2': ['LeftHandRing3'],
'LeftHandRing3': ['LeftHandRing4'],
'LeftHandPinky1': ['LeftHandPinky2'],
'LeftHandPinky2': ['LeftHandPinky3'],
'LeftHandPinky3': ['LeftHandPinky4'],
}

CHILDREN_TABLE_RIGHTHAND = {
'RightHand': ['RightHandMiddle1', 'RightHandThumb1', 'RightHandIndex1', 'RightHandRing1', 'RightHandPinky1'],
'RightHandMiddle1': ['RightHandMiddle2'],
'RightHandMiddle2': ['RightHandMiddle3'],
'RightHandMiddle3': ['RightHandMiddle4'],
'RightHandThumb1': ['RightHandThumb2'],
'RightHandThumb2': ['RightHandThumb3'],
'RightHandThumb3': ['RightHandThumb4'],
'RightHandIndex1': ['RightHandIndex2'],
'RightHandIndex2': ['RightHandIndex3'],
'RightHandIndex3': ['RightHandIndex4'],
'RightHandRing1': ['RightHandRing2'],
'RightHandRing2': ['RightHandRing3'],
'RightHandRing3': ['RightHandRing4'],
'RightHandPinky1': ['RightHandPinky2'],
'RightHandPinky2': ['RightHandPinky3'],
'RightHandPinky3': ['RightHandPinky4'],

}


# map from mixamo to CMU
MIXAMO_TO_CMU = {
 'Hips': 'root',
 'Spine': 'lowerback',
 'LeftUpLeg': 'lhipjoint',
 'RightUpLeg': 'rhipjoint',
 'Spine1': 'upperback',
 'Spine2': 'thorax',
 'Neck': 'lowerneck',
 'LeftShoulder': None,
 'RightShoulder': None,
 'Head': 'upperbeck',
 'HeadTop_End': None,
 'LeftArm': 'lclavicle',
 'LeftForeArm': 'lhumerus',
 'LeftHand': 'lwrist',
 'RightArm': 'rclavicle',
 'RightForeArm': 'lhumerus',
 'RightHand': 'lwrist',
 'LeftLeg': 'lfemur',
 'LeftFoot': 'ltibia',
 'LeftToeBase': 'lfoot',
 'LeftToe_End': 'ltoes',
 'RightLeg': 'rfemur',
 'RightFoot': 'rtibia',
 'RightToeBase': 'lfoot',
 'RightToe_End': 'ltoes',

}


def parse_mixamo_armature():
    armature_obj = bpy.data.objects['Armature']
    bpy.context.view_layer.objects.active = armature_obj
    armature_obj.select_set(True)
    bones = armature_obj.pose.bones



joint_parent_dict = {}
relative_rotation_around_parent = {}
canonical_joint_locations = {}

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
            
            child_joint_location = canonical_joint_locations[parent] + Vector(joint_orientations[child]) * bone_length_current
            
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
    


