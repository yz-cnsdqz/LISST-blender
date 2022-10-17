import numpy as np


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


# mixamo armature properties

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

MIXAMO_JOINT_NAMES_BODY = []
for key, val in MIXAMO_CHILDREN_TABLE_BODY.items():
    if key not in MIXAMO_JOINT_NAMES_BODY:
        MIXAMO_JOINT_NAMES_BODY.append(key)
    for vv in val:
        MIXAMO_JOINT_NAMES_BODY.append(vv)


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


