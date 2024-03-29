import numpy as np


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

JOINT_DEFAULT_ORIENTATION = np.array([
    [ 0.0000e+00,  0.0000e+00,  0.0000e+00],
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


BDOY_MEAN_SHAPE = np.array([[0.0000, 0.1415, 0.3968, 0.4271, 0.1153, 0.0578, 0.1401, 0.4003, 0.4246,
         0.1179, 0.0590, 0.1151, 0.1149, 0.1161, 0.0966, 0.0970, 0.0972, 0.1974,
         0.2864, 0.1866, 0.0933, 0.0365, 0.0294, 0.0422, 0.1958, 0.2892, 0.1877,
         0.0939, 0.0379, 0.0305, 0.0439]])