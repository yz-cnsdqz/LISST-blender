import sys
import os
import pickle
import bpy
#sys.path.append(".")
sys.path.append("C:\LISST-blender")
sys.path.append("C:\LISST-blender\mesh_drive")
INPUT_FILE_PATH = r"C:\Users\hshang\Downloads\results.pkl"

# from mesh_drive.forward_kinematics import rescale_bones
# from mesh_drive.forward_kinematics import create_animation_forward_kinematics
# from mesh_drive.forward_kinematics import set_rest_pose
# from mesh_drive.forward_kinematics import change_mixamo_rest_pose
# from mesh_drive.forward_kinematics import create_armature
# from mesh_drive.forward_kinematics import create_animation_inverse_kinematics
from mesh_drive.forward_kinematics import fix_sliding
# from mesh_drive.forward_kinematics import JOINT_NAMES
# from mesh_drive.forward_kinematics import JOINT_DEFAULT_ORIENTATION
# from mesh_drive.forward_kinematics import CHILDREN_TABLE
# from mesh_drive.forward_kinematics import BONE_NAMES

if __name__ == '__main__':
    if os.path.exists(INPUT_FILE_PATH):
        print("Importing animation")
        with open(INPUT_FILE_PATH, "rb") as f:
            motiondata = pickle.load(f, encoding="latin1")
            
        duration=100
        
        """demo5: (does not work currently) fix sliding
        """
        armature5 = bpy.data.objects['Armature.001']
        fix_sliding(armature5, n = 0.245)
    else:
        print("Input file not found")