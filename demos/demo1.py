import sys
import os
import pickle
#sys.path.append(".")
sys.path.append("C:\LISST-blender")
sys.path.append("C:\LISST-blender\mesh_drive")

INPUT_FILE_PATH = r"C:\Users\hshang\Downloads\results.pkl"

from mesh_drive.forward_kinematics import create_armature
from mesh_drive.forward_kinematics import create_animation_forward_kinematics
from mesh_drive.forward_kinematics import JOINT_NAMES
from mesh_drive.forward_kinematics import JOINT_DEFAULT_ORIENTATION
from mesh_drive.forward_kinematics import CHILDREN_TABLE
from mesh_drive.forward_kinematics import BONE_NAMES

if __name__ == '__main__':
    if os.path.exists(INPUT_FILE_PATH):
        print("Importing animation")
        with open(INPUT_FILE_PATH, "rb") as f:
            motiondata = pickle.load(f, encoding="latin1")
            
        duration=100
        """demo1: create armature and create fk animation
        """
        armature1 = create_armature(motiondata['J_shape'], "forward_kinematics_body")
        create_animation_forward_kinematics(armature1, motiondata, duration)
        
       
    else:
        print("Input file not found")