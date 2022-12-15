import sys
import os
import pickle
import bpy
<<<<<<< Updated upstream
#sys.path.append(".")
sys.path.append("C:\LISST-blender")
sys.path.append("C:\LISST-blender\mesh_drive")
=======
sys.path.append(".")
>>>>>>> Stashed changes
INPUT_FILE_PATH = r"C:\Users\hshang\Downloads\results.pkl"

from mesh_drive.forward_kinematics import rescale_bones
from mesh_drive.forward_kinematics import create_animation_forward_kinematics
from mesh_drive.forward_kinematics import set_rest_pose
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
        
        
        """demo2: get the imported armature with mesh, rescale, set new rest pose, create fk animation
        """
        armature2 = bpy.data.objects['Armature.001']
        rescale_bones(motiondata['J_shape'], armature2)
        set_rest_pose(armature2)
        create_animation_forward_kinematics(armature2, motiondata, duration)

       
    else:
        print("Input file not found")