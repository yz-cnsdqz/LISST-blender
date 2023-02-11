# LISST motion visualization


## LISST Base Mesh Animation

### Base Body Mesh
The base body mesh is developped based on the Mixamo Mannequin character.
Specificially, we re-scale, deform, and repose the original body mesh, so that it can fit the **LISST canonical body skeleton with the mean shape**. We ask a professional artist to paint the skinning weights, and combine the LISST body skeleton with the orginal Mixamo hand skeletons. 
The base body mesh is saved in `data/LISST_canonical_mesh.fbx`. After importing, it stands on the ground with *Z-up*. 

### LISST Motion File
Either motion capture or motion synthesis will produce some files to store the motions. To unify all workflows, we define a motion file as a `.pkl` file containing a python dictionary of `numpy.ndarray`:
```
motion_data = {
    'r_locs': ...,      # LISST body root (pelvis) locations in the world coordinate, with shape (t, 1, 3)
    'J_rotmat': ...,    # LISST joint rotation matrices in world coordinate wrt the canonical rest pose, with shape (t,J,3,3)
    'J_shape': ...,     # LISST bone lengths, with shape (J,)
    'J_locs_3d': ...,   # LISST joint locations in the world coordinate, with shape (t,J,3)
    'J_locs_2d': ...,   # LISST joint locations in indiviual camera views, only available for motion capture results, with shape (t, n_views, J, 2)
}
```

For **add_animation_batch**, the file format is as follows, in which the batch dimension is right after the time dimension.
```
motion_data = {
    'r_locs': ...,      # LISST body root (pelvis) locations in the world coordinate, with shape (t, b, 1, 3)
    'J_rotmat': ...,    # LISST joint rotation matrices in world coordinate wrt the canonical rest pose, with shape (t,b,J,3,3)
    'J_shape': ...,     # LISST bone lengths, with shape (b,J)
    'J_locs_3d': ...,   # LISST joint locations in the world coordinate, with shape (t,b,J,3)
    'J_locs_2d': ...,   # LISST joint locations in indiviual camera views, only available for motion capture results, with shape (t, b, n_views, J, 2)
}
```



### Animation

The animation process consists of two steps. First, we rescale the body mesh according to the input `J_shape`. Second, we transfer the motion data to the armature. Note that we *DON'T* transfer the joint locations, because the bone roll will be lost. Instead, we transfer the root locations and the bone rotations. Blender automatically solves all local transformations via forward kinematics.



## Motion Retargeting

### Retarget and Montion Drive Update 13.12.2022
We switch our retargeting pipline to rokoko https://github.com/Rokoko/rokoko-studio-live-blender

Added 5 demos in mesh-drive/forward-kinematics.py

### Retarget Update 26.10.2022

_based on the add-on https://github.com/cgvirus/Simple-Retarget-Tool-Blender under GPL-3.0 license<br /> You are also welcome to use my more up-to-date FK folder with every step done https://polybox.ethz.ch/index.php/s/4byApEzNiQb9AiW
please also refer to this retarget demo retarget.blend https://polybox.ethz.ch/index.php/s/bZ29DIANmk8cpyn_


1. Install and activate the addon in blender through Edit->Preferences->Install... (any previous version of the addon should be removed first) (save preferances!)<br />
![alt text](/images/Install_Add_On.png)
2. Adjust the target armature in to the same scale/pose/orientation in pose mode and set as rest pose with the 'set rest pose with objects' method under pose->Simple Retarget->set rest pose with objects<br />
![alt text](/images/Posing.png)
3. First select one of the bones of the LISST armature then shift select one of the bones of the target armature (order matters)<br />
![alt text](/images/Retarget.png)
4. pose->Simple Retarget->Retarget LISST to Mixamo<br />
![alt text](/images/done.png)

<p align="right">(<a href="#readme-top">back to top</a>)</p>
