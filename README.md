# LISST motion visualization
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
