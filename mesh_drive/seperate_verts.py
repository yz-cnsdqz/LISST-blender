
import numpy as np

import os
import argparse

from os.path import join

parser = argparse.ArgumentParser('Rename inconsistent image files.')
parser.add_argument('--data_dir', default=r'C:\Users\hshang\Downloads\mocap_zju', type=str, help='Data direcotry.')
args = parser.parse_args()

if __name__ == '__main__':
    data_dir = args.data_dir
    # path_list = ['313','315','377','386','387','390','392','393','394']
    path_list =['313']
    
    for seq_name in path_list:
        desti = join(data_dir, seq_name, 'joints3d')
        if not os.path.exists(desti):
            os.makedirs(desti)
        all_verts = np.load(join(data_dir, f'CoreView_{str(seq_name)}_verts.npy'))
        all_verts = all_verts[:,::1,:]
        for i, verts in enumerate(all_verts):
            np.save(join(desti,str(i)), verts)
    print(verts.shape)



    