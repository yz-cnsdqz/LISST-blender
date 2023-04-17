import pickle
import numpy as np



dir = r'C:\Users\hshang\Downloads\CoreView_315.pkl'
# Open the file for reading in binary mode
with open(dir, 'rb') as file:
    # Call the load method to deserialize the data
    data = pickle.load(file)

# Use the data as needed
kps = np.stack(data['J_locs_3d'])[:,0:31]
print(kps.shape)



