import numpy as np

FILEPATH = r"C:\Users\hshang\Downloads\params"
params = np.load(r"C:\Users\hshang\Downloads\params\0.npy", allow_pickle=True)
# for i in range(646):
#     params = np.load(FILEPATH+"\{i}.npy")
print(params.tolist()['shapes'])