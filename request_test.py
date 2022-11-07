import read_files
import numpy as np

request_matrix = read_files.R
demand = [0] * len(read_files.T)
quantities = request_matrix[:, 6]
result = {node: quantities[request_matrix[:, 1] == node].sum() for node in np.unique(request_matrix[:, 1])}

for k,v in result.items():
    demand[k] = v

print(request_matrix)
print(result)
print(demand)
