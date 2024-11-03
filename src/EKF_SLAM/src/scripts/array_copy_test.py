"""import numpy as np

a=np.array([
    [1,2,3],
    [3,4,5]
])

a= np.array([
    a[1,:],
    a[0,:],
    [5,6,7]
])

a = np.array([
    a,
    [[1,2,3]]
])

print(a)"""

import numpy as np

# def expand_matrix(a, iterations):
#     a = np.array(a)
#     for i in range(iterations):
#         # Add two columns of zeros
#         a = np.pad(a, ((0, 0), (0, 2)), constant_values=0)
        
#         # Add two new rows with an identity matrix block at the end
#         new_rows = np.zeros((2, a.shape[1]), dtype=int)
#         new_rows[0, -2] = 1
#         new_rows[1, -1] = 1
        
#         # Append the new rows to the matrix
#         a = np.vstack((a, new_rows))
    
#     return a.tolist()

# # Initial matrix
# a = [[1, 2, 3], [2, 3, 4], [5, 6, 7]]
# # Expand the matrix for 1 iteration
# result = expand_matrix(a, 1)

# print(result)
a=np.array([
    [1,2,3],
    [2,3,4]
])

b=np.array([
    [2,3,4],
    [4,5,6]
])

c=np.vstack((a,b))
print(c)