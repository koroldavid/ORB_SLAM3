import numpy as np

np.set_printoptions(suppress=True, precision=8)

def calculate_rotation_difference(R1, R2):
    # Calculate the inverse of R1
    R1_inv = np.linalg.inv(R1)
    
    # Calculate the difference: R_diff = R1_inv * R2
    R_diff = np.dot(R1_inv, R2)
    
    return R_diff

def calculate_R2(R1, R_diff):
    # Calculate R2 = R1 * R_diff
    R2 = np.dot(R1, R_diff)
    return R2

def calculate_t_diff(R1, R2, t1, t2):
    # Calculate the inverse of R1
    R1_inv = np.linalg.inv(R1)
    
    # Calculate the translation difference: t_diff = R1_inv * (t2 - t1)
    t_diff = np.dot(R1_inv, (t2 - t1))
    
    return t_diff
    
# Example: Define rotation matrices for both cameras
R_c1 = np.array([
    [ 0.9878196 ,  0.05763929, -0.14453427],
    [-0.05697001,  0.99828413, -0.00188424],
    [ 0.14417766,  0.01032024,  0.989498  ]
])

R_c2 = np.array([
    [ 0.99419197,  0.04829369, -0.09617717],
    [-0.04770324,  0.99882598,  0.00843047],
    [ 0.09647139, -0.00379355,  0.99532853]
])

t1 = np.array([0.0, 0.0, 0.0])
t2 = np.array([-0.0255, 0.0, 0.0])

# Calculate the relative rotation matrix R_c1_c2
R_diff = calculate_rotation_difference(R_c1, R_c2)
print("Relative Rotation Matrix R_diff:")
print(R_diff)

print("")
print("calculate_t_diff T_diff:")
print(calculate_t_diff(R_c1, R_c2, t1, t2))