import numpy as np
from scipy.spatial.transform import Rotation as sciROT 


def meanRotationMatrix(R1_list):
    #For better averaging performance the matrices are converted to quaternions and than averaged

    # Convert to Quaternion
    R1_temp = sciROT.from_matrix(R1_list).as_quat()
    # Average them
    R_mean_quat=np.mean(R1_temp,axis =0)

    # Convert it back to a Rot-Matrix
    R_mean = sciROT.from_quat(R_mean_quat)    
    return R_mean.as_matrix()

def getEulerAngles(R): # Convention ZYX
    # Calculate the Euler Angles from the Rot-Matrix. Output in Order 'ZYX'

    if R.shape == (4,4):
        R,_=  split_Affine_Matrix(R) # If R is a homog Coordinate (4x4) -> get Rot-Matrix

    R1_temp = sciROT.from_matrix(R)

    return R1_temp.as_euler('zyx',degrees=False)

def getRotMatfromEuler(euler):
    # Calculate Rot-Matrix from Euler-Angles in Order 'ZYX'
    
    R = sciROT.from_euler('zyx',euler,degrees=False)
    
    return R.as_matrix()


def getQuaternion(R): 
    # Calculate Quaternion out f Rot-Matrix
    
    if R.shape == (4,4):
        R,_=  split_Affine_Matrix(R)

    R1_temp = sciROT.from_matrix(R)

    return R1_temp.as_quat()

def split_Affine_Matrix(A):
    # Split a homog coordinate (4x4) into a translation Vector (3x1) and a Rot-Matrix (3x3)
    T = A[:3, 3]
    R = A[:3, :3]

    return R, T

def buildTransformationMatrix(R, T):
    # Combine Rotation and Translation into one homog Coordinate
    if  T.shape != (3, 1):
        T_set = T.reshape((3, 1))   # transpose it
    else:
        T_set = T                   #Dimension is ok

    M1 = np.concatenate((R,T_set),1)
    M2 = np.array([[0, 0, 0, 1]])
    M = np.concatenate((M1,M2),0)
    return M
