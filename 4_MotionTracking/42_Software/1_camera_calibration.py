####################################################################################################################
#   Script that calculates the distortion coefficiants for the camera
#   Therefore it uses the already exported images (via 0_images_extraction.py)
#
#   Most of this code is based on a tutorial of openCV:
#       *https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
#
#   Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
#   MIT LICENSED  
#   Have fun guys!
####################################################################################################################

import numpy as np
import cv2
import glob

print('Calibration starts...')
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Configuration of the checkered calibration Board (in this example 9 height, 13 width).
# Take care: Python counting start from 0
nx = 9-1
ny = 13-1


## prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((nx*ny,3), np.float32)
objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)
objp = objp
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('0_calibration/calibration_result/img_calibration/*.jpg')    #List of all images with *jpg ending in this folder

for fname in images:
    ## get the video for processing
    print(fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Convert the colored image to black/white (higher contrast)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (nx,ny), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (nx,ny), corners2, ret)
        print("Success")
        img2 = cv2.resize(img, (800, 640))
        cv2.imshow('img', img2)
        cv2.waitKey(2)


## Start calibration
camera_mtx_init = np.array([[ 4000,    0, gray.shape[1]/2],
                            [   0, 4000, gray.shape[0]/2],
                            [   0,    0,           1]])
dist_coeff_init = np.zeros((5,1))

flags = (cv2.CALIB_USE_INTRINSIC_GUESS + 
         cv2.CALIB_FIX_PRINCIPAL_POINT + 
         cv2.CALIB_FIX_ASPECT_RATIO + 
         cv2.CALIB_ZERO_TANGENT_DIST + 
         cv2.CALIB_FIX_K1 + 
         cv2.CALIB_FIX_K2 + 
         cv2.CALIB_FIX_K3)

print('Start: cv2.calibrateCamera')
# the main function to calculate calibration matrix
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None) 
## CALIBRATION FINISHED
print("Camera Calibrated: ", ret)
print('mtx:\n', mtx, '\n dist:', dist.T)

# save results as csv
np.savetxt("0_calibration/calibration_result/cam_mtx.csv", mtx)
np.savetxt("0_calibration/calibration_result/cam_dist.csv", dist)



## CALCULATION RE-PROJECTION ERROR
# The closer to zero the better
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
cv2.destroyAllWindows()
print( "re-projection error: {}".format(mean_error/len(objpoints)) )
print('------------')
print('Calibration done!')


