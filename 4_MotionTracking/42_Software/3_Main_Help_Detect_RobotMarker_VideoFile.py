####################################################################################################################
#   Script to check if all ArUco marker can be detected
# 	Therefore a sample video file is loaded, the marker are detected/estimated 
#	The result is shown as a frame
#
#   Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
#   MIT LICENSED  
#   Have fun guys!
####################################################################################################################

import cv2
import cv2.aruco as aruco
import numpy as np

# Length of the Aruco marker in Meter
Robot_Aruco_length = 0.03 		# Mounted on the robot 
RefBoard_Aruco_length = 0.08	# Reference Board

print('Seting Up System...')

# Video File, which should be checked
video_file = 'D:/4_LocalMassStorage/LowBudegt Robot LOCAL/aufgezeichnete Videos/Messungen_24_03_04/Messung1/Messung1_24_03_04.mp4'


# Setting up the Aruco-parameter 
Robot_ArucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
Robot_ArucoParams = aruco.DetectorParameters_create()
RefBoard_AurcoDict = aruco.Dictionary_get(aruco.DICT_5X5_50)
RefBoard_ArucoParams = aruco.DetectorParameters_create()

print('Setting Up OpenCV')
# Load calibration file
cam_dist = np.loadtxt('0_calibration/calibration_result/cam_dist.csv')
cam_mtx = np.loadtxt('0_calibration/calibration_result/cam_mtx.csv')

# Load Video File
cap = cv2.VideoCapture(video_file)

if not cap.isOpened():
    print('Cannot open video file')
    exit()

count_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) # get sum of frames from video
print('...Video File loaded')
print('Detecting Marker')

for i_step in range(0,count_frames):

	# Set Frame & load Frame
	cap.set(cv2.CAP_PROP_POS_FRAMES, i_step)
	ret, image = cap.read() # Capture one frame
	
	# if frame is read correctly ret is true
	if not ret:
		print('Cant receive frame. Exiting')
		break

	frame_remapped = image
	frame_remapped_gray = cv2.cvtColor(frame_remapped,cv2.COLOR_BGR2GRAY) # Change color from color to black/white

	# Detect Markers (both for Robot & for Reference Board)	
	Robot_corners, Robot_ids, Robot_rejected = aruco.detectMarkers(frame_remapped_gray,Robot_ArucoDict, parameters =Robot_ArucoParams)
	RefBoard_corners, RefBoard_ids, RefBoard_rejected = aruco.detectMarkers(frame_remapped_gray,RefBoard_AurcoDict, parameters =RefBoard_ArucoParams)


	# verify *at least* one Robot-ArUco marker was detected
	if len(Robot_corners) > 0:
	
    	# Draw the detected Markers and the individual coordinate system of every marker 
		for i in range(0,len(Robot_ids)):

			# estimatePoseSingleMarkers = returns the coordinate system of the marker (the rotation rvec and translation tvec)		
			rvec,tvec,markerPoints = aruco.estimatePoseSingleMarkers(Robot_corners[i],Robot_Aruco_length,cam_mtx,cam_dist) 

			#Draw the marker-box + id + coor-Sys on he frame
			frame_remapped = aruco.drawAxis(frame_remapped, cam_mtx,cam_dist,rvec, tvec, Robot_Aruco_length)
			frame_remapped = aruco.drawDetectedMarkers(frame_remapped, Robot_corners,Robot_ids)

		
	if len(RefBoard_corners) > 0:
	
		for i in range(0,len(RefBoard_ids)):

			rvec,tvec,markerPoints = aruco.estimatePoseSingleMarkers(RefBoard_corners[i],RefBoard_Aruco_length,cam_mtx,cam_dist) 

			frame_remapped = aruco.drawAxis(frame_remapped, cam_mtx,cam_dist,rvec, tvec, RefBoard_Aruco_length)
			frame_remapped = aruco.drawDetectedMarkers(frame_remapped, RefBoard_corners,RefBoard_ids,(255,255,0))

		
		
	# All detected markers are added to the frame -> Show the image
	cv2.imshow("Frame",frame_remapped)
	key = cv2.waitKey(20) # Wait 20ms, for some reason this is necessairy otherwise the window is not opening sometimes
    

cv2.destroyAllWindows() # close everything
    
    
    