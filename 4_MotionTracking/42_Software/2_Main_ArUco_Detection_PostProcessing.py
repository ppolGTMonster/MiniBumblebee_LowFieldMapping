####################################################################################################################
#   Script which analyzes the video file from the measurement and estimates the position of the robot joints
#   Therefore the marker are detected, the position is estimated and the joints are calculated
#
#   Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
#   MIT LICENSED  
#   Have fun guys!
####################################################################################################################




import cv2
import cv2.aruco as aruco
import numpy as np
import time


from MotionTracking_Classes import MarkerReferenceBoard, ReferenceBoard, MarkerOnRobot, Robot_ArucoMarker, Robot_Cybernetics
#from Functions_MatrixCalculation import meanRotationMatrix,buildTransformationMatrix
#from Functions_Localhost import sendViaLocalhost,waitOnMatlabCommand
from Functions_MotionTracking import estimateRefBoard, estimateRobotArucos,calculateCoorTrafo,calculateRealAnglesOfRobot,drawAxisAndMarker,safeImage,drawAxisAndMarker_Debugging
from Functions_PostProcessingSetUp import save_Data

#### 0) SetUp Motion Tracking ####
## 0.0) Measurement Information
# Amount of moved positions during the measurement (nends to fit the video!)
count_pos = 512
# Path of Videofile
video_file = 'D:/4_LocalMassStorage/LowBudegt Robot LOCAL/aufgezeichnete Videos/Messungen_24_04_17/Halbach FOV/V3/Halbach2.mp4'


## 0.1) Create Objects
# Check the class description - these objects are the most important stuff here ;)
# All estimated positions of all markers are stored in the attributes of the object.
# The most important calculations are done inside their methods
RefBoard = ReferenceBoard()
RobotMarker = Robot_ArucoMarker()
RobotCyber = Robot_Cybernetics()

## 0.2) SetUp Open CV
print('Setting Up OpenCV...')
# load calibration files
cam_dist = np.loadtxt('0_calibration/calibration_result/cam_dist.csv')
cam_mtx = np.loadtxt('0_calibration/calibration_result/cam_mtx.csv')
# load video file
cap = cv2.VideoCapture(video_file)

if not cap.isOpened():
	print('Cannot open Video File')
	exit()
print('...Video File loaded')
start_time_script = time.time()     # Starting time Analysis


## 0.4) SetUp Motion Tracking
# Number of frames, which are averaged for one measurement
# The video has 25fps. Every robot position is hold around 16seconds. Minus a bit tolerance -> Set 100 Frames = 4sec  
Avg_Frames = 100

# Starting Time, when the Motion Tracking is starting
fps = cap.get(cv2.CAP_PROP_FPS) # get fps
MT_Start = 0        #in seconds, Motion Tracking Start
MT_Time_Move = 4    #in seconds, Motion Tracking Time Duration one Move of Robot
MT_Time_Meas = 16   #in seconds, MT Time Duration one Measurement of Robot
MT_Time_SaveGap = 1   

# Recalculation to frames
MT_Start = 0
MT_Time_Move = np.floor(MT_Time_Move * fps)
MT_Time_Meas = np.floor(MT_Time_Meas * fps)
MT_Time_SaveGap = np.floor(MT_Time_SaveGap * fps)



#### 1) Start Motion Tracking   ####

print('Detecting Marker')

for i_steps in range(0,count_pos):

    print("   Iteration: ",i_steps+1," out of ", count_pos, " Steps"  )
    print('     - Detect Aruco Marker')
    start_time_iter = time.time()           # Starting time Analysis

    ## 1.2) Start Motion Tracking Routine
    for i_loop in range(0,Avg_Frames):

        ## 1.3) Get Image from Video File
        # Calculate start/end index of this step
        interval_start = MT_Start + (MT_Time_Move + MT_Time_Meas)*i_steps
        meas_middle = interval_start +  MT_Time_Move + MT_Time_Meas/2
        meas_start = meas_middle-Avg_Frames/2    
      
        # Set Frame & load Frame
        cap.set(cv2.CAP_PROP_POS_FRAMES, meas_start+i_loop)
        ret, image  = cap.read()
        
        # if frame is read correctly ret is true
        if not ret:
            print('    - Cant load frame. Exiting')
            break

        frame_remapped = image
        frame_remapped_gray = cv2.cvtColor(frame_remapped,cv2.COLOR_BGR2GRAY)

        ## 1.4) Detect Markers (using the openCV method "detectMarkers")
        Board_corners, Board_ids, Board_rejected = aruco.detectMarkers(frame_remapped_gray,RefBoard.ArucoDict, RefBoard.ID_list,parameters = RefBoard.ArucoParams)
        Robot_corners, Robot_ids, Robot_rejected = aruco.detectMarkers(frame_remapped_gray,RobotMarker.ArucoDict, RobotMarker.ID_list, parameters = RobotMarker.ArucoParams)

        # verify *at least* one ArUco marker was detected in Reference-Board (which is a bit more important than the robot markers)
        if len(Board_corners) > 0:

            ## 1.5) Estimate Position Board
            estimateRefBoard(RefBoard,Board_ids,Board_corners,cam_mtx,cam_dist)        
            
            ## 1.6) Estimate Position of Robot Markers  
            if len(Robot_corners)>0:
                
                estimateRobotArucos(RobotMarker,RefBoard,Robot_ids,Robot_corners,cam_mtx,cam_dist)
		

    #### 2) Interpretation of the tracked positions    ####
    cv2.destroyAllWindows() # close all open windows

    # "count_pos" much frames were stored in the lists (inside objects) -> start averaging now 
    ## 2.1) Averaging
    print('     - Detection finished - Start Averaging & kinematics')
    RobotMarker.calculateT_R_avg_CamSys()
    RefBoard.calculateT_R_avg_CamSys()

    ## 2.2) Calculate Coordinate Transformation
    calculateCoorTrafo(RobotMarker,RefBoard)

    ## 2.3) Calculate Joint Angles of the robot
    calculateRealAnglesOfRobot(RobotMarker,RefBoard,RobotCyber)

    #### 3) Save data as CSV file  ####
    print('     - Append Data to CSV-File')
    save_Data(RefBoard,RobotCyber,meas_start, meas_start+Avg_Frames)
    
    #### 4) Show analyzed frame ####
    ## 4.1) Create an image with markers  
    image_withRobot_final = drawAxisAndMarker(frame_remapped,
                                        RefBoard,RobotMarker,cam_mtx,cam_dist,Board_corners,Board_ids,Robot_corners,
                                        Robot_ids,1)
    cv2.imshow("Frame",image_withRobot_final) # start the window
    key = cv2.waitKey(10) # this is necessairy, but dont know why. If is not added the window is not shown

    ## 4.2) Safe the image 
    safeImage(image_withRobot_final,i_steps)

  
    #### 5) Prepare the next Iteration/Position -> Delete everything! ####
    RobotMarker.resetMarkers()  # Clear all scanned markers on Robot
    # Clear Aruco Board. Camera is standing still ->
    # -> Position is not changing (moving averaging could be done here)
    # However clear the list, but keep the calculated former average as the first value 
    RefBoard.resetBoard()
    
    print("     - Elapsed time: ", time.time()-start_time_iter, " sec")

  
    
#### 6) Finishing the whole analysis ####
cv2.destroyAllWindows()

print("Evaluation completed. Elapsed time: ", time.time() - start_time_script, " sec")
    



    
