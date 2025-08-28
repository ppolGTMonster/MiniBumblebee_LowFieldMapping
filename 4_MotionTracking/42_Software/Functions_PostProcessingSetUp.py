import csv
import os
from Functions_MatrixCalculation import getEulerAngles


def save_Data(RefBoard,RC,frame_start, frame_end):
    # save data to CSV file
    
    RefBoard_euler = getEulerAngles(RefBoard.R_avg) # Euler Angl ein order 'zyx'
    RefBoard.T_avg # Vector with 3 elements
   

    data_to_add = [ frame_start, frame_end,
                    RefBoard_euler[0], RefBoard_euler[1], RefBoard_euler[2],
                    RefBoard.T_avg[0], RefBoard.T_avg[1], RefBoard.T_avg[2],
                    RC.alpha0, RC.alpha1, RC.alpha2, RC.alpha3]
    
    adding_Data_to_CSV(data_to_add)

def adding_Data_to_CSV(data):
    # save the new line to the CSV file

    filename = 'MotionTracking_Log.csv'
    folder = '2_SaveFolder'
    filename = os.path.join(folder,filename)
    # check, if the file already exists
    file_exists = os.path.isfile(filename)

    header_line = ["FrameStart","FrameEnd",
                   "RefB_EulerZ","RefB_EulerY","RefB_EulerX",
                   "RefB_TvecX","RefB_TvecY","RefB_TvecZ",
                   "Robot_alpha0","Robot_alpha1","Robot_alpha2","Robot_alpha3"]

    with open(filename,'a+',newline='') as file:
        writer = csv.writer(file)

        # File is generated new: Add Headerline
        if not file_exists:
            writer.writerow(header_line)

        writer.writerow(data) #save the new data line