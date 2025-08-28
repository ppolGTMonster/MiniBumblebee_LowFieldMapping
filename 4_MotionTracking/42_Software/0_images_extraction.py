####################################################################################################################
#   Script that reads the calibration video & extracts num_img equidistant images out of it for calibration
#
#   Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
#   MIT LICENSED  
#   Have fun guys!
####################################################################################################################

import os, cv2

# number of needed images
num_img = 20 # we recommend to set 300 for good results

# video path for conversion
for f in os.listdir('0_calibration/'):
    if f.startswith('calibration_video.'):
        video_path = "0_calibration/" + f

# images saving path
path_img_saving = "0_calibration/calibration_result/img_calibration/"

for f in os.listdir(path_img_saving):
    os.remove(path_img_saving + f)


# Start the Export of images from the video file
cap = cv2.VideoCapture(video_path)
num_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) #get number of overall frames 
distance = num_frames//num_img
frame_id = 1
print ("Extracting images...")


while(cap.isOpened()):
    ret, frame = cap.read() 
    if (ret is not True):
            break
    if num_frames <= num_img:
        filename = path_img_saving + "image_" +  frame_id + ".jpg"
        cv2.imwrite(filename, frame)
    elif frame_id<=num_img*distance and frame_id%distance == 0:
        filename = path_img_saving + "image_" +  str(int(frame_id//distance)) + ".jpg"
        cv2.imwrite(filename, frame)
    frame_id += 1
cap.release()
print ("Extraction done!") 