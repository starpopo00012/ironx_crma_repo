#!/usr/bin/env python3

import rospy
import time
import cv2
import numpy as np
import easyocr


from std_msgs.msg import *

def capture_and_save_image(data):
    # Open the default camera (usually the built-in webcam)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Capture a single frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not capture an image.")
        return

    # Save the captured frame to the specified file
    time.sleep(3)
    cv2.imwrite("/home/por/catkin_ws/src/mic_cam/src/num_org.jpg", frame)
    cap.release()

    img = cv2.imread("num_org.jpg")

    # convert input image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    try:

        cascade = cv2.CascadeClassifier('haarcascade_russian_plate_number.xml')

        plates = cascade.detectMultiScale(gray, 1.1, 4)

        # loop over all plates
        for (x,y,w,h) in plates:

            # draw bounding rectangle around the license number plate
            cv2.rectangle(img, (x,y-30), (x+w+30, y+h), (0,0,255), 2)
            cv2.putText(img, "tabain", (x,y-35),
                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,255, 0),1)
            gray_plates = gray[y-30:y+h, x:x+w+30]
            color_plates = img[y:y+h, x:x+w]

            # save number plate detected
            cv2.imwrite('num_org.jpg', gray_plates)
            # Release the camera and close the OpenCV windows
            cap.release()
            cv2.destroyAllWindows()
    except:
        cap.release()

    reader = easyocr.Reader(['en','th']) # this needs to run only once to load the model into memory
    result = reader.readtext('num_org.jpg')
    print(result[0][1])
    cap.release()
    return (f"{result[0][1]}")

def handle_get_plate(req):
    plate_number = "ABC"
    rospy.loginfo("Generated plate number: %s", plate_number)
    return ("ทะเบียน: %s" %plate_number)

if __name__ == "__main__":
    rospy.init_node('get_plate')
    rospy.Subscriber('get_plate', String, capture_and_save_image)
    rospy.spin()







 


            
    