import easyocr

# REFER
# https://www.tutorialspoint.com/how-to-detect-license-plates-using-opencv-python

# import required libraries
import cv2
import numpy as np

# Read input image
img = cv2.imread("plt_2.jpg")

# convert input image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# read haarcascade for number plate detection
cascade = cv2.CascadeClassifier('haarcascade_russian_plate_number.xml')

# Detect license number plates
plates = cascade.detectMultiScale(gray, 1.1, 4)
print('เจอทะเบียน:', len(plates))

# loop over all plates
for (x,y,w,h) in plates:
   
   # draw bounding rectangle around the license number plate
   cv2.rectangle(img, (x,y-30), (x+w+30, y+h), (0,0,255), 2)
   cv2.putText(img, "tabain", (x,y-35),
               cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,255, 0),1)
   gray_plates = gray[y-30:y+h, x:x+w+30]
   color_plates = img[y:y+h, x:x+w]
   
   # save number plate detected
   cv2.imwrite('Numberplate.jpg', gray_plates)


   reader = easyocr.Reader(['en','th']) # this needs to run only once to load the model into memory
   result = reader.readtext('Numberplate.jpg')

   print(result[0][1])

   #cv2.waitKey(0)
cv2.destroyAllWindows()

