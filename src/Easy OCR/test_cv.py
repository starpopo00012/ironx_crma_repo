import cv2
import easyocr
import numpy as np

def capture_and_save_image(file_path):
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
    cv2.imwrite("num_org.jpg", frame)

    img = cv2.imread("num_org.jpg")

    # convert input image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

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

    # Release the camera and close the OpenCV window
    cap.release()
    cv2.destroyAllWindows()

    print(f"Image saved as {file_path}")

# Usage example:
file_path = "num_org.jpg"
capture_and_save_image(file_path)

reader = easyocr.Reader(['en','th']) # this needs to run only once to load the model into memory
result = reader.readtext(file_path )

print(result[0][1])



