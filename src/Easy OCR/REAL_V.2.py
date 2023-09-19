import cv2

# read haarcascade for number plate detection
cascade = cv2.CascadeClassifier('haarcascade_russian_plate_number.xml')

# Open the default camera (0) or provide the camera index if you have multiple cameras
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # convert input frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect license number plates
    plates = cascade.detectMultiScale(gray, 1.1, 4)
    print('Detected license plates:', len(plates))

    # loop over all plates
    for (x, y, w, h) in plates:

        # draw bounding rectangle around the license number plate
        cv2.rectangle(frame, (x, y - 30), (x + w + 30, y + h), (0, 0, 255), 2)
        cv2.putText(frame, "tabain", (x, y - 35),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 1)

        gray_plates = gray[y - 30:y + h, x:x + w + 30]
        color_plates = frame[y:y + h, x:x + w]

        # save number plate detected (you can remove this line if you don't need to save images)
        cv2.imwrite('Numberplate.jpg', gray_plates)

    cv2.imshow('Number Plate Image', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
