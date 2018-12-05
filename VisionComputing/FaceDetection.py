import cv2 as cv
import pickle
import imutils #to speed up the FPS

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
recognizer = cv.face.LBPHFaceRecognizer_create()
recognizer.read("trainner.yml")

labels = {}
with open("labels_pickle",'rb') as file:
    old_labels = pickle.load(file)
    labels = {v:k for k,v in old_labels.items()}


cam = cv.VideoCapture(1)

while(True):
    ret, frame = cam.read()
    frame = imutils.resize(frame, width=400)
    gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(frame, 1.05, 5)
    for (x, y, w, h) in faces:
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]  # region of interest
        roi_color = frame[y:y + h, x:x + w]
        eyes = eye_cascade.detectMultiScale(roi_gray,1.05,5)
        for (ex, ey, ew, eh) in eyes:
            cv.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)
            id_, accuracy = recognizer.predict(roi_gray)
            if accuracy >= 40:
                # print(labels[id_], accuracy)
                font = cv.FONT_HERSHEY_SIMPLEX
                # name = labels[id_] + " " + str(accuracy)
                name = labels[id_]
                color = (255, 255, 255)
                cv.putText(frame, name, (x, y), font, 1, color, 2, cv.LINE_AA)

    cv.namedWindow("Frame", cv.WND_PROP_FULLSCREEN)
    cv.setWindowProperty("Frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    cv.imshow('Frame', frame)
    # press q to stop.
    # ord convert char to its number.
    if cv.waitKey(25) & 0xFF == ord('q'):
        break
cam.release()
cv.destroyAllWindows()
