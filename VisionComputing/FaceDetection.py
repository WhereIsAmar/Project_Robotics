import cv2 as cv
import pickle
import imutils  # to speed up the FPS
import argparse

# OpenCV is BGR
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
FONT = cv.FONT_HERSHEY_SIMPLEX

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
recognizer = cv.face.LBPHFaceRecognizer_create()
recognizer.read("trainner.yml")


labels = {}
with open("labels_pickle", 'rb') as file:
    old_labels = pickle.load(file)
    labels = {v: k for k, v in old_labels.items()}


def readargs():
    parser = argparse.ArgumentParser(description='face detection')
    parser.add_argument('--feed', default=1, type=int, help="camera feed #")
    parser.add_argument('--accuracy', default=40, type=int,
                        help="threshold for displaying names")
    parser.add_argument('--full_screen', default=False,
                        type=bool, help="Display in full screen")
    return parser.parse_args()


def label_person(frame, roi_gray, pt, accuracy):
    """ if person is recognized, show name """
    id_, accuracy = recognizer.predict(roi_gray)
    if accuracy < args.accuracy:
        return

    # print(labels[id_], accuracy)
    name = labels[id_] + " " + str(accuracy)
    # name = labels[id_]
    cv.putText(frame, name, pt, FONT, 1, WHITE, 2, cv.LINE_AA)


def bound_eyes(roi_gray, roi_color):
    """ draw bounding boxes around eyes """
    eyes = eye_cascade.detectMultiScale(roi_gray, 1.05, 5)
    for (ex, ey, ew, eh) in eyes:
        cv.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), GREEN, 2)


def bound_faces(args, frame):
    """ draw bounding boxes around faces """
    faces = face_cascade.detectMultiScale(frame, 1.05, 5)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    for (x, y, w, h) in faces:
        cv.rectangle(frame, (x, y), (x + w, y + h), BLUE, 2)

        roi_gray = gray[y:y + h, x:x + w]  # region of interest
        roi_color = frame[y:y + h, x:x + w]
        bound_eyes(roi_gray, roi_color)
        label_person(frame, roi_gray, (x, y), args.accuracy)


def main(args):
    cam = cv.VideoCapture(args.feed)
    if args.full_screen:
        cv.namedWindow("Frame", cv.WND_PROP_FULLSCREEN)
        cv.setWindowProperty("Frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    else:
        cv.namedWindow("Frame", cv.WINDOW_NORMAL)

    while(True):
        ret, frame = cam.read()
        frame = imutils.resize(frame, width=400)
        bound_faces(args, frame)

        cv.imshow('Frame', frame)

        # press q to stop.
        # ord convert char to its number.
        if cv.waitKey(25) & 0xFF == ord('q'):
            break

    cam.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    args = readargs()
    main(args)
