#!/usr/bin/env python

import cv2 as cv
import pickle
import imutils  # to speed up the FPS
import argparse
import rospy
import std_msgs
import os

# OpenCV is BGR
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
FONT = cv.FONT_HERSHEY_SIMPLEX


def path(name):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_dir, name)


class FaceDetector(object):

    def __init__(self, accuracy_threshold=80, publisher=None):
        self.face_cascade = cv.CascadeClassifier(path('haarcascade_frontalface_default.xml'))
        self.eye_cascade = cv.CascadeClassifier(path('haarcascade_eye.xml'))
        self.recognizer = cv.face.LBPHFaceRecognizer_create()
        self.recognizer.read(path("trainner.yml"))
        self.accuracy_threshold = accuracy_threshold
        self.publisher = publisher
        self._reload()

    def _reload(self):
        self.labels = {}
        with open(path("labels_pickle"), 'rb') as file:
            old_labels = pickle.load(file)
            self.labels = {v: k for k, v in old_labels.items()}

    def _bound_eyes(self, roi_gray, roi_color):
        """ draw bounding boxes around eyes """
        eyes = self.eye_cascade.detectMultiScale(roi_gray, 1.05, 5)
        for (ex, ey, ew, eh) in eyes:
            cv.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), GREEN, 2)

    def _label_person(self, frame, roi_gray, pt):
        """ if person is recognized, show name """
        id_, accuracy = self.recognizer.predict(roi_gray)
        if accuracy < self.accuracy_threshold:
            return

        name = self.labels[id_]
        if self.publisher:
            self.publisher.publish(std_msgs.msg.String(name))

        cv.putText(frame, name, pt, FONT, 1, WHITE, 2, cv.LINE_AA)

    def detect(self, frame):
        """ draw bounding boxes around faces """
        faces = self.face_cascade.detectMultiScale(frame, 1.05, 5)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        for (x, y, w, h) in faces:
            cv.rectangle(frame, (x, y), (x + w, y + h), BLUE, 2)

            roi_gray = gray[y:y + h, x:x + w]  # region of interest
            roi_color = frame[y:y + h, x:x + w]
            self._bound_eyes(roi_gray, roi_color)
            self._label_person(frame, roi_gray, (x, y))


def readargs():
    parser = argparse.ArgumentParser(description='face detection')
    parser.add_argument('--feed', default=1, type=int, help="camera feed #")
    parser.add_argument('--accuracy', default=40, type=int,
                        help="threshold for displaying names")
    parser.add_argument('--full_screen', default=False,
                        type=bool, help="Display in full screen")
    return parser.parse_args()


def main(args):
    cam = cv.VideoCapture(args.feed)

    pub = rospy.Publisher('handshake/play', std_msgs.msg.String, queue_size=1)
    rate = rospy.Rate(5)  #Hz

    if args.full_screen:
        cv.namedWindow("Frame", cv.WND_PROP_FULLSCREEN)
        cv.setWindowProperty("Frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    else:
        cv.namedWindow("Frame", cv.WINDOW_NORMAL)

    fd = FaceDetector(accuracy_threshold=args.accuracy, publisher=pub)
    while not rospy.is_shutdown():
        ret, frame = cam.read()
        frame = imutils.resize(frame, width=400)
        fd.detect(frame)

        cv.imshow('Frame', frame)

        # press q to stop.
        # ord convert char to its number.
        if cv.waitKey(25) & 0xFF == ord('q'):
            break

        rate.sleep()

    cam.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('handshake_facedetector')
    args = readargs()
    main(args)
