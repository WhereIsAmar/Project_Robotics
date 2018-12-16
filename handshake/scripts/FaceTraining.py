import numpy as np
import cv2 as cv
from PIL import Image
import os
import pickle
import imutils
import sys
import os

# the file directory without the file name
base_dir = os.path.dirname(os.path.abspath(__file__))
images_dir = os.path.join(base_dir, "images")

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
recognizer = cv.face.LBPHFaceRecognizer_create()


def mkdir_p(path):
    try:
        os.makedirs(path)
    except:
        pass


def trainImages():
    cur_id = 0
    label_ids = {}  # dictionary
    x_train = []  # list of face images in numpy
    y_labels = []
    for root, dirs, files in os.walk(images_dir):
        for file in files:
            path = os.path.join(root, file)
            label = os.path.basename(os.path.dirname(path))

            if label in label_ids:
                pass
            else:
                label_ids[label] = cur_id
                cur_id += 1

            id_ = label_ids[label]

            img = Image.open(path).convert("L")  # L = grayscale
            image_arr = np.array(img)

            faces = face_cascade.detectMultiScale(image_arr, 1.05, 5)
            for (x, y, w, h) in faces:
                roi = image_arr[y:y+h, x:x+w]
                x_train.append(roi)
                y_labels.append(id_)

    with open("labels_pickle", 'wb') as file:
        pickle.dump(label_ids, file)

    recognizer.train(x_train, np.array(y_labels))
    recognizer.save("trainner.yml")
    print("Training was successful!")


def saveImagesFromCam(destination):
    dest = os.path.join(images_dir, destination)
    mkdir_p(dest)
    cam = cv.VideoCapture(int(os.getenv('FEED', 1)))

    # get the last pic num in the file dest
    picList = []
    for root, dir, files in os.walk(dest):
        for file in files:
            picList.append(str(file).split('.')[0])
    picList = list(map(int, picList))
    ctr = 1

    off = 0
    if len(picList):
        off = max(picList)

    while True:
        _, frame = cam.read()
        frame = imutils.resize(frame, width=400)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(frame, 1.05, 5)
        for (x, y, w, h) in faces:
            # print("face detected")
            roi_gray = gray[y:y + h, x:x + w]  # region of interest
            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex, ey, ew, eh) in eyes:
                print("eye detected", ctr)
                file_name = str(off + ctr) + ".jpg"
                ctr += 1
                cv.imwrite(os.path.join(dest, file_name), frame)

        cv.namedWindow("Frame", cv.WND_PROP_FULLSCREEN)
        cv.setWindowProperty("Frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
        cv.imshow('Frame', frame)

        # press q to stop.
        # ord convert char to its number.
        if (cv.waitKey(25) & 0xFF == ord('q')) or ctr > 300:
            break

    cam.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('usage: FaceTraining.py [name | train]')

    if sys.argv[1] == 'train':
        trainImages()
    else:
        saveImagesFromCam(sys.argv[1])
