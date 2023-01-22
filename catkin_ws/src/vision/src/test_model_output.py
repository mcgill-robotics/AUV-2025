#!/usr/bin/env python3
from ultralytics import YOLO
import os
import cv2

if __name__ == '__main__':
    pwd = os.path.realpath(os.path.dirname(__file__))
    img_path = pwd + "/data/augmented/train/images/img3.png"
    img = cv2.imread(img_path)
    # Load a model
    model = YOLO(pwd + "/model.pt")  # load a pretrained model (recommended for training)
    # Use the model
    pred = model(img)
    for results in pred:
        box = results.boxes
        print(box)
        print("\nprediction:" + str(list(box.xywh)))
        print("\nprediction:" + str(list(box.conf)))
        print("\nprediction:" + str(list(box.cls)))
