#!/usr/bin/env python3
from ultralytics import YOLO
import os

if __name__ == '__main__':
    #hacky solution since relative paths in the .yaml file do not behave as expected:
    #generates .yaml with absolute path to datasets
    pwd = os.path.realpath(os.path.dirname(__file__))
    with open(pwd + '/data.yaml', 'w+') as f:
        f.write("train: " + pwd + "/data/augmented/train/images\n")
        f.write("test: " + pwd + "/data/augmented/test/images\n")
        f.write("val: " + pwd + "/data/augmented/val/images\n")
        f.write("nc: 1\n")
        f.write('names: ["lane_marker"]')

    # Load a model
    model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)
    # Use the model
    model.train(data=pwd+"/data.yaml", epochs=3, device=0, batch=16, degrees=360, flipud=0.5, fliplr=0.5, perspective=0.001, translate=0.5, scale=0.75, pretrained=True, task='detect')  # train the model
    model.val()
