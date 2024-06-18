import os, shutil
from os import listdir
from os.path import isfile, join
import cv2
import albumentations as A
import copy
import random
import numpy as np
from ultralytics import YOLO
import torch
from roboflow import Roboflow


# Roboflow parameters
roboflow_api_key = "MJQUATZvpcKoBxRjLuXx"
roboflow_workspace_name = "auv2024"
roboflow_project_name = "front-cam-felps"
roboflow_project_version = 5

# Training parameters
target_classes = ["Buoy", "Gate"]
train_test_val_split = (0.7, 0.2, 0.1)
model_save_filename = "best_AUV_sim_front_cam_model.pt"
epoch_increments = 200 # save the model weights to google drive every `epoch_increments` epochs
batch_size = -1
epoch_increments = 200 # save the model weights to google drive every `epoch_increments` epochs
batch_size = -1

# Custom augmentation parameters
colorAugmentProb = 0.3
noiseAugmentProb = 0.3
resolutionAugmentProb = 0.3
contrastAugmentProb = 0.3
blurAugmentProb = 0.3
brightnessAugmentProb = 0.3

# WARNING: do not change these "randomly", check the YoloV8 docs for what these parameters affect before modifying (these values have worked well)
# See the last cell for where these parameters are used in the model
degrees = 360
flipud = 0.5
fliplr = 0.5
max_perspective_change = 0.001
max_translate = 0.1
max_scale_change = 0.3
mosaic = 0.5
mixup = 0.5


if __name__ == "__main__":
     data_yaml_file_absolute_path = os.path.abspath("data.yaml")

     model = YOLO("yolov9c.pt") #load a pretrained model

     # Start the training process
     while True:
          try:
               model.train(
                    data=data_yaml_file_absolute_path,
                    epochs=epoch_increments,
                    # device=0, # --> it can automatically detect the device available
                    batch=batch_size,
                    pretrained=True,
                    task='detect',
                    cache=False,
                    workers=2,
               )
               shutil.copyfile("runs\\detect\\train\\weights\\best.pt", model_save_filename)
          except RuntimeError as e:
               print(f"Caught a RuntimeError: {e}")
               break  # Break out of the loop if an error occurs to prevent infinite loop
