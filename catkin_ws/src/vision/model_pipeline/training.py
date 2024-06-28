# Only use this file if all the folders and augmentations are set-up (including runs folder).
# This file is meant to avoid using the ugly and annoying jupyter notebook's output format.  

import os
import shutil
from ultralytics import YOLO
import torch

is_front_camera_training = False # Change it to False if training for down camera.
################################# OS PARAMETERS ##################################
os_name = os.name
# Windows = nt, [Linux, Apple] = posix. 
os_path = "\\" if os_name == "nt" else "/"  
##################################################################################
############################## ROBOFLOW  PARAMETERS ##############################
roboflow_api_key = "MJQUATZvpcKoBxRjLuXx"
roboflow_workspace_name = "auv2024"
dataset_version_download = "yolov8"
if is_front_camera_training:
     roboflow_project_name = "front-camera-sim"
     roboflow_project_version = 6
else:
     roboflow_project_name = "down-camera-sim"
     roboflow_project_version = 1
##################################################################################
############################## TRAINING  PARAMETERS ##############################
if is_front_camera_training:
     target_classes = ["bouy", "gate", "octagon-table"]
     model_save_filename = "best_AUV_sim_front_camera_model.pt"
else: 
     target_classes = ["bin", "lane-marker", "octagon-table"]
     model_save_filename = "best_AUV_sim_down_camera_model.pt"
model_name = "yolov8n.pt"
train_test_val_split = (0.7, 0.2, 0.1)
epoch_increments = 200
batch_size = -1 # Auto Mode (60% GPU Memory): Use batch=-1 to automatically 
                # adjust batch size for approximately 60% CUDA memory 
                # utilization.
workers = 1
cache = False
pretrained = True
hsv_h = 0.001
hsv_s = 0.1
hsv_v = 0.1
translate = 0.1
scale = 0.2
fliplr = 0.2
mosaic = 0.5
erasing = 0.1
crop_fraction = 0.5
##################################################################################
######################### CUSTOM AUGMENTATION PARAMETERS #########################
colorAugmentProb = 0.3
noiseAugmentProb = 0.3
resolutionAugmentProb = 0.3
contrastAugmentProb = 0.3
blurAugmentProb = 0.3
brightnessAugmentProb = 0.3
##################################################################################
########################## YOLO AUGMENTATION PARAMETERS ##########################
# Currently not being used.
degrees = 360
flipud = 0.5
fliplr = 0.5
max_perspective_change = 0.001
max_translate = 0.1
max_scale_change = 0.3
mosaic = 0.5
mixup = 0.5
##################################################################################

if __name__ == "__main__":
     data_yaml_file_absolute_path = os.path.abspath("data.yaml")

     model = YOLO("yolov8n.pt") # load a pretrained model.

     # Start the training process.
     while True:
          try:
               model.train(
                    data=data_yaml_file_absolute_path,
                    epochs=epoch_increments,
                    batch=batch_size,
                    pretrained=pretrained,
                    task="detect",
                    cache=cache,
                    workers=workers,
                    hsv_h=hsv_h,
                    hsv_s=hsv_s,
                    hsv_v=hsv_v,
                    translate=translate,
                    scale=scale,
                    fliplr=fliplr,
                    mosaic=mosaic,
                    erasing=erasing,
                    crop_fraction=crop_fraction
               )
               shutil.copyfile(f"runs{os_path}detect{os_path}train{os_path}weights{os_path}best.pt", model_save_filename)
          except RuntimeError as e:
               print(f"Caught a RuntimeError: {e}.")
               break  # Break out of the loop if an error occurs to prevent infinite loop.
