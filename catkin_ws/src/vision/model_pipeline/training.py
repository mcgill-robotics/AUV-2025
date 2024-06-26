# How-to use this file:
#    1. This file is used ONLY for training.
#       Training will only work if you have  
#       all the images have been pre-processed
#       and have been moved to right folders.
#       Training with this file helps to avoid
#       Jupyter's cell output bs.
#    2. Some parameters need to be specified by
#       the user based on your computer
#       performance. Make sure to test different
#       values before training with one for a 
#       long time. You might get an error in 
#       the middle of training if the parameters
#       are too high for your computer (e.g., 
#       epoch_increments).
#    3. Please make sure you have CUDA available.
#       It will take FOREVER to train if you 
#       don't have it. Trust me.
#    4. If you have problems with training,
#       consider lowering .
#    5. No need to change anything outside the 
#       user-defined parameters box.

import os 
import shutil
from ultralytics import YOLO
import torch

if not (torch.cuda.is_available() and torch.cuda.device_count()):
    raise RuntimeError("CUDA is NOT available. Please ensure that your system has a compatible GPU and the necessary drivers are installed.")

############################## USER-DEFINED PARAMETERS ##############################
# Windows = nt, [Linux, Apple, Google Colab] = posix. 
os_name = os.name
is_front_camera_training = True         # False = training for down camera. 
model_name = "yolov8n.pt"
epoch_increments = 200                  # Ideally, make it as high as your computer
                                        # can handle.
workers = 2
cache = False
pretrained = True
#####################################################################################

# Training parameters
if is_front_camera_training:
     target_classes = ["gate", "buoy", "octagon-table"]
     model_save_filename = "best_AUV_sim_front_camera_model.pt"
else: 
     target_classes = ["lane-marker", "bin", "octagon-table"]
     model_save_filename = "best_AUV_sim_down_camera_model.pt"

if os_name == "nt":
     weights_file_path = "runs\\detect\\train\\weights\\best.pt"
else:
     weights_file_path = "runs/detect/train/weights/best.pt"

batch_size = -1 # YOLO can compute how large the batch_size can be based on your
                # computer when batch_size = -1.


if __name__ == "__main__":
     data_yaml_file_absolute_path = os.path.abspath("data.yaml")

     model = YOLO(model_name) # Load model.

     # Start the training process.
     while True:
          try:
               model.train(
                    data=data_yaml_file_absolute_path,
                    epochs=epoch_increments,
                    batch=batch_size,
                    pretrained=pretrained,
                    task='detect',
                    cache=cache,
                    workers=workers
               )
               shutil.copyfile(weights_file_path, model_save_filename)
          except RuntimeError as e:
               print(f"Caught a RuntimeError: {e}")
               break  # Break out of the loop if an error occurs to prevent infinite loop.
