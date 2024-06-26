Steps to train a model:
1. Collect images samples (either sim or real).
2. Either open or add the images to AUV2024 workspace on Roboflow.
    a. Ask a lead for the McGill Robotics account information.
3. Upload your data and label it using bounding boxes.
    a. The train/valid/test split does not matter.
4. Generate a new version of the dataset. 
    a. Do NOT add any pre-processing/augmentation. We already do that in the training notebook.
5. Change the parameters in the training notebook as needed.
6. Run training notebook.
    a. There is also the option of running training.py. Read the How-to section at the top of the file.