# Steps to train a model
1. Collect image samples (either sim or real).
    1. Ideally, you should collect at least ~50 raw images for each class. 
3. Either open or add the images to AUV2024 workspace on Roboflow.
    1. Ask a lead for the McGill Robotics account information.
4. Upload your data and label it using bounding boxes.
    1. You can keep the train/valid/test default split suggested by Roboflow.
    2. Make sure to properly label the objects. It doesn't have to be perfect, but it does affect the training performance.
5. Generate a new version of the dataset. 
    1. Do NOT add any pre-processing/augmentation. We already do that in the training notebook.
6. Change the parameters in the training notebook as needed.
    1. The order of classes in the script has to be _**alphabetical**_ (same as Roboflow).
    2. As of now [28/06/2024], ultralytics only stably supports YOLOv8. YOLOv9 is available but not fully reliable. 
    3. The script automatically chooses the lastest dataset version. If you want to train an older version, follow these steps:
        1. Open the script,
        2. Go to "Download dataset from Roboflow" section,
        3. Change the value of `latest_version` to the version you want,
        4. If you make other changes in the script and would like to commit them, make sure you undo the `lastest_version` change. Lastest dataset version should be the default.
8. Run training notebook.
    1. Note: I _**HIGHLY**_ recommend training a model only if you have a **_GPU_** available.
        1. If you don't have one, it will take days to reach an acceptable performance. So, ask someone who does to let the model train overnight for you.
    3. [Optional] If you don't like Jupyter's output (as I don't), you can execute all cells EXCEPT the final one (which trains the model). Instead, use the `training.py` file to train the model, which displays the training progress in the terminal.
        1. Make sure to update the parameters in the parameter boxes.
        2. There is no need to change the rest of the code.
