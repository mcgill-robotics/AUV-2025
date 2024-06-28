# Steps to train a model
1. Collect images samples (either sim or real).
2. Either open or add the images to AUV2024 workspace on Roboflow.
    1. Ask a lead for the McGill Robotics account information.
3. Upload your data and label it using bounding boxes.
    1. You can keep the train/valid/test defualt split suggested by Roboflow.
4. Generate a new version of the dataset. 
    1. Do NOT add any pre-processing/augmentation. We already do that in the training notebook.
5. Change the parameters in the training notebook as needed.
6. Run training notebook.
    1. [Optional] If you don't like Jupyter's output (as I don't), you can execute all cells EXCEPT the final one (which trains the model). Instead, use the `training.py` file to train the model, which displays the training progress in the terminal.
        1. Make sure to update the parameters in the paramater boxes.
        2. There is no need to change the rest of the code.
