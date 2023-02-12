All data must be put into the format expected by the YOLO v7 model.

That format is:
    - An "images" folder containing all training images
    - In the same folder as the images folder, a "labels" folder containing one .txt file for each image in /images
    - Each .txt file should have the same name as the image it annotates.

The format for annotation in the .txt files is:
    - each line describes one bounding box: "class, x_center, y_center, width, height"
    - The fields x, y, width, and height should be normalized to the dimensions of the image i.e. between 0 and 1


Dependencies to use model in project:
    pip install opencv-python
    pip install ultralytics

To train model on Google Colab:
- Open .ipynb file in Google Colab
- Ensure Colab is using GPU (set this in Edit > Notebook Settings > Hardware Acceleration)
- Run all cells for installing and importing librairies, as well as those to set up directory structure and generate data.yaml file
- Customize (if necessary) training label count and names in data.yaml file ('nc' and 'names' fields)
- Upload training images and labels (in YOLO format) in content/data/raw/images/ and content/data/raw/labels/ respectively
- Run the rest of the notebook

