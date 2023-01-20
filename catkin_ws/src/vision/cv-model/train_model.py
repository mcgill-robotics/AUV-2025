from ultralytics import YOLO
import os


if __name__ == '__main__':
    # Load a model
    model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

    #hacky solution since relative paths in the .yaml file do not behave as expected:
    #generates .yaml with absolute path to dataset
    pwd = os.path.realpath(os.path.dirname(__file__))
    with open(pwd + '/clean-data/data.yaml', 'w+') as f:
        f.write("train: " + pwd + "/clean-data/train/images\n")
        f.write("test: " + pwd + "/clean-data/test/images\n")
        f.write("val: " + pwd + "/clean-data/val/images\n")
        f.write("nc: 1\n")
        f.write('names: ["lane_marker"]')


    # Use the model
    accuracy = 0
    #while accuracy < 0.8:
    results = model.train(data=pwd+"/clean-data/data.yaml", epochs=3, batch=16, degrees=360, flipud=0.5, fliplr=0.5, pretrained=True)  # train the model
    results = model.val()  # evaluate model performance on the validation set
    print("Model evaluation: " + str(results))
    accuracy = 1.0
    success = model.export(format="onnx")  # export the model to ONNX format



    #IN ROS

    #model = YOLO("trained-model.pt")  # load a pretrained model (recommended for training)
    #results = model("image")  # predict on an image
