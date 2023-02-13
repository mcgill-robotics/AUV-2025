For training YOLO models on custom data, see /cv-model folder.

Launching object_detection.launch file will:
- start up usb_cam node to capture video from webcams and publish the images to specific topics (if not yet started)
- run the models specified in object_detection.py on images received on the topics associated to the AUV cameras (one model per camera feed)
- output the models' predictions as ObjectDetectionViewframe messages (defined in auv_msgs) to the /vision/viewframe_detection topic
- create one visualization per camera feed showing the model's predictions (boundings boxes, labels, headings, ...) and publish them as images to /vision/[camera_name]_visual

Launching record_cameras.launch file will:
- start up usb_cam node to capture video from webcams and publish the images to specific topics (if not yet started)
- record the image feed from the AUV cameras
- save the images as a video file to the file location specified in record_cameras.py

Launching vision.launch file will:
- launch both record_cameras and object_detection nodes