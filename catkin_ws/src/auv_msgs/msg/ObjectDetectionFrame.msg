#array containing classes detected
int16[] label
#arrays containing bounding box dimensions
float64[] bounding_box_x
float64[] bounding_box_y
float64[] bounding_box_width
float64[] bounding_box_height
#array containing confidence in detections (probabilities)
float64[] confidence
#which camera this was detected on
uint8 camera
#message info for lane markers (None for other objects)
float64[] heading1
float64[] heading2
float64[] center_x
float64[] center_y