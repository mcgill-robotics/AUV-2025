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
#where the AUV was when it detected this
float64[] obj_x
float64[] obj_y
float64[] obj_z
float64[] obj_theta_z
#message info for lane markers (None for other objects)
float64[] heading1
float64[] heading2
