#array containing classes detected
int16[] label
#pose info
float64[] x
float64[] y
float64[] z
float64[] theta_z
#array containing confidence in object position
float64[] label_confidence
float64[] pose_confidence
int16[] observation_count
#additional data field (for additional info on objects like lane marker second heading)
float64[] extra_field