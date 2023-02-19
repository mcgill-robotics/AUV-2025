#FOR downwards camera:

#when object is detected, freeze AUV position and orientation
#attempt, if necessary, to center object in viewframe (using x/y translation)
#attempt, if necessary, to move closer/further from object (by changing AUV depth)
#if object persists in detection then add it (approximate location) to object map
#if same class object already exists nearby in object map than improve accuracy of location of that object (if possible)


#FOR stereo cameras
#when object is detected, freeze AUV position and orientation
#get approximate depth of detected object
#record approximate depth measurement and AUV position/orientation for every frame
#if far away, rotate AUV to center object in viewframe, otherwise use x/y translation 
#attempt, if necessary, to move closer/further from object (by mvoing AUV forward)
#use recordings of depth/AUV position to assume approximate location of object
#add to object map
