import rospy
import smach
from .utility.functions import *
import numpy as np
import quaternion

class NavigateBuoy(smach.State):
    def __init__(self, control, state, mapping):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.color = rospy.get_param("target_color") 
        self.radius = rospy.get_param("buoy_circumnavigation_radius")
        self.offset_distance = rospy.get_param("buoy_centering_offset_distance")

    def execute(self, ud):
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, rospy.get_param("nominal_depth")))
        self.control.flatten()

        print("Starting buoy navigation.")
        
        buoy_object = self.mapping.getClosestObject(cls="Buoy", pos=(self.state.x, self.state.y))
        if buoy_object is None:
            print("No buoy in object map! Failed.")
            return 'failure'
        
        print("Centering and rotating in front of buoy.")

        self.control.move((None, None, buoy_object[3]))

        position_auv = [self.state.x, self.state.y, self.state.z]
        position_buoy = [buoy_object[1], buoy_object[2],buoy_object[3]]

        direction = np.array(position_buoy) - np.array(position_auv)
        current_distance = np.linalg.norm(direction)

        forward_vector = np.array([1, 0, 0])
        auv_quat = self.state.quat
        auv_forward_vector = quaternion.rotate_vectors(auv_quat, forward_vector)

        rotation_quaternion = quaternion_between_vectors(np.array([auv_forward_vector[0], auv_forward_vector[1],auv_forward_vector[2]]), np.array([direction[0],direction[1],direction[2]]))

        self.control.rotateDelta([rotation_quaternion.w, rotation_quaternion.x, rotation_quaternion.y, rotation_quaternion.z])

        vector_auv_buoy = direction / current_distance * self.offset_distance
        new_position = np.array([position_buoy[0],position_buoy[1],position_buoy[2]]) - np.array([vector_auv_buoy[0],vector_auv_buoy[1],vector_auv_buoy[2]])
        print("Moving to new position: ", new_position)
        self.control.move((new_position[0], new_position[1], new_position[2]))


        # wait and keep measuring just to be safe
        print("Waiting to improve measurement accuracy")
        rospy.sleep(rospy.get_param("object_observation_time"))

        self.control.move((None, None, buoy_object[3]))

        position_auv = [self.state.x, self.state.y, self.state.z]
        position_buoy = [buoy_object[1], buoy_object[2],buoy_object[3]]

        direction = np.array(position_buoy) - np.array(position_auv)
        current_distance = np.linalg.norm(direction)

        forward_vector = np.array([1, 0, 0])
        auv_quat = self.state.quat
        auv_forward_vector = quaternion.rotate_vectors(auv_quat, forward_vector)

        rotation_quaternion = quaternion_between_vectors(np.array([auv_forward_vector[0], auv_forward_vector[1],auv_forward_vector[2]]), np.array([direction[0],direction[1],direction[2]]))

        self.control.rotateDelta([rotation_quaternion.w, rotation_quaternion.x, rotation_quaternion.y, rotation_quaternion.z])

        vector_auv_buoy = direction / current_distance * self.offset_distance
        new_position = np.array([position_buoy[0],position_buoy[1],position_buoy[2]]) - np.array([vector_auv_buoy[0],vector_auv_buoy[1],vector_auv_buoy[2]])
        print("Moving to new position: ", new_position)
        self.control.move((new_position[0], new_position[1], new_position[2]))
        
        # Start buoy navigation
        rad = self.radius

        if self.color == "red":
            self.control.moveDeltaLocal((0,-rad,0))
            self.control.moveDeltaLocal((self.offset_distance + rad,0,0))
            self.control.rotateDeltaEuler((0,0,90))
            self.control.moveDeltaLocal((2*rad,0,0))
            self.control.rotateDeltaEuler((0,0,90))
            self.control.moveDeltaLocal((self.offset_distance + rad,0,0))
        else:
            self.control.moveDeltaLocal((0,rad,0))
            self.control.moveDeltaLocal((self.offset_distance + rad,0,0))
            self.control.rotateDeltaEuler((0,0,-90))
            self.control.moveDeltaLocal((2*rad,0,0))
            self.control.rotateDeltaEuler((0,0,-90))
            self.control.moveDeltaLocal((self.offset_distance + rad,0,0))

        print("Successfully completed buoy task!")
        
        return 'success'
        
