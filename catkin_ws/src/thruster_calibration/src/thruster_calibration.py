#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterForces
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
import csv, time
import os
import pathlib 


class States:
    
   
    # O if we into account this variable otherwise None
    variables = {
        'x' : 0,
        'y' : None,
        'z' : None,
        'theta_x' : None,
        'theta_y' : None,
        'theta_z' : None,

    }
    




    def reset(self):
        for i, j in self.variables.items():
            if j != None:
               self.variables[i] = False

    def __str__(self):
        return str(self.x)

    def set_x(self, x):
        x = x.data
        self.variables['x'] = x

    def set_y(self, y):
        y = y.data
        self.variables['y'] = y

    def set_z(self, z):
        z = z.data
        self.variables['z'] = z

    def set_theta_x(self, x):
        x = x.data
        self.variables['theta_x'] = x

    def set_theta_y(self, y):
        y = y.data
        self.variables['theta_y'] = y

    def set_theta_z(self, z):
        z = z.data
        self.variables['theta_z'] = z

    def isEquilibrium(self):
        global delta
        final_state = 0
        for i,j in self.variables.items():
            if type(j)==bool:
                return False
            elif j != None:
                final_state += j

        self.reset()
        return abs(final_state) <= delta

    
    
def writeForces(data):
    global thrusterForcesArray
    f = data.force
    t = data.torque

    thrusterForcesArray = [f.x, f.y, f.z, t.x, t.y, t.z]
   
    

if __name__ == '__main__':

    number_of_repetion = 10
    delta = 5 #stability value
    
    counter = 0
    thrusterForcesArray = []

    thrusterForcesFinalArray = []




    rospy.init_node('listener', anonymous=True)

    states = States()
    rospy.Subscriber('/state_x', Float64, states.set_x)
    rospy.Subscriber('/state_y', Float64, states.set_y)
    rospy.Subscriber('/state_z', Float64, states.set_z)
    rospy.Subscriber('/state_theta_x', Float64, states.set_theta_x)
    rospy.Subscriber('/state_theta_y', Float64, states.set_theta_y)
    rospy.Subscriber('/state_theta_z', Float64, states.set_theta_z)
    rospy.Subscriber('/effort', Wrench, writeForces)

    
    for _ in range(number_of_repetion):
        while counter < 5:
            if states.isEquilibrium():
                counter+=1
                thrusterForcesFinalArray.append(thrusterForcesArray)
            else:
                counter = 0

            time.sleep(0.1)
        
        counter = 0
        
        

    with open('data_calibration.csv', mode = 'a') as data_file:
        data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        #Header of the CSV file : force: {x: 20.0, y: 0.0, z: 0.0}, torque:{x: 0.0, y: 0.0, z: 0.0}
        for i in thrusterForcesFinalArray:    
            data_writer.writerow(i)

        
        
  
        
    

    rospy.spin()
    

