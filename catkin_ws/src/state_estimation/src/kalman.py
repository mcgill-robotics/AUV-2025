import numpy as np
import rospy


class KalmanXY:

    def __init__(self,vx,vy, cov=np.eye(2),Q=np.eye(2)*0.1,initial_state = np.array([0,0])):
        self.prev_v = np.array([vx,vy])
        self.covariance = cov
        self.Q = Q
        self.prev_state = initial_state

        self.times = [rospy.get_time()]
        self.velocities = [(0,0)]


    def accel_reading(self,accel):
        t = rospy.get_time()
        delta_t = t - self.times[-1]
        self.times.append(t)
        A = np.eye(2)
        B = np.eye(2) * delta_t

        v_pred = np.matmul(A,self.prev_v) + np.matmul(B,accel)
        self.covariance = np.matmul(A,np.matmul(self.covariance,np.transpose(A))) + self.Q
        self.prev_v = v_pred
        self.velocities.append(v_pred)

    def dvl_reading(self,dvl):
        t = rospy.get_time()
        delta_t = t - self.times[-1]
        self.times.append(t)
        residual = dvl - self.prev_predicted_velocity
        S = self.covariance + self.R
        S_inv = np.linalg.inv(S)
        K = np.matmul(self.covariance,S_inv)
        final = self.prev_predicted_velocity + np.matmul(K,residual)
        self.covariance -=  np.matmul(K,self.covariance)
        self.prev_v = final

    

