import numpy as np
import rospy

class KalmanXY:

    def __init__(self,vx,vy, cov=np.eye(2),Q=np.eye(2)*0.1,R=np.eye(2)*0.05,initial_state = np.array([0,0])):
        self.prev_v = np.array([vx,vy])
        self.prev_t = rospy.get_time()
        self.covariance = cov
        self.Q = Q
        self.R = R
        self.prev_state = initial_state


    def estimate_x_y(self,dvl,accel):
        t = rospy.get_time()
        detla_t = t - self.prev_t
        self.prev_t = t
        A = np.eye(2)
        B = np.eye(2) * detla_t

        v_pred = np.matmul(A,self.prev_v) + np.matmul(B,accel)
        residual = dvl - v_pred
        cov = np.matmul(A,np.matmul(self.covariance,A)) + self.Q
        S = cov + self.R
        S_inv = np.linalg.inv(S)
        K = np.matmul(cov,S_inv)
        final = v_pred + np.matmul(K,residual)
        self.covariance = cov - np.matmul(cov,np.matmul(S_inv,cov))

        self.prev_v = final

        position = self.prev_state + detla_t*final
        self.prev_state = position
        return position


    

