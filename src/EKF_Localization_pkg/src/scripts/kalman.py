#!/usr/bin/env python3 

import rospy,math,numpy as np
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class EKF_Localization :
    x,y,theta,x_prev,y_prev,theta_prev=0,0,0,0,0,0
    yaw=0
    delta_trans,delta_rot1,delta_rot2=0,0,0
    x_predicted,y_predicted,theta_predicted=0,0,0

    def __init__(self):
        self.counter = 0
        # self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.reference_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.reference_coordinates)#state_subscriber
        self.pose_sub = rospy.Subscriber("/odom", Odometry , self.odom_model)
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan,self.laser_callback)#lidar_subscriber
        self.approx_angular_position = [] # stores the estimated cylinder angular position
        self.approx_linear_distance = [] # sotres the estimated cylinder linear position
        self.cylinder_radii = 0 # this is to get the cylinder radii 
        self.min_dist = 0 # this is to make cylinder pairs
        self.sigma_l = 0 # This is to define standard deviation in the distance measurement 
        self.sigma_r = 0  # This is to define the standard deviation in the angle measurement
        self.mu_bar = np.zeros((3, 1))
        self.covariance = np.eye(3) * 1e-3 #initialization of the covariance matrix
        self.approx_angular_position = [] #initialization of the angular position of cylinder
        self.approx_linear_distance = [] # '''''''''''''' '' ''' linear  '''''''' '' ''''''''
        self.result = [] #initialization of the result
        self.cylinder_final_estim_cordinates = [] #initializaion of thecoordinates function
        # self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)

    

    def reference_coordinates(self, msg):

        self.alpha=list(range(10))
        a2=len(msg.pose)
        for i in range(0,a2):
            self.alpha[i] = [msg.pose[4+i].position.x,msg.pose[4+i].position.y]                
        # print(self.alpha)
        # above, the reference co-ordinates of the objects is being obtainned 
    
    def callback_reset_counter(self, req):
        if req.data:
            self.counter = 0
            return True, "Counter has been successfully reset"
        return False, "Counter has not been reset"
    
    def ticks_to_pose(self):
        pass
        #Right now not in use as we are focusing on the simulation and it has a direct availability of the pose.
    
    def odom_model(self,msg1):
        self.x=msg1.pose.pose.position.x
        self.y=msg1.pose.pose.position.y
        
        orientation_q = msg1.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        self.theta = normalize_angle(self.yaw)
        self.delta_trans = math.sqrt((self.x-self.x_prev)**2+(self.y-self.y_prev)**2)
        self.delta_rot1 = normalize_angle(math.atan2(self.y - self.y_prev, self.x - self.x_prev) - self.theta_prev)
        self.delta_rot2 = normalize_angle(self.theta - self.theta_prev - self.delta_rot1)

        self.x_prev = self.x
        self.y_prev = self.y
        self.theta_prev = self.theta
        #Odometry_Motion_Model is definned here

    def pose_predictor(self):
        self.x_predicted = self.x + self.delta_trans*math.cos(self.theta + self.delta_rot1)
        self.y_predicted = self.y + self.delta_trans*math.sin(self.theta + self.delta_rot2)
        self.theta_predicted = normalize_angle(self.theta + self.delta_rot1 + self.delta_rot2)

        
        self.mu_bar = np.array([
            [self.x_predicted],
            [self.y_predicted],
            [self.theta_predicted]
        ])
        #Above is the predcited pose of the robot w.r.t. to the robot's odometry motion model

    def state_covariance_calc(self):

        self.G_t = np.array([
            [1 , 0  , -self.delta_trans*math.sin(self.theta+self.delta_rot1)],
            [0 , 1 , self.delta_trans*math.cos(self.theta+self.delta_rot1)],
            [0 , 0, 1]
        ]) #W.R.T. STATES(POSITION,ORIENTATION)         

    def control_covariance_calc(self):

        self.V = np.array([
            [-self.delta_trans*(math.sin(self.theta+self.delta_rot1)) , math.cos(self.theta + self.delta_rot1) , 0],
            [self.delta_trans*math.cos(self.theta + self.delta_rot1) , math.sin(self.theta + self.delta_rot1) , 0],
            [1 , 0 , 1]      
        ]) #W.R.T. CONTROLS U=[DEL_R1,DEL_T,DEL_R2]

    def prediction_covariance_calc(self):

        alpha1 = 0.05
        alpha2 = 0.01
        alpha3 = 0.05
        alpha4 = 0.01
        self.rot1_variance = alpha1 * abs(self.delta_rot1) + alpha2 * abs(self.delta_trans)
        self.trans_variance = alpha3 * abs(self.delta_trans) + alpha4 * (abs(self.delta_rot1) + abs(self.delta_rot2))
        self.rot2_variance = alpha1 * abs(self.delta_rot2) + alpha2 * abs(self.delta_trans)
        control_covariance = np.diag([self.rot1_variance, self.trans_variance, self.rot2_variance])

        self.covariance = np.dot(self.G_t, np.dot(self.covariance, self.G_t.T)) + np.dot(self.V, np.dot(control_covariance, self.V.T))
        #Prediction Covariance is calculated here.

    def laser_callback(self,msg):
        self.laser_points = msg.ranges
        for i in range (0, len(self.laser_points)):
            if math.isnan(self.laser_points[i]) or math.isinf(self.laser_points[i]):
                self.laser_points[i]=msg.range_max
        #lidar_data being filtered and saved into an object

    def observed_cylinders(self):
        jumps=[]
        jumps_index=[]
        for i in range(1,len(self.laser_points)-1):
            next_point = self.laser_points[i+1]
            prev_point = self.laser_points[i-1]
            if(prev_point>.2 and next_point>.2):
                derivative = (next_point-prev_point)/2
            if(abs(derivative)>.3):
                jumps.append(derivative)
                jumps_index.append(i)
        
        cylin_detected,no_of_rays,sum_ray_indices,sum_depth=0,0,0,0        
        
        for i in range(0,len(jumps)):

            if(jumps[i]<0 and cylin_detected==0): #a falling edge detected in the lidar's scan and currently not on a cylinder
                cylin_detected = 1 #the first cylinder has been detected
                no_of_rays += 1 #increment the number of rays that are falling on the cylinder
                sum_ray_indices += jumps_index[i] #sum up the indices of the rays
                sum_depth += self.laser_points[jumps_index[i]]  #sum of the distance travelled by the rays falling on cylinder 

            elif(jumps[i]<0 and cylin_detected==1): # a second falling edge has beenn detected so ignore the previous data if already on the cylinder
                cylin_detected = 0  # now reset the value of cylinder_detected
                no_of_rays = 0 # reset the no. of rays falling on the cylinders
                sum_ray_indices = 0 # reset
                sum_depth = 0 # reset
                i-=1 # decrementing the index so that this falling edge can be checked again and can be passed to the 1st if statement
            
            elif(jumps[i]>0 and cylin_detected == 1):#the rising edge means the cylinder boundary is completed if right now on a cylinder
                cylin_detected = 0
                self.approx_angular_position.append(sum_ray_indices/no_of_rays)
                self.approx_linear_distance.append(sum_depth/no_of_rays) 
            
            else:
                pass #do_nothing

    def cylinder_cartesian(self):
        self.result=[]
        # Converting the cylinder's co-ordinates into Li-DAR's co-ordinate frame
        for i in range(0,len(self.approx_linear_distance)):
            result1 = (self.approx_linear_distance[i]+self.cylinder_radii)
            x = result1 * math.cos(normalize_angle(self.approx_angular_position[i]))
            y = result1 * math.sin(normalize_angle(self.approx_angular_position[i]))
            self.result.append([x,y])

    def cylinder_cartesian_to_world(self):
        # Cylinder's co-ordinates transform from lidar to world co-ordinate system
        self.cylinder_final_estim_cordinates=[]
        for i in range(0,len(self.result)):
            x_global = self.x + self.result[i][0] * math.cos(normalize_angle(self.theta)) - self.result[i][1] * math.sin(normalize_angle(self.theta))
            y_global = self.y + self.result[i][0] * math.sin(normalize_angle(self.theta)) + self.result[i][1] * math.cos(normalize_angle(self.theta))
            self.cylinder_final_estim_cordinates.append([x_global,y_global])

    def finding_closest_neighbours(self):
        # now comparing the distances between observed and reference cylinders and making pairs accordingly
        self.match_pairs_left=[]
        self.match_pairs_right=[]
        flag=[0]*len(self.alpha)
        for i in range(0,len(self.cylinder_final_estim_cordinates)): # making cylinder pairs.
            x_global_estim = self.cylinder_final_estim_cordinates[i][0]
            y_global_estim = self.cylinder_final_estim_cordinates[i][1]
            for j in range(0,len(self.alpha)):
                x_ref_cylin = self.alpha[j][0]
                y_ref_cylin = self.alpha[j][1]
                dist = math.sqrt((x_global_estim-x_ref_cylin)**2 + (y_global_estim-y_ref_cylin))
                if(dist < self.min_dist and flag[j]==0):# Checking if the same cylinder is not being matched multiple times
                    self.match_pairs_left.append([x_global_estim,y_global_estim])
                    self.match_pairs_right.append([x_ref_cylin,y_ref_cylin])
                    flag[j]=1

                else:
                    pass # Do Nothing

    def obs_model(self):
        # Implementing an observation model to obtain the requirred variables for the correction step for the EKF
        '''While implementing this code, I had a doubt that why we are repeating the step of recalculating the angle and distance info when we can do it in the previous state.'''
        
        for i in range(0,len(self.match_pairs_left)):
            
            x_estim = self.match_pairs_left[i][0]
            y_estim = self.match_pairs_left[i][1]
            x_ref = self.match_pairs_right[i][0]
            y_ref = self.match_pairs_right[i][1]

            x_delta = x_ref - self.x
            y_delta = y_ref - self.y

            x_estim_delta = x_estim - self.x
            y_estim_delta = y_estim - self.y

            dist = math.sqrt((x_delta)**2 + (y_delta)**2) # distance of the reference cylinder from the robot's current position
            alpha = math.atan2(y_delta,x_delta) - self.theta # angle subtended by reference cylinder w.r.t. robot's heading

            estim_dist = math.sqrt((x_estim_delta)**2 + (y_estim_delta)**2) # distance of the measured cylinder from the robot's current position
            estim_alpha = math.atan2(y_estim_delta,x_estim_delta) - self.theta # angle subtended by measured cylinder w.r.t. robot's heading
            
            #gives the z_matrix for the final mean and covariance calculation
            z_matrix = np.array([
                [dist],
                [alpha]
            ])

            #h for calculating the innovation
            h_matrix = np.array([
                [estim_dist],
                [estim_alpha]
            ])

            #Jacobian of the h_matrix to obtain the Kalman Gain
            H_t = np.array([
                [(-x_delta/dist) , (-y_delta/dist) , 0],
                [(y_delta/(dist**2)) , (-x_delta/(dist**2)) , -1]
            ])
            
            #Q matrix signifying the measurement covariances
            q_matrix = np.array([
                [self.sigma_l**2 , 0 ],
                [0 , self.sigma_l**2]
            ])

            self.covariance = np.array(self.covariance)

            k_gain = np.array(np.dot(np.dot(self.covariance,H_t),np.linalg.inv((np.dot(H_t,(self.covariance),H_t.T) + q_matrix)))) # to claculate the kalman gain for each observed cylinder

            Innovation_matrix = np.array([
                [z_matrix[0, 0] - h_matrix[0, 0]],  # distance difference
                [normalize_angle(z_matrix[1, 0] - h_matrix[1, 0])]  # angle difference
            ])
            # Innovation matrix to get the difference between the observed cylinder and reference cylinder

            self.mu =  self.mu_bar + np.dot(k_gain , Innovation_matrix) # final mean calculation

            Identity = np.eye(3) # Identity matrix for the final covariance calculation

            self.final_covariance = np.dot((Identity - np.dot(k_gain,H_t)),self.covariance) # final covariance calculation

    def run(self):
        while not rospy.is_shutdown():
            self.pose_predictor()
            self.state_covariance_calc()
            self.control_covariance_calc()
            self.prediction_covariance_calc()
            self.observed_cylinders()
            self.cylinder_cartesian()
            self.cylinder_cartesian_to_world()
            self.finding_closest_neighbours()
            self.obs_model()



if __name__ == '__main__':
    rospy.init_node('ekf_localization_node')
    ekf = EKF_Localization()
    ekf.run()
    rospy.spin()