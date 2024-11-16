#!/usr/bin/env python3 

import rospy,math,numpy as np
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class EKF_Localization :

    def __init__(self):
        self.counter = 0
        # self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.reference_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.reference_coordinates)#state_subscriber
        self.pose_sub = rospy.Subscriber("/odom", Odometry , self.odom_model)
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan,self.laser_callback)#lidar_subscriber
        self.approx_angular_position = [] # stores the estimated cylinder angular position
        self.approx_linear_distance = [] # sotres the estimated cylinder linear position
        self.cylinder_radii = .16 # this is to get the cylinder radii 
        self.min_dist = 0.05 # this is to make cylinder pairs
        self.sigma_r = 0.06 # This is to define standard deviation in the distance measurement 
        self.sigma_alpha = 0.01  # This is to define the standard deviation in the angle measurement
        self.mu_bar = np.zeros((3, 1))
        self.covariance = np.eye(3) * 1e-3 #initialization of the covariance matrix
        self.approx_angular_position = [] #initialization of the angular position of cylinder
        self.approx_linear_distance = [] # '''''''''''''' '' ''' linear  '''''''' '' ''''''''
        self.result = [] #initialization of the result
        self.cylinder_final_estim_cordinates = [] #initializaion of thecoordinates function
        # self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)
        self.x_prev = 0
        self.y_prev = 0
        self.theta_prev = 0
        self.final_covariance = np.eye(3)
        self.pose_pub = rospy.Publisher("/ekf_pose_covariance", PoseWithCovarianceStamped, queue_size=10)
        # Wait for non-zero lidar data to initialize lidar_points
        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message("/scan", LaserScan, timeout=1.0)
                if any(r > 0 for r in msg.ranges):  # Check for non-zero values
                    # Initialize lidar_points with valid ranges data
                    self.lidar_points = [
                        r if not (math.isnan(r) or math.isinf(r)) else msg.range_max
                        for r in msg.ranges
                    ]
                    rospy.loginfo("Lidar data received and initialized.")
                    break  # Exit loop once lidar_points is initialized
                else:
                    rospy.logwarn("Waiting for non-zero lidar data...")
            except rospy.ROSException:
                rospy.logwarn("Timeout waiting for /scan. Retrying...")
    

    def reference_coordinates(self, msg):
        a2 = len(msg.pose) - 3
        self.alpha = []  # Initialize as an empty list

        for i in range(a2):
            # Append each coordinate as a list
            self.alpha.append([msg.pose[3+i].position.x, msg.pose[3+i].position.y])
              
        # #(self.alpha)
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
        #Odometry_Motion_Model is definned here

    def pose_predictor(self):
        self.x_predicted = self.x_prev + self.delta_trans*math.cos(self.theta + self.delta_rot1)
        self.y_predicted = self.y_prev + self.delta_trans*math.sin(self.theta + self.delta_rot1)
        self.theta_predicted = normalize_angle(self.theta_prev + self.delta_rot1 + self.delta_rot2)

        
        self.mu_bar = np.array([
            [self.x_predicted],
            [self.y_predicted],
            [self.theta_predicted]
        ])
        #Above is the predcited pose of the robot w.r.t. to the robot's odometry motion model

    def state_covariance_calc(self):

        self.G_t = np.array([
            [1 , 0  , -self.delta_trans*math.sin(self.theta_prev+self.delta_rot1)],
            [0 , 1 , self.delta_trans*math.cos(self.theta_prev+self.delta_rot1)],
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

        self.covariance = np.dot(self.G_t, np.dot(self.final_covariance, self.G_t.T)) + np.dot(self.V, np.dot(control_covariance, self.V.T))
        #Prediction Covariance is calculated here.

    def laser_callback(self,msg):
        self.laser_points = [0]*len(msg.ranges)
        for i in range(0,len(msg.ranges)):
            if(math.isnan(msg.ranges[i]) or math.isinf(msg.ranges[i])):
                self.laser_points[i] = msg.range_max
            else: 
                self.laser_points[i] = msg.ranges[i]
        #lidar_data being filtered and saved into an object

    def observed_cylinders(self):
        jumps=[0]*len(self.laser_points)
        jumps_index=[]
        for i in range(1,len(self.laser_points)-1):
            next_point = self.laser_points[i+1]
            prev_point = self.laser_points[i-1]
            if(prev_point>.2 and next_point>.2):
                derivative = (next_point-prev_point)/2
            if(abs(derivative)>.3):
                jumps[i] = derivative
                jumps_index.append(i)
        
        cylin_detected,no_of_rays,sum_ray_indices,sum_depth,count,i=0,0,0,0,0,-1        
        self.approx_linear_distance,self.approx_angular_position =[],[]
        while(i<len(jumps)-1):
            
            i+=1
            if(jumps[i]<0 and cylin_detected==0): #a falling edge detected in the lidar's scan and currently not on a cylinder
                cylin_detected = 1 #the first cylinder has been detected
                no_of_rays += 1 #increment the number of rays that are falling on the cylinder
                sum_ray_indices += i #sum up the indices of the rays
                sum_depth += self.laser_points[i]  #sum of the distance travelled by the rays falling on cylinder 

            elif(jumps[i]<0 and cylin_detected==1): # a second falling edge has been detected so ignore the previous data if already on the cylinder
                cylin_detected = 0  # now reset the value of cylinder_detected
                no_of_rays = 0 # reset the no. of rays falling on the cylinders
                sum_ray_indices = 0 # reset
                sum_depth = 0 # reset
                i-=1 # decrementing the index so that this falling edge can be checked again and can be passed to the 1st if statement
            
            elif(jumps[i]>0 and cylin_detected == 1):#the rising edge means the cylinder boundary is completed if right now on a cylinder
                cylin_detected = 0  # now reset the value of cylinder_detected
                self.approx_angular_position.append(sum_ray_indices/no_of_rays)
                self.approx_linear_distance.append(sum_depth/no_of_rays) 
                no_of_rays = 0 # reset the no. of rays falling on the cylinders
                sum_ray_indices = 0 # reset
                sum_depth = 0 # reset
                
            elif(jumps==0 and cylin_detected==1):
                no_of_rays+=1
                sum_depth+=self.laser_points[i]
                sum_ray_indices+=i

            else:
                pass #do_nothing
        # print(self.approx_linear_distance)
        # print(self.approx_angular_position)
        #(jumps)

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
            x_global = self.x_predicted + self.result[i][0] * math.cos(normalize_angle(self.theta)) - self.result[i][1] * math.sin(normalize_angle(self.theta))
            y_global = self.y_predicted + self.result[i][0] * math.sin(normalize_angle(self.theta)) + self.result[i][1] * math.cos(normalize_angle(self.theta))
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
                dist = math.sqrt((x_global_estim-x_ref_cylin)**2 + (y_global_estim-y_ref_cylin)**2)
                if(dist < self.min_dist and flag[j]==0):# Checking if the same cylinder is not being matched multiple times
                    self.match_pairs_left.append([x_global_estim,y_global_estim])
                    self.match_pairs_right.append([x_ref_cylin,y_ref_cylin])
                    flag[j]=1

                else:
                    pass # Do Nothing
        # print(self.match_pairs_left)

    def obs_model(self):
        # Implementing an observation model to obtain the requirred variables for the correction step for the EKF
        '''While implementing this code, I had a doubt that why we are repeating the step of recalculating the angle and distance info when we can do it in the previous state.'''
        #(len(self.match_pairs_left))
        for i in range(0,len(self.match_pairs_left)):
            
            x_estim = self.match_pairs_left[i][0]
            y_estim = self.match_pairs_left[i][1]
            x_ref = self.match_pairs_right[i][0]
            y_ref = self.match_pairs_right[i][1]

            x_delta = x_ref - self.x_predicted
            y_delta = y_ref - self.y_predicted

            x_estim_delta = x_estim - self.x_predicted
            y_estim_delta = y_estim - self.y_predicted

            dist = math.sqrt((x_delta)**2 + (y_delta)**2) # distance of the reference cylinder from the robot's current position
            alpha = math.atan2(y_delta,x_delta) - self.theta_predicted # angle subtended by reference cylinder w.r.t. robot's heading

            estim_dist = math.sqrt((x_estim_delta)**2 + (y_estim_delta)**2) # distance of the measured cylinder from the robot's current position
            estim_alpha = math.atan2(y_estim_delta,x_estim_delta) - self.theta_predicted # angle subtended by measured cylinder w.r.t. robot's heading
            
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
                [self.sigma_r**2 , 0 ],
                [0 , self.sigma_alpha**2]
            ])

            self.covariance = np.array(self.covariance)

            # print(f"a1,b1 = {np.shape(self.covariance)}, a2,b2 = {np.shape(H_t)}, a3,b3 = {np.shape(q_matrix)}")

            k_gain = np.dot(self.covariance,np.dot(H_t.T,np.linalg.inv(np.dot(H_t,np.dot(self.covariance,H_t.T)))+q_matrix)) # to claculate the kalman gain for each observed cylinder

            Innovation_matrix = np.array([
                [z_matrix[0, 0] - h_matrix[0, 0]],  # distance difference
                [normalize_angle(z_matrix[1, 0] - h_matrix[1, 0])]  # angle difference
            ])
            # Innovation matrix to get the difference between the observed cylinder and reference cylinder

            self.mu =  self.mu_bar + np.dot(k_gain , Innovation_matrix) # final pose calculation

            Identity = np.eye(3) # Identity matrix for the final covariance calculation

            self.final_covariance = np.dot((Identity - np.dot(k_gain,H_t)),self.covariance) # final covariance calculation
        if (len(self.match_pairs_left)==0):
            self.mu = self.mu_bar
            self.final_covariance = self.covariance
            #(len(self.match_pairs_left))
        #print(self.final_covariance)

        self.x_prev = self.x
        self.y_prev = self.y
        self.theta_prev = self.theta

    def publish_pose_with_covariance(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"  # Use the appropriate frame of reference
        
        # Setting the pose based on the mean (mu)
        pose_msg.pose.pose.position.x = self.mu[0, 0]
        pose_msg.pose.pose.position.y = self.mu[1, 0]
        pose_msg.pose.pose.position.z = 0  # Assume planar navigation

        # Convert orientation from Euler to quaternion
        quat = quaternion_from_euler(0, 0, self.mu[2, 0])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Fill in the covariance (flattened row-major order)
        covariance_flat = self.final_covariance.flatten()
        pose_msg.pose.covariance = [covariance_flat[i] if i < len(covariance_flat) else 0 for i in range(36)]

        # Publish the message
        self.pose_pub.publish(pose_msg)


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz or any other desired rate
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
            self.publish_pose_with_covariance()
            rate.sleep()




if __name__ == '__main__':
    rospy.init_node('ekf_localization_node')
    ekf = EKF_Localization()
    ekf.run()
    rospy.spin()
