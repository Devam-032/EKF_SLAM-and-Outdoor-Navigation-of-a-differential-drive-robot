#!/usr/bin/env python3

import rospy,math,numpy as np
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class EKF_SLAM:
    def __init__(self):
        rospy.init_node('ekf_slam_node')
        
        # Initialize Subscribers
        self.pose_sub = rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        
        # Initialize Pose Variables
        self.prev_sub_x, self.prev_sub_y, self.prev_sub_theta = 0, 0, 0
        self.pose_sub_x, self.pose_sub_y, self.sub_theta = 0, 0, 0
        self.yaw = 0

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

        # EKF-SLAM Parameters
        self.cylin_threshold = 1.0
        self.cylin_radii = 0.5
        self.reference_cylinder = []
        self.threshold = 0.5  # Cylinder match distance threshold

        # Containers for Cylinder Detection and Mapping
        self.h = np.zeros((2, 1))  # Initial observation matrix
        self.z = np.zeros((2, 1))  # Initial measurement matrix
        self.H = np.zeros((2, 3))  # Initial Jacobian matrix H
        self.K = np.zeros((3, 2))  # Kalman gain matrix
        self.innovation = np.zeros((2, 1))  # Innovation vector
        self.final_mean = np.zeros((3, 1))  # State mean vector
        self.final_covariance = np.zeros(3,3)  # Initial covariance matrix
        self.sigma_r, self.sigma_alpha = 0.1, 0.1  # Measurement standard deviations
        self.g_resultant = np.zeros((3, 1))  # Initial state
        self.G_resultant = np.eye(3)  # State Jacobian
        self.R_resultant = np.zeros((3, 3))
        self.covariance = np.zeros(3,3)  # Initial covariance

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def pose_callback(self,msg):
        self.pose_sub_x = msg.pose.pose.position.x
        self.pose_sub_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        self.sub_theta = self.normalize_angle(self.yaw)

    def motion_model_params_calculation(self):
        x_diff = self.pose_sub_x - self.prev_sub_x
        y_diff = self.pose_sub_y - self.prev_sub_y
        self.delta_d = math.sqrt((x_diff)**2 + (y_diff)**2)
        if(y_diff==0 and x_diff==0):
            self.delta_rot1 = 0
        else :    
            self.delta_rot1 = self.normalize_angle(math.atan2(y_diff,x_diff)-self.prev_sub_theta)
     
        self.delta_rot2 = self.normalize_angle(self.sub_theta - self.prev_sub_theta - self.delta_rot1)

    def pose_prediction(self):
        self.x_predicted = self.prev_sub_x + self.delta_d*math.cos(self.sub_theta + self.delta_rot1)
        self.y_predicted = self.prev_sub_y + self.delta_d*(math.sin(self.sub_theta + self.delta_rot1))
        self.theta_predicted = self.prev_sub_theta + self.delta_rot1 + self.delta_rot2
        self.g_resultant[:3,:] = np.array([
            [self.x_predicted],
            [self.y_predicted],
            [self.theta_predicted]
        ])
    
    def jacobean_robot_wrt_state(self):
        self.dg1dx,self.dg2dx,self.dg3dx = 1,0,-self.delta_d*(math.sin(self.prev_sub_theta+self.delta_rot1))
        self.dg1dy,self.dg2dy,self.dg3dy = 0,1,self.delta_d*(math.cos(self.prev_sub_theta+self.delta_rot1))
        self.dg1dtheta,self.dg2dtheta,self.dg3dtheta = 0,0,1
        self.G_resultant[:3,:3] = np.array([
            [self.dg1dx,self.dg2dx,self.dg3dx],
            [self.dg1dy,self.dg2dy,self.dg3dy],
            [self.dg1dtheta,self.dg2dtheta,self.dg3dtheta]
        ])

    def jacobean_robot_wrt_control(self):
        self.dg1deltarot1,self.dg1deltad,self.dg1deltarot2 = -self.delta_d*(math.sin(self.prev_sub_theta+self.delta_rot1)),math.cos(self.prev_sub_theta+self.delta_rot1),0
        self.dg2deltarot1,self.dg2deltad,self.dg2deltarot2 = self.delta_d*(math.cos(self.prev_sub_theta+self.delta_rot1)),math.sin(self.prev_sub_theta+self.delta_rot1),0
        self.dg3deltarot1,self.dg3deltad,self.dg3deltarot2 = 1,0,1

    def prediction_covariance_calc_for_robot(self):  
        alpha1,alpha2,alpha3,alpha4 = 0.05,0.01,0.05,.01
        rot1_variance = alpha1*(self.delta_rot1) + alpha2*abs(self.delta_d)
        trans_variance = alpha3 * abs(self.delta_d) + alpha4 * (abs(self.delta_rot1) + abs(self.delta_rot2))
        rot2_variance = alpha1 * abs(self.delta_rot2) + alpha2 * abs(self.delta_d)
        self.R_t = np.diag([rot1_variance, trans_variance, rot2_variance])
        self.R_resultant[:3,:3] = self.R_t

    def lidar_callback(self,msg):
        self.lidar_points = [0]*len(msg.ranges)
        for i in range(0,len(msg.ranges)):
            if(math.isnan(msg.ranges[i]) or math.isinf(msg.ranges[i])):
                self.lidar_points[i] = msg.range_max
            else: 
                self.lidar_points[i] = msg.ranges[i]
    
    def derivative_calculation(self):
        if not self.lidar_points:
            rospy.logwarn("Lidar data not available yet")
            return
        self.jumps=[0]*len(self.lidar_points)
        for i in range(1,len(self.lidar_points)-1):
            prev_scan_point = self.lidar_points[i-1]
            next_scan_point = self.lidar_points[i+1]
            derivative = (prev_scan_point+next_scan_point)/2
            if(abs(derivative)>self.cylin_threshold):
                self.jumps[i] = 1

    def cylinder_classification(self):
        sum_of_rays,sum_of_distances,no_of_rays = 0,0,0
        self.linear_distance_from_cylinder,self.angular_position_of_cylinder=[],[]
        on_cylinder = 0
        for i in range(0,len(self.jumps)):
            if(self.jumps[i]<0 and on_cylinder==0):
                on_cylinder = 1
                no_of_rays += 1
                sum_of_distances+=self.lidar_points[i]
                sum_of_rays+=i 

            elif(self.jumps[i]<0 and on_cylinder==1):
                on_cylinder,no_of_rays,sum_of_distances,sum_of_rays = 0,0,0,0
                i-=1
            
            elif(self.jumps[i]>0 and on_cylinder==1):
                self.linear_distance_from_cylinder.append(sum_of_distances/no_of_rays)
                self.angular_position_of_cylinder.append(sum_of_rays/no_of_rays)
                on_cylinder,no_of_rays,sum_of_distances,sum_of_rays = 0,0,0,0

            elif(self.jumps[i]==0 and on_cylinder==1):
                no_of_rays+=1
                sum_of_rays+=i
                sum_of_distances+=self.lidar_points[i]

            else:
                pass
    
    def cartesian_to_lidar(self):
        self.result=[]
        for i in range(0,len(self.linear_distance_from_cylinder)):
            result = self.linear_distance_from_cylinder[i]
            x = result*(math.cos(self.normalize_angle(self.angular_position_of_cylinder[i])))
            y = result*(math.sin(self.normalize_angle(self.angular_position_of_cylinder[i])))
            self.result.append([x,y])

    def cartesian_lidar_to_world(self):
        self.cylinder_final_co_ordinates=[]
        for i in range(0,len(self.result)):
            result_x = self.x_predicted + self.result[i][0]*math.cos(self.normalize_angle(self.theta_predicted)) - self.result[i][1]*math.sin(self.normalize_angle(self.theta_predicted))
            result_y = self.y_predicted + self.result[i][0]*math.sin(self.normalize_angle(self.theta_predicted)) + self.result[i][1]*math.cos(self.normalize_angle(self.theta_predicted))
            self.cylinder_final_co_ordinates.append([result_x,result_y])

    def add_cylinder(self,x,y):
        self.reference_cylinder.append([x,y])
        self.g_cylin = np.array([
            [x],
            [y]
        ])
        self.g_resultant = np.vstack(self.g_resultant,self.g_cylin)
        self.G_cylin = np.eye(2)
        self.G_resultant = np.pad(self.G_resultant,((0,2),(0,2)),constant_values=0)
        self.G_resultant[-2:,-2:] = self.G_cylin
        self.R_resultant = np.pad(self.R_resultant,((0,2),(0,2)),constant_values=0)
        self.final_covariance = np.pad(self.final_covariance,((0,2),(0,2)),constant_values=0)
        init_cylin_covariance = np.diag([100,100])
        self.final_covariance[-2:,-2:] = init_cylin_covariance
        pass

    def matching(self):
        self.match_flag=[-1]*len(self.cylinder_final_co_ordinates)
        for i in range(0,len(self.cylinder_final_co_ordinates)):
            x_predicted = self.cylinder_final_co_ordinates[i][0]
            y_predicted = self.cylinder_final_co_ordinates[i][1]
            if (len(self.reference_cylinder)==0):
                pass
            else:
                for j in range(0,len(self.reference_cylinder)):
                    x_reference = self.reference_cylinder[j][0]
                    y_reference = self.reference_cylinder[j][1]
                    diff_x = x_predicted - x_reference                     
                    diff_y = y_predicted - y_reference
                    dist = math.sqrt((diff_x)**2 + (diff_y)**2)
                    if(dist<self.threshold):
                        self.match_flag[i]=j

    def final_prediction(self):
        self.covariance = np.dot(self.G_resultant,np.dot(self.final_covariance,self.G_resultant.T)) + self.R_resultant
        
    def obs_model(self):
        self.H = np.zeros((2,3+len(self.reference_cylinder)))
        if (len(self.cylinder_final_co_ordinates)!=0):
            for i in range(0,len(self.match_flag)):
                if(self.match_flag[i]!=-1):
                    x_estim = self.cylinder_final_co_ordinates[i][0]
                    y_estim = self.cylinder_final_co_ordinates[i][1]
                    x_prev_estim = self.reference_cylinder[self.match_flag[i]][0]
                    y_prev_estim = self.reference_cylinder[self.match_flag[i]][1]
                    delta_x_estim = x_estim - self.x_predicted
                    delta_y_estim = y_estim - self.y_predicted
                    delta_x_ref = x_prev_estim - self.x_predicted
                    delta_y_ref = y_prev_estim - self.y_predicted
                    dist_estim = math.sqrt((delta_x_estim)**2 + (delta_y_estim)**2)
                    dist_ref = math.sqrt((delta_x_ref)**2 + (delta_y_ref)**2)
                    alpha_estim = self.normalize_angle(math.atan2(delta_y_estim,delta_x_estim) - self.theta_predicted)
                    alpha_ref = self.normalize_angle(math.atan2(delta_y_ref,delta_x_ref) - self.theta_predicted)
                    
                    self.h[2*(self.match_flag[i]):2+2*(self.match_flag[i]),0] = np.array([
                        [dist_ref],
                        [alpha_ref]
                    ])

                    self.z[2*(self.match_flag[i]):2+2*(self.match_flag[i]),0] = np.array([
                        [dist_estim],
                        [alpha_estim]
                    ])

                    self.H[:2, :3] = np.array([
                    [(x_estim / dist_estim), (y_estim / dist_estim), 0],
                    [(-y_estim / (dist_estim**2)), (x_estim / (dist_estim**2)), -1]
                    ])

                    self.H[:2, 3 + 2*self.match_flag[i] : 3 + 2*(self.match_flag[i] + 1)] = -self.H[:2, :2]

                    q_matrix = np.array([
                    [self.sigma_r**2 , 0 ],
                    [0 , self.sigma_alpha**2]
                    ]) #Innovation_covariance

                    self.K = np.array(np.dot(np.dot(self.covariance,self.H),np.linalg.inv((np.dot(self.H,(self.covariance),self.H.T) + q_matrix))))

                    self.innovation[2*(self.match_flag[i]):2*(self.match_flag[i+1]),0] = np.array(self.z - self.h)

                    self.innovation[1+2*(self.match_flag[i])][0] = self.normalize_angle(self.innovation[1+2*(self.match_flag[i])])

                    self.mu = self.g_resultant + np.dot(self.K,self.innovation)
                    
                    dim_I,_ = np.shape(np.dot(self.K,self.H))

                    Identity = np.eye(dim_I)

                    self.final_covariance = np.dot((Identity - np.dot(self.K,self.H)),self.covariance)
                
                else:
                    self.add_cylinder(self.cylinder_final_co_ordinates[i][0],self.cylinder_final_co_ordinates[i][1])
            
        else: 
            self.mu = self.g_resultant
            self.final_covariance = np.dot(Identity,self.covariance)
    def reset(self):
        self.prev_sub_x = self.pose_sub_x
        self.prev_sub_y = self.pose_sub_y
        self.prev_sub_theta = self.sub_theta

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.motion_model_params_calculation()
            self.pose_prediction()
            self.jacobean_robot_wrt_state()
            self.jacobean_robot_wrt_control()
            self.prediction_covariance_calc_for_robot()
            self.derivative_calculation()
            self.cylinder_classification()
            self.cartesian_to_lidar()
            self.cartesian_lidar_to_world()
            self.matching()
            self.final_prediction()
            self.obs_model()
            self.reset()
            rate.sleep()

if __name__ == "__main__":
    try:
        ekf = EKF_SLAM()
        ekf.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
