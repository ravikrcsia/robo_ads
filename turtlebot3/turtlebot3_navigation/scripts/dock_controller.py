#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse


class DockingController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('point_controller_node', anonymous=True)

        # Create a publisher for sending velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to the robot's odometry to get the current pose
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        rospy.Subscriber('scan', LaserScan, self.laser_callback)

        # Service
        dock_service = rospy.Service('/docking_service', Trigger, self.docking_trigger)

        rospy.Subscriber('charging_dock_pose', PoseStamped, self.dock_pose_callback)

        self.distance_threshold = 0.02
        self.linear_velocity = 1.0  # Adjust as needed
        self.kp_linear = 1.0  # Proportional gain for linear velocity
        self.kp_angular = 6.0  # Proportional gain for angular velocity
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        #Lasor Data from LiDAR
        self.lidar_regions={}

        self.dock_point={}

    def odometry_callback(self, msg):
        # Extract the robot's current pose from the odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract the robot's orientation (yaw) from the odometry message
        orientation = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def laser_callback(self,msg):

        ''' LiDAR sensor data, to detect the dock distance'''

        self.lidar_regions = {
            'fright': min(min(msg.ranges[0:119]),10),
            'front': min(min(msg.ranges[120:239]),10),
            'fleft': min(min(msg.ranges[240:359]),10),
            'bleft': min(min(msg.ranges[360:479]),10),
            'back': min(min(msg.ranges[480:599]),10),
            'bright': min(min(msg.ranges[600:719]),10)
        }

        # print(self.regions)

    def dock_pose_callback(self, msg):
        
        x,y = msg.pose.position.x, msg.pose.position.y
        orientation = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.x, orientation.y, orientation.z, orientation.w])
        yaw= self.normalize_angle(yaw)

        # print(yaw)
        self.dock_point[msg.header.frame_id]=(x,y,0.0)

        # print(self.dock_point)

    def docking_trigger(self, request):
        self.start_docking=True

        goal_dock = self.dock_point[list(self.dock_point)[0]]
        goal_x, goal_y = goal_dock[0], goal_dock[1]

        angle=0.0

        if (abs(angle) < 0.52) or (abs(angle) < 3.14 and abs(angle) > 2.62):
            if self.robot_x > goal_x:
                goal_x+=0.5
            else:
                goal_x-=0.5
        else:
            if self.robot_y > goal_y:
                goal_y+=0.5
            else:
                goal_y-=0.5

        # print(goal_x, goal_y)

        start_time=rospy.get_time()
        end_time1 = self.move_to_dock_pose(goal_x, goal_y)
        # print("Moving to pre-dock pose",rospy.Time.secs()-start_time)
        end_time2 = self.dock_it(angle)

        px="Docking time: "+str(end_time2-start_time)+"sec "+"; TF distances (x,y):"+str(abs(self.robot_x-goal_x))+", "+str(abs(self.robot_y-goal_y))
        print(px)

        return TriggerResponse( success=True, message=px)

    
    def calculate_error(self, goal_x, goal_y):
        # Calculate the distance to the goal
        distance_to_goal = ((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)**0.5

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(goal_y - self.robot_y, goal_x - self.robot_x)

        # Calculate the heading error (difference in yaw angles)
        heading_error = angle_to_goal - (self.robot_yaw+ math.pi)

        return distance_to_goal, heading_error
    
    def normalize_angle(self, angle):

        ''' Converting angle to the shortest angle possible '''
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def move_to_dock_pose(self, goal_x, goal_y):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            distance_to_goal, heading_error = self.calculate_error(goal_x, goal_y)

            if distance_to_goal < self.distance_threshold:
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0  
                cmd_vel_msg.angular.z = 0.0

                end_time = rospy.get_time()
                # Publish the Twist message
                self.cmd_vel_pub.publish(cmd_vel_msg)
                rospy.loginfo("Goal reached!")
                
                return end_time

            # Proportional control for linear and angular velocities
            linear_error = self.kp_linear * distance_to_goal
            angular_error = self.kp_angular * self.normalize_angle(heading_error)

            # print(heading_error, self.normalize_angle(heading_error))

            # Create a Twist message to send velocity commands
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = -min(linear_error, self.linear_velocity)  # Limit linear velocity
            cmd_vel_msg.angular.z = angular_error

            # Publish the Twist message
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # print(self.dock_point)

            # Sleep to control the rate of publishing
            rate.sleep()

    def dock_it(self, angle):
        rate = rospy.Rate(10)  # 10 Hz

        # time_new = rospy.Time.now()

        min_distance_dock = 0.1
        linear_tolerance = 0.02
        angular_tolerance = 0.02

        kp_angular=3.0
        is_controller_running_angular=True
        kp_linear=0.5
        is_controller_running_linear =False

        while not rospy.is_shutdown():

            if is_controller_running_angular:
                error_z = self.normalize_angle(angle - self.robot_yaw)
                twist_msg = Twist()
                if round(abs(error_z), 3) > angular_tolerance:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = kp_angular*error_z

                    self.cmd_vel_pub.publish(twist_msg)
                else:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0

                    self.cmd_vel_pub.publish(twist_msg)

                    print("Oriented towards goal")
                    is_controller_running_angular = False
                    is_controller_running_linear = True

            if is_controller_running_linear:

                twist_msg = Twist()

                distance_from_dock = self.lidar_regions['back']
                error_x = distance_from_dock - min_distance_dock

                linear_velocity = -kp_linear * error_x
                angular_velocity = 0.0

                if abs(error_x) > linear_tolerance:
                    twist_msg.linear.x = linear_velocity
                    twist_msg.angular.z = angular_velocity

                    self.cmd_vel_pub.publish(twist_msg)

                else:
                    
                    end_time = rospy.get_time()

                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0

                    self.cmd_vel_pub.publish(twist_msg)

                    print(f'Reached Goal Successfully !!!')

                    is_controller_running_linear = False
                    return end_time

            # Sleep to control the rate of publishing
            rate.sleep()

if __name__ == '__main__':
    # Specify the goal coordinates (adjust these values accordingly)
    

    # Create the PointController instance
    docking_controller = DockingController()

    rospy.spin()

    # try:
        
    # except rospy.ROSInterruptException:
    #     rospy.logerr("Interrupted by user.")
