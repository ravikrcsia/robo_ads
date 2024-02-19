#!/usr/bin/env python3

import rospy
import actionlib
import math
from std_msgs.msg import String, Float32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Trigger, TriggerRequest

class RoboNav:
    def __init__(self):

        rospy.init_node('robo_nav', anonymous=True)

        self.nav_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.nav_client.wait_for_server()

        rospy.wait_for_service('/docking_service')

        rospy.Subscriber('battery_level', Float32, self.battery_check)
        rospy.Subscriber('battery_status', String, self.battery_status_callback)
        rospy.Subscriber('charging_dock_pose', PoseStamped, self.dock_pose_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        self.charge_pub = rospy.Publisher('battery_status', String, queue_size=10)

        self.gl_rate = rospy.Rate(2)  # 5 Hz

        self.goals=[
                [0, 1, 1.57],
                [0, 4, 0],
                [3, 3, 3.14],
                [0.5, -3, 1.57]
                ]
        
        self.dock_point={}

        self.current_goal=0
        self.nav_mode = "GOAL_LOOP" #"STAY" #"DOCK" #"CHARGING"
        self.battery_level=0
        self.battery_status=""

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

    def battery_check(self, msg):

        self.battery_level=msg.data

        if self.battery_level < 30 and self.nav_mode != "CHARGING":
            self.nav_mode = "DOCK"
            rospy.loginfo("Will dock the charging after this goal")
        elif self.battery_level >= 100:
            self.nav_mode = "GOAL_LOOP"

    def battery_status_callback(self, msg):
        self.battery_status=msg.data

    def normalize_angle(self, angle):

        ''' Converting angle to the shortest angle possible '''
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle
    
    def odometry_callback(self, msg):
        # Extract the robot's current pose from the odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract the robot's orientation (yaw) from the odometry message
        orientation = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])


    def dock_pose_callback(self, msg):
        
        rospy.loginfo_once("DOCK PORT DETECTED !")
        x,y = msg.pose.position.x, msg.pose.position.y
        orientation = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.x, orientation.y, orientation.z, orientation.w])
        yaw= self.normalize_angle(yaw)
        self.dock_point[msg.header.frame_id]=(x,y,0.0)

    def movebase_client(self, goal_pose):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pose[0]
        goal.target_pose.pose.position.y = goal_pose[1]
        q=quaternion_from_euler(0,0,goal_pose[2])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.nav_client.send_goal(goal)
        wait = self.nav_client.wait_for_result()
        if wait:
            return self.nav_client.get_result()
        
    def goal_looping(self):

        while not rospy.is_shutdown() and self.nav_mode=="GOAL_LOOP":
            print(self.current_goal)
            result = self.movebase_client(self.goals[self.current_goal])

            if result:
                rospy.loginfo("Goal {} execution done!".format(self.current_goal+1))
            else:
                rospy.logwarn("Goal {} execution FAIL!".format(self.current_goal+1))
            
            if self.current_goal < 3:
                self.current_goal+=1
            else:
                self.current_goal=0

            self.gl_rate.sleep()

    def move_to_dock(self):

        goal_dock = self.dock_point[list(self.dock_point)[0]]
        goal_x, goal_y = goal_dock[0], goal_dock[1]

        angle=0.0

        if (abs(angle) < 0.52) or (abs(angle) < 3.14 and abs(angle) > 2.62):
            if self.robot_x > goal_x:
                goal_x+=1.0
            else:
                goal_x-=1.0
        else:
            if self.robot_y > goal_y:
                goal_y+=1.0
            else:
                goal_y-=1.0

        result = self.movebase_client((goal_x, goal_y, angle))
        if result:
            rospy.loginfo("Moving to Dock for Charging >> done!")
            self.nav_mode = "STAY"
        else:
            rospy.logwarn("Moving to Dock for Charging >> FAIL!")

    def trigger_docking(self):
        dock_service = rospy.ServiceProxy('/docking_service', Trigger)

        result = dock_service(TriggerRequest())

        # print(result.success, result.message)

        return result


if __name__ == '__main__':

    robo_nav = RoboNav()

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if robo_nav.nav_mode=="GOAL_LOOP":
            print("GOING TO LOOP")
            robo_nav.goal_looping()
        elif robo_nav.nav_mode=="DOCK":
            print("GOING TO DOCK")
            robo_nav.move_to_dock()
        elif robo_nav.nav_mode=="STAY":
            print("DOCKING....!!!!")
            result=robo_nav.trigger_docking()

            if result.success:
                print(result.message)
                robo_nav.battery_status="CHARGING"
                robo_nav.charge_pub.publish(robo_nav.battery_status)

                robo_nav.nav_mode="CHARGING"
            else:
                rospy.logerr("DOCKING FAILED, NOT CHARGING !")
        elif robo_nav.nav_mode=="CHARGING":
            print("ROBO is Charging up!")
        
        rate.sleep()
