#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32

class BatterySimulator:
    def __init__(self, initial_battery=100, discharge_rate=1, charge_rate=10):
        self.battery_level = initial_battery
        self.discharge_rate = discharge_rate
        self.charge_rate = charge_rate

        rospy.init_node('battery_simulator', anonymous=True)
        rospy.Subscriber('battery_status', String, self.charge_control_callback)
        self.battery_pub = rospy.Publisher('battery_level', Float32, queue_size=10)

        self.charge_pub = rospy.Publisher('battery_status', String, queue_size=10)

        self.rate = rospy.Rate(1)  # 1 Hz

        self.battery_status="READY"

    def charge_control_callback(self, msg):
        self.battery_status=msg.data
       
        if self.battery_status=="CHARGING":
            pass
        else:
            rospy.logwarn("Invalid charge control command")

    def discharge_battery(self):
        while not rospy.is_shutdown() and self.battery_status=="READY":

            if self.battery_level > 0:
                self.battery_level -= self.discharge_rate
                self.battery_pub.publish(self.battery_level)
            else:
                self.battery_pub.publish(self.battery_level)
            self.rate.sleep()

    def charge_battery(self):
        while not rospy.is_shutdown() and self.battery_status=="CHARGING":

            if self.battery_level < 100:
                self.battery_level += self.charge_rate
                if self.battery_level>100:
                    self.battery_level=100
                self.battery_pub.publish(self.battery_level)
            elif self.battery_level == 100:
                self.battery_status="READY"
                self.battery_pub.publish(self.battery_level)
                self.charge_pub.publish(self.battery_status)
            self.rate.sleep()

    def run(self):
        rospy.loginfo("Battery simulation started. Listening for charge control commands.")
        while not rospy.is_shutdown():
            if self.battery_level <= 0:
                self.battery_level = 0
                # self.charge_battery()
            elif self.battery_level >= 100 and self.battery_status=="READY":
                self.battery_level = 100
                self.discharge_battery()

            if self.battery_status == "CHARGING":
                self.charge_battery()

            # print(self.battery_status)

            self.rate.sleep()

if __name__ == '__main__':
    battery_simulator = BatterySimulator()
    battery_simulator.run()
