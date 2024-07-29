#!/usr/bin/python3

import rospy
from std_msgs.msg import String
#from sensor_msgs.msg import Joy
from navic_messages.msg import ScanlinkControl
from geometry_msgs.msg import PoseWithCovarianceStamped
#from strath_meca_control.msg import MecaVelControl
from geometry_msgs.msg import Twist, TwistStamped

import time

class RobotController:
    def __init__(self):
        self.vive_pose_crawler = rospy.Subscriber("/vive/LHR_EA821C01_pose", PoseWithCovarianceStamped , self.AtoB_navic_callback, queue_size=1)
        # self.vive_pose_plate = rospy.Subscriber("/vive/LHR_6727695C_pose", PoseWithCovarianceStamped , self.vive_plate_callback, queue_size=1)        
        self.navic_publisher = rospy.Publisher("/scanlink_control", ScanlinkControl, queue_size=1)
        self.timer_crawler_update = rospy.Timer(rospy.Duration(1.0/25.0), self.update_speed_callback)
        self.speed = 0
        self.steer = 0

        self.steer_ratio = 100
        self.speed_ratio = 1000    

        self.start_pose = 0
        self.forward = True

        self.plate_pose = PoseWithCovarianceStamped()
        self.navic_pose = PoseWithCovarianceStamped()
    
    def vive_plate_callback(self, input):
        self.plate_pose = input


    def AtoB_navic_callback(self, input):
        navic_x = input.pose.pose.position.x
        navic_y = input.pose.pose.position.y
        navic_z = input.pose.pose.position.z
        if self.start_pose == 0:
            self.start_pose = [navic_x, navic_y, navic_z]
            print("Initial Pose: " + str(self.start_pose))  

        if self.forward:
            self.speed = 1
            self.steer = 0
            print("Forward Motion: " + str(["%.4f" % navic_x, "%.4f" % navic_y, "%.4f" % navic_z]))
            if navic_x >= self.start_pose[0]+0.05:
                self.forward = False
                self.speed = 0
                print("Hold Position...")
                time.sleep(1)
        else:
            self.speed = -1
            self.steer = 0
            print("Reverse Motion: " + str(["%.4f" % navic_x, "%.4f" % navic_y, "%.4f" % navic_z]))
            if navic_x <= self.start_pose[0]:
                self.forward = True
                self.speed = 0
                print("Hold Position...")
                time.sleep(1)
                

        
        #while not rospy.is_shutdown():
        
    def update_speed_callback(self, timer_crawler):
        scanlink_message = ScanlinkControl()
        scanlink_message.speed = int(self.speed * self.speed_ratio)
        scanlink_message.steer = int(self.steer * self.steer_ratio)
        self.navic_publisher.publish(scanlink_message)


if __name__ == '__main__':
    rospy.init_node('navic_AtoB', anonymous=True)
    robot = RobotController() 
    rospy.spin()