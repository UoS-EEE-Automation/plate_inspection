#!/usr/bin/python3

import rospy
from std_msgs.msg import String
#from sensor_msgs.msg import Joy
from navic_messages.msg import ScanlinkControl
from geometry_msgs.msg import PoseWithCovarianceStamped
#from strath_meca_control.msg import MecaVelControl
from geometry_msgs.msg import Twist, TwistStamped
# import tf functionality
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformStamped
import geometry_msgs.msg
import actionlib
from plate_inspection.msg import scanAction, scanGoal, scanResult, scanFeedback

import time
import numpy
import math

class RobotController:
    def __init__(self):
        # Set up subscribers
        self.vive_pose_plate = rospy.Subscriber("/vive/LHR_6727695C_pose_filt", PoseWithCovarianceStamped , self.vive_plate_callback, queue_size=1)        
        self.vive_pose_navic = rospy.Subscriber("/vive/LHR_EA821C01_pose_filt", PoseWithCovarianceStamped , self.AtoB_navic_callback, queue_size=1)
        # Set up tf listeners
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Set up publishers
        self.navic_rel_plate = rospy.Publisher("navic_rel_plate", PoseWithCovarianceStamped, queue_size=1)
        self.navic_publisher = rospy.Publisher("/scanlink_control", ScanlinkControl, queue_size=1)
        # Set up subscriber definition & rate
        self.timer_navic_update = rospy.Timer(rospy.Duration(1.0/25.0), self.update_speed_callback)

        # Initialise variables
        self.speed = 0
        self.steer = 0

        self.trans = TransformStamped()

        self.steer_ratio = 100
        self.speed_ratio = 1000    

        self.plate_pose = PoseWithCovarianceStamped()
        self.navic_pose = PoseWithCovarianceStamped()
        self.plate_pose_buffer = []
        self.start_pose = []

        self.waypoints_generated = 0
        self.navic_wp = []

        self.count = 0

        self.insp_stage = 1
        self.stop = 0

    def vive_plate_callback(self, input):
        self.plate_pose = input

        # if self.count >= 2:
        if len(self.plate_pose_buffer) == 0:
            self.plate_pose_buffer = [input.pose.pose.position.x, input.pose.pose.position.y, input.pose.pose.position.z]
        elif len(self.plate_pose_buffer) == 3:
            self.plate_pose_buffer = numpy.vstack(
                [
                    self.plate_pose_buffer, 
                    [
                        input.pose.pose.position.x,
                        input.pose.pose.position.y,
                        input.pose.pose.position.z,
                    ]
                ]
            )
        else:
            self.plate_pose_buffer = numpy.delete(self.plate_pose_buffer, (0), axis=0)
            self.plate_pose_buffer = numpy.vstack(
                [
                    self.plate_pose_buffer,
                    [
                        input.pose.pose.position.x,
                        input.pose.pose.position.y,
                        input.pose.pose.position.z,
                    ]
                ]
            )
            if all(self.plate_pose_buffer[0]) == all(self.plate_pose_buffer[1]) and self.waypoints_generated == 0:
                self.generate_waypoints(self, self.plate_pose_buffer[1])

    def AtoB_navic_callback(self, input):
        if self.count >= 10:
            self.trans = self.tfBuffer.lookup_transform("LHR_6727695C_pose_filt", "navic_control_link", rospy.Time())
            # print(str(self.trans))
        else:
            self.count += 1
            return

        if self.waypoints_generated == 1:

            navic_x = self.trans.transform.translation.x*1000
            navic_y = self.trans.transform.translation.y*1000
            navic_z = self.trans.transform.translation.z*1000
            navic_qx = self.trans.transform.rotation.x
            navic_qy = self.trans.transform.rotation.y
            navic_qz = self.trans.transform.rotation.z
            navic_qw = self.trans.transform.rotation.w

            if self.insp_stage in range(1,4):

                # Calculate Steer
                navic_skew = euler_from_quaternion([navic_qx, navic_qy, navic_qz, navic_qw])
                navic_yaw = (navic_skew[2] * 180/math.pi) % 360 - 180
                # Calculate target skew based upon fixed waypoint:
                # target_skew = math.atan2(self.navic_wp[-self.insp_stage][1] - navic_y, self.navic_wp[-self.insp_stage][0] - navic_x) * 180/math.pi  % 360 - 180
                # Calculate skew based upon point 200mm in front of crawler:
                target_skew = math.atan2(self.navic_wp[-self.insp_stage][1] - navic_y, - 200) * 180/math.pi  % 360 - 180
                theta_err = (target_skew - navic_yaw) 

                # print("Navic Skew: " + str(navic_yaw))
                # print("Target Skew: " + str(target_skew))
                # print("Skew Error: " + str(theta_err))
                # print("Navic X: " + str(navic_x))
                # print("Navic Y: " + str(navic_y))
                # print("Waypoint X: " + str(self.navic_wp[-1][0]))
                # print("Waypoint Y: " + str(self.navic_wp[-1][1]))
                # print("")

                if navic_x > self.navic_wp[-self.insp_stage][0] and self.stop == 0:
                    self.speed = 0.5
                    self.steer = theta_err/-20
                    if abs(self.steer) > 0.8:
                        if self.steer < 0:
                            self.steer = -0.8
                        elif self.steer > 0:
                            self.steer = 0.8
                    # print("Moving forward...   Steer: " + str(self.steer*self.steer_ratio))
                    rospy.logdebug("Moving forward...   Steer: " + str(self.steer*self.steer_ratio))

                elif navic_x <= self.navic_wp[-self.insp_stage][0] and self.stop == 0:
                    self.speed = 0
                    self.steer = 0
                    self.stop = 1
                    # print("Stopping...")
                    rospy.logdebug("Stopping...")
                else:
                    # print("Waypoint " + str(self.insp_stage) + " reached... Pausing for 3s.")
                    rospy.logdebug("Waypoint " + str(self.insp_stage) + " reached... Pausing for 3s.")
                    
                    # Send Action to conduct scan
                    goal = scanGoal()
                    goal.scan_start = 1
                    client.send_goal(goal)
                    client.wait_for_result()
                    print('Scan complete')
                    goal.scan_start = 0
                    # Scan Complete
                    
                    # rospy.sleep(3)
                    self.insp_stage += 1
                    self.stop = 0
            elif self.insp_stage == 4:
                if navic_x < 1100:
                    # print("Returning to home...")
                    rospy.logdebug("Returning to home...")
                    self.speed = -0.8
                    self.steer = 0
                elif navic_x >= 1100:
                    # print("Inspection Complete...")
                    rospy.logdebug("Inspection Complete...")
                    self.speed = 0
                    self.steer = 0
                    
                
    def generate_waypoints(self, *input):
        
        for i in range(1,4,1):
            if i == 1:
                self.navic_wp = [+240+260*(i-1), -419-138.3, 0, 0, 0, 180]
            else:
                self.navic_wp = numpy.vstack(
                    [
                        self.navic_wp,
                        [
                            +240+260*(i-1), 
                            -419-138.3, 
                            0, 
                            0, 
                            0, 
                            180
                        ]
                    ]
                )
        self.waypoints_generated = 1
        #while not rospy.is_shutdown():
        
    def update_speed_callback(self, timer_crawler):
        scanlink_message = ScanlinkControl()
        scanlink_message.speed = int(self.speed * self.speed_ratio)
        scanlink_message.steer = int(self.steer * self.steer_ratio)
        self.navic_publisher.publish(scanlink_message)


if __name__ == '__main__':
    rospy.init_node('BAE_waypoint', anonymous=True, log_level=rospy.DEBUG)
    robot = RobotController() 
    rospy.spin()

    rospy.init_node('scan_action_client')
    client = actionlib.SimpleActionClient('scan', scanAction)
    client.wait_for_server()
