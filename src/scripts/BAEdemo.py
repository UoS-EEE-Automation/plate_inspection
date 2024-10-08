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
import transforms3d
from tf2_ros import TransformStamped
import geometry_msgs.msg
import actionlib
from plate_inspection.msg import scanAction, scanGoal, scanResult, scanFeedback

import time
import numpy
import math
from geometry_msgs.msg import Pose


class RobotController:
    def __init__(self):
        # Initialise variables
        self.speed = 0
        self.steer = 0

        self.trans = TransformStamped()
        self.trans_probe = TransformStamped()
        self.trans_probe_2 = TransformStamped()

        self.steer_ratio = 100
        self.speed_ratio = 1000    

        self.plate_pose = PoseWithCovarianceStamped()
        self.navic_pose = PoseWithCovarianceStamped()
        self.plate_pose_buffer = []
        self.start_pose = []

        self.waypoints_generated = 0
        self.navic_wp = []
        self.probe_wp = []

        self.count = 0

        self.insp_stage = 1
        self.stop = 0

        self.debug_msg_count = 0

        # Set up tf listeners
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Set up publishers
        self.navic_rel_plate = rospy.Publisher("navic_rel_plate", PoseWithCovarianceStamped, queue_size=1)
        self.navic_publisher = rospy.Publisher("/scanlink_control", ScanlinkControl, queue_size=1)
        # Set up subscriber definition & rate
        self.timer_navic_update = rospy.Timer(rospy.Duration(1.0/25.0), self.update_speed_callback)
        # Set up tf publishers for waypoints
        self.probe_wp_broadcaster = tf2_ros.TransformBroadcaster()
        self.navic_wp_broadcaster = tf2_ros.TransformBroadcaster()
        self.probe_target_transformStamped = geometry_msgs.msg.TransformStamped()
        self.navic_target_transformStamped = geometry_msgs.msg.TransformStamped()

        # Set up subscribers
        self.vive_pose_plate = rospy.Subscriber("/vive/LHR_6727695C_pose_filt", PoseWithCovarianceStamped , self.vive_plate_callback, queue_size=1)        
        self.vive_pose_navic = rospy.Subscriber("/vive/LHR_EA821C01_pose_filt", PoseWithCovarianceStamped , self.waypoint_navic_callback, queue_size=1)
        
        # Create action client
        self.act_client = actionlib.SimpleActionClient('scan', scanAction)
        self.act_client.wait_for_server()
        self.scan_start = Pose()

    def vive_plate_callback(self, input):
        self.plate_pose = input

        
        if self.waypoints_generated == 0:
            self.generate_waypoints()

        if self.waypoints_generated == 1 and self.insp_stage < 4:
            ################################################################## Publish the current probe target frame
            self.probe_target_transformStamped.header.stamp = rospy.Time.now()
            self.probe_target_transformStamped.header.frame_id = "LHR_6727695C_pose_filt"
            self.probe_target_transformStamped.child_frame_id = "probe_target"

            probe_wp_q = transforms3d.euler.euler2quat(
                self.probe_wp[-self.insp_stage][3]*math.pi/180, 
                self.probe_wp[-self.insp_stage][4]*math.pi/180, 
                self.probe_wp[-self.insp_stage][5]*math.pi/180
                )

            self.probe_target_transformStamped.transform.translation.x = float(self.probe_wp[-self.insp_stage][0]/1000)
            self.probe_target_transformStamped.transform.translation.y = float(self.probe_wp[-self.insp_stage][1]/1000)
            self.probe_target_transformStamped.transform.translation.z = float(self.probe_wp[-self.insp_stage][2]/1000)

            self.probe_target_transformStamped.transform.rotation.w = probe_wp_q[0]
            self.probe_target_transformStamped.transform.rotation.x = probe_wp_q[1]
            self.probe_target_transformStamped.transform.rotation.y = probe_wp_q[2]
            self.probe_target_transformStamped.transform.rotation.z = probe_wp_q[3]

            ################################################################## Publish the current navic target frame
            self.navic_target_transformStamped.header.stamp = rospy.Time.now()
            self.navic_target_transformStamped.header.frame_id = "LHR_6727695C_pose_filt"
            self.navic_target_transformStamped.child_frame_id = "navic_target"

            navic_wp_q = transforms3d.euler.euler2quat(
                self.navic_wp[-self.insp_stage][3]*math.pi/180, 
                self.navic_wp[-self.insp_stage][4]*math.pi/180, 
                self.navic_wp[-self.insp_stage][5]*math.pi/180
                )

            self.navic_target_transformStamped.transform.translation.x = float(self.navic_wp[-self.insp_stage][0]/1000)
            self.navic_target_transformStamped.transform.translation.y = float(self.navic_wp[-self.insp_stage][1]/1000)
            self.navic_target_transformStamped.transform.translation.z = float(self.navic_wp[-self.insp_stage][2]/1000)

            self.navic_target_transformStamped.transform.rotation.w = navic_wp_q[0]
            self.navic_target_transformStamped.transform.rotation.x = navic_wp_q[1]
            self.navic_target_transformStamped.transform.rotation.y = navic_wp_q[2]
            self.navic_target_transformStamped.transform.rotation.z = navic_wp_q[3]

    def waypoint_navic_callback(self, input):
        self.read_navic_position()

        self.debug_msg_count += 1    

    def read_navic_position(self):
        if self.count >= 10:
            self.trans = self.tfBuffer.lookup_transform("LHR_6727695C_pose_filt", "navic_control_link", rospy.Time())
        else:
            self.count += 1
            return

        if self.waypoints_generated == 1:
            self.navic_x = self.trans.transform.translation.x*1000
            self.navic_y = self.trans.transform.translation.y*1000
            self.navic_z = self.trans.transform.translation.z*1000
            self.navic_qx = self.trans.transform.rotation.x
            self.navic_qy = self.trans.transform.rotation.y
            self.navic_qz = self.trans.transform.rotation.z
            self.navic_qw = self.trans.transform.rotation.w

            self.calculate_wheel_speeds()

    def calculate_wheel_speeds(self):
        if self.insp_stage in range(1,4):

            # Calculate Steer
            navic_skew = euler_from_quaternion([self.navic_qx, self.navic_qy, self.navic_qz, self.navic_qw])
            navic_yaw = (navic_skew[2] * 180/math.pi) % 360 - 180
            # Calculate target skew based upon fixed waypoint:
            # target_skew = math.atan2(self.navic_wp[-self.insp_stage][1] - navic_y, self.navic_wp[-self.insp_stage][0] - navic_x) * 180/math.pi  % 360 - 180
            # Calculate skew based upon point 200mm in front of crawler:
            target_skew = math.atan2(self.navic_wp[-self.insp_stage][1] - self.navic_y, - 200) * 180/math.pi  % 360 - 180
            theta_err = (target_skew - navic_yaw) 

            # print("Navic Skew: " + str(navic_yaw))
            # print("Target Skew: " + str(target_skew))
            # print("Skew Error: " + str(theta_err))
            # print("Navic X: " + str(navic_x))
            # print("Navic Y: " + str(navic_y))
            # print("Waypoint X: " + str(self.navic_wp[-1][0]))
            # print("Waypoint Y: " + str(self.navic_wp[-1][1]))
            # print("")

            if self.navic_x > self.navic_wp[-self.insp_stage][0] and self.stop == 0:
                self.speed = 0.5
                # if abs(navic_x - self.navic_wp[-self.insp_stage][0]) < 100:
                #     self.speed = 0.2
                if abs(self.navic_x - self.navic_wp[-self.insp_stage][0]) <= 100 and abs(self.navic_x - self.navic_wp[-self.insp_stage][0]) > 10:
                    self.speed = abs(self.navic_x - self.navic_wp[-self.insp_stage][0])/100 * self.speed
                elif abs(self.navic_x - self.navic_wp[-self.insp_stage][0]) < 10:
                    self.speed = 0.05

                self.steer = theta_err/-15
                if abs(self.steer) > 0.8:
                    if self.steer < 0:
                        self.steer = -0.8
                    elif self.steer > 0:
                        self.steer = 0.8
                # print("Moving forward...   Steer: " + str(self.steer*self.steer_ratio))
                if self.debug_msg_count%50 == 0:
                    rospy.logdebug("Moving forward...   Speed: " + str(int(self.speed*self.speed_ratio)) + "   Steer: " + str(int(self.steer*self.steer_ratio)))

            elif self.navic_x <= self.navic_wp[-self.insp_stage][0] and self.stop == 0:
                self.speed = 0
                self.steer = 0
                self.stop = 1
                # print("Stopping...")
                rospy.logdebug("Stopping...")
            else:
                # print("Waypoint " + str(self.insp_stage) + " reached... Pausing for 3s.")
                rospy.logdebug("Waypoint " + str(self.insp_stage) + " reached... Approaching sample....")

                # Calculate transform between Meca base and probe start waypoint, this is fed through to the robot controller                    
                self.trans_probe = self.tfBuffer.lookup_transform("probe_target", "meca_base_link", rospy.Time()) # Provides correct angle
                self.trans_probe_2 = self.tfBuffer.lookup_transform("meca_base_link", "probe_target", rospy.Time()) # Provides correct translation

                target_eul = transforms3d.euler.quat2euler(
                    [
                    self.trans_probe.transform.rotation.w,
                    self.trans_probe.transform.rotation.x,
                    self.trans_probe.transform.rotation.y,
                    self.trans_probe.transform.rotation.z
                    ]
                )
                target_eul = numpy.array(target_eul)*-180/math.pi

                # Send Action to conduct scan
                goal = scanGoal()

                probe_target_pose = [
                    self.trans_probe_2.transform.translation.x*1000-20, # Convert from m to mm
                    self.trans_probe_2.transform.translation.y*1000,
                    self.trans_probe_2.transform.translation.z*1000,
                    target_eul[0], # Convert from rad to deg
                    0, #target_eul[1] We want to set this value ourself
                    target_eul[2]
                    ]
                
                goal.scan_start_pose = probe_target_pose
                goal.scan_length = 240
                self.act_client.send_goal(goal)
                self.act_client.wait_for_result()
                rospy.logdebug("Scan " + str(self.insp_stage) + " complete.")
                # goal.scan_start = 0

                # Scan Complete

                rospy.sleep(3)
                self.insp_stage += 1
                self.stop = 0
        elif self.insp_stage == 4:
            self.go_home()

    def go_home(self):    
        if self.navic_x < 1100:
            # print("Returning to home...")
            if self.debug_msg_count%50 == 0:
                rospy.logdebug("Returning to home...")
            self.speed = -0.8
            self.steer = 0
        elif self.navic_x >= 1100:
            # print("Inspection Complete...")
            self.speed = 0
            self.steer = 0
            self.insp_stage += 1
        elif self.insp_stage == 5:
            if self.debug_msg_count%50 == 0:
                rospy.logdebug("Inspection Complete...")
            self.speed = 0
            self.steer = 0
                        
                
    def generate_waypoints(self):
        
        for i in range(1,4,1):
            if i == 1:
                self.navic_wp = [+240+260*(i-1), -419-138.3, 0, 0, 0, 180]
                self.probe_wp = [+240-53.19-20+125+260*(i-1), -170, 0, 0, 0, 90]
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
                self.probe_wp = numpy.vstack(
                [
                    self.probe_wp,
                    [
                        +240-53.19-20+125+260*(i-1), 
                        -170, 
                        0, 
                        0, 
                        0, 
                        90
                    ]
                ]
                )
        self.waypoints_generated = 1
        
    def update_speed_callback(self, timer_crawler):
        if self.waypoints_generated == 1:
            self.probe_wp_broadcaster.sendTransform(self.probe_target_transformStamped)
            self.navic_wp_broadcaster.sendTransform(self.navic_target_transformStamped)
        scanlink_message = ScanlinkControl()
        scanlink_message.speed = int(self.speed * self.speed_ratio)
        scanlink_message.steer = int(self.steer * self.steer_ratio)
        self.navic_publisher.publish(scanlink_message)


if __name__ == '__main__':
    rospy.init_node('BAE_waypoint', anonymous=True, log_level=rospy.DEBUG)
    robot = RobotController() 
    rospy.spin()


