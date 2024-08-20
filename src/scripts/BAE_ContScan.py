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
from geometry_msgs.msg import Pose, PoseArray


class RobotController:
    def __init__(self):
        # Set up subscribers
        self.vive_pose_plate = rospy.Subscriber("/vive/LHR_6727695C_pose_filt", PoseWithCovarianceStamped , self.vive_plate_callback, queue_size=1)        
        self.vive_pose_navic = rospy.Subscriber("/vive/LHR_EA821C01_pose_filt", PoseWithCovarianceStamped , self.waypoint_navic_callback, queue_size=1)
        # Set up tf listeners
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Set up publishers
        self.navic_rel_plate = rospy.Publisher("navic_rel_plate", PoseWithCovarianceStamped, queue_size=1)
        self.navic_publisher = rospy.Publisher("/scanlink_control", ScanlinkControl, queue_size=1)
        # Set up subscriber definition & rate
        self.timer_navic_update = rospy.Timer(rospy.Duration(1.0/25.0), self.update_speed_callback)
        # Set up tf publishers for current waypoint
        self.probe_wp_broadcaster = tf2_ros.TransformBroadcaster()
        self.navic_wp_broadcaster = tf2_ros.TransformBroadcaster()
        self.probe_target_transformStamped = geometry_msgs.msg.TransformStamped()
        self.navic_target_transformStamped = geometry_msgs.msg.TransformStamped()
        # Set up publisher for waypoint arrays
        self.probe_wp_publisher = rospy.Publisher('probe_targets', PoseArray)
        self.navic_wp_publisher = rospy.Publisher('navic_targets', PoseArray)
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
        self.navic_wp = PoseArray()
        self.probe_wp = PoseArray()
        self.no_wp = 46

        self.count = 0

        self.insp_stage = 1
        self.stop = 0

        self.meca_cmd_vel = Twist()

        # Create action client
        self.act_client = actionlib.SimpleActionClient('scan', scanAction)
        self.act_client.wait_for_server()
        self.scan_start = Pose()
        # Create action client
        self.act_client = actionlib.SimpleActionClient('cont_scan', scanAction)
        self.act_client.wait_for_server()
        self.scan_start = Pose()

        # Create action client
        self.act_client = actionlib.SimpleActionClient('scan', scanAction)
        self.act_client.wait_for_server()
        self.scan_start = Pose()


    def vive_plate_callback(self, input):
        self.plate_pose = input

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

        if self.waypoints_generated == 1 and self.insp_stage < 4:
            ################################################################## Publish the current probe target frame
            self.probe_target_transformStamped.header.stamp = rospy.Time.now()
            self.probe_target_transformStamped.header.frame_id = "LHR_6727695C_pose_filt"
            self.probe_target_transformStamped.child_frame_id = "probe_target"

            self.probe_target_transformStamped.transform.translation.x = float(self.probe_wp.poses[0].position.x)
            self.probe_target_transformStamped.transform.translation.y = float(self.probe_wp.poses[0].position.y)
            self.probe_target_transformStamped.transform.translation.z = float(self.probe_wp.poses[0].position.z)

            self.probe_target_transformStamped.transform.rotation.w = self.probe_wp.poses[0].orientation.w
            self.probe_target_transformStamped.transform.rotation.x = self.probe_wp.poses[0].orientation.x
            self.probe_target_transformStamped.transform.rotation.y = self.probe_wp.poses[0].orientation.y
            self.probe_target_transformStamped.transform.rotation.z = self.probe_wp.poses[0].orientation.z

            ################################################################## Publish the current navic target frame
            self.navic_target_transformStamped.header.stamp = rospy.Time.now()
            self.navic_target_transformStamped.header.frame_id = "LHR_6727695C_pose_filt"
            self.navic_target_transformStamped.child_frame_id = "navic_target"

            self.navic_target_transformStamped.transform.translation.x = float(self.navic_wp.poses[0].position.x)
            self.navic_target_transformStamped.transform.translation.y = float(self.navic_wp.poses[0].position.y)
            self.navic_target_transformStamped.transform.translation.z = float(self.navic_wp.poses[0].position.z)

            self.navic_target_transformStamped.transform.rotation.w = self.navic_wp.poses[0].orientation.w
            self.navic_target_transformStamped.transform.rotation.x = self.navic_wp.poses[0].orientation.x
            self.navic_target_transformStamped.transform.rotation.y = self.navic_wp.poses[0].orientation.y
            self.navic_target_transformStamped.transform.rotation.z = self.navic_wp.poses[0].orientation.z

    def waypoint_navic_callback(self, input):
        if self.count >= 10:
            self.trans = self.tfBuffer.lookup_transform("LHR_6727695C_pose_filt", "navic_control_link", rospy.Time())
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

            
            navic_target_x = self.navic_wp.poses[0].position.x
            navic_target_y = self.navic_wp.poses[0].position.y
            navic_target_z = self.navic_wp.poses[0].position.z

            if self.insp_stage in range(1,4):

                # Calculate Steer
                navic_skew = euler_from_quaternion([navic_qx, navic_qy, navic_qz, navic_qw])
                navic_yaw = (navic_skew[2] * 180/math.pi) % 360 - 180
                # Calculate skew based upon point 200mm in front of crawler:
                target_skew = math.atan2(self.navic_wp.poses[0].position.y - navic_y, - 200) * 180/math.pi  % 360 - 180
                theta_err = (target_skew - navic_yaw) 

                if navic_x > self.navic_wp.poses[0].position.x and self.stop == 0:
                    self.speed = 0.5
                    if abs(navic_x - self.navic_wp.poses[0].position.x) < 100:
                        self.speed = 0.2
                    self.steer = theta_err/-15
                    if abs(self.steer) > 0.8:
                        if self.steer < 0:
                            self.steer = -0.8
                        elif self.steer > 0:
                            self.steer = 0.8
                    # print("Moving forward...   Steer: " + str(self.steer*self.steer_ratio))
                    rospy.logdebug("Moving forward...   Speed: " + str(self.speed*self.speed_ratio) + "   Steer: " + str(self.steer*self.steer_ratio))

                elif navic_x <= self.navic_wp.poses[0].position.x and self.stop == 0:
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
                        target_eul[1],
                        target_eul[2]
                        ]
                    
                    goal.scan_start = probe_target_pose
                    self.act_client.send_goal(goal)
                    self.act_client.wait_for_result()
                    rospy.logdebug("Scan " + str(self.insp_stage) + " complete.")
                    # goal.scan_start = 0

                    # Scan Complete

                    rospy.sleep(3)
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
                    self.speed = 0
                    self.steer = 0
                    self.insp_stage += 1
            elif self.insp_stage == 5:
                    rospy.logdebug("Inspection Complete...")
                    self.speed = 0
                    self.steer = 0
                    
                
    def generate_waypoints(self, *input):
        self.probe_wp.header.frame_id = "LHR_6727695C_pose_filt"
        self.probe_wp.header.stamp = rospy.Time.now()
        self.navic_wp.header.frame_id = "LHR_6727695C_pose_filt"
        self.probe_wp.header.stamp = rospy.Time.now()
        scan_length = 900
        for i in range(self.no_wp):
            probe_wp_fill = Pose()
            navic_wp_fill = Pose()
            probe_wp_fill.position.x = (scan_length-((scan_length/(self.no_wp-1))*i))/1000
            probe_wp_fill.position.y = -180/1000
            probe_wp_fill.position.z = 0
            probe_wp_fill.orientation.x = 0
            probe_wp_fill.orientation.y = 0
            probe_wp_fill.orientation.z = 0.707
            probe_wp_fill.orientation.w = 0.707 # Eul: 0, 0, 90

            self.probe_wp.poses.append(probe_wp_fill)

            navic_wp_fill.position.x = (scan_length+20+53.19-((scan_length/(self.no_wp-1))*i))/1000
            navic_wp_fill.position.y = (-419-138.3)/1000
            navic_wp_fill.position.z = 0
            navic_wp_fill.orientation.x = 0
            navic_wp_fill.orientation.y = 0
            navic_wp_fill.orientation.z = 1
            navic_wp_fill.orientation.w = 0 # Eul: 0, 0, 180

            self.navic_wp.poses.append(navic_wp_fill)

        self.waypoints_generated = 1
        
    def update_speed_callback(self, timer_crawler):
        if self.waypoints_generated == 1:
            self.probe_wp_broadcaster.sendTransform(self.probe_target_transformStamped)
            self.navic_wp_broadcaster.sendTransform(self.navic_target_transformStamped)
            self.probe_wp_publisher.publish(self.probe_wp)
            self.navic_wp_publisher.publish(self.navic_wp)
        scanlink_message = ScanlinkControl()
        scanlink_message.speed = int(self.speed * self.speed_ratio)
        scanlink_message.steer = int(self.steer * self.steer_ratio)
        self.navic_publisher.publish(scanlink_message)


if __name__ == '__main__':
    rospy.init_node('BAE_waypoint', anonymous=True, log_level=rospy.DEBUG)
    robot = RobotController() 
    rospy.spin()

