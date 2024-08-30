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

                # Set up tf listeners
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Set up publishers
        self.navic_publisher = rospy.Publisher("/scanlink_control", ScanlinkControl, queue_size=1)
        # Set up subscriber definition & rate
        self.timer_navic_update = rospy.Timer(rospy.Duration(1.0/25.0), self.update_speed_callback)
        # Set up tf publishers for waypoints
        self.probe_wp_broadcaster = tf2_ros.TransformBroadcaster()
        self.probe_target_transformStamped = geometry_msgs.msg.TransformStamped()

        # Create action client
        self.act_client = actionlib.SimpleActionClient('scan', scanAction)
        self.act_client.wait_for_server()
        self.scan_start = Pose()

        # Set up subscribers
        self.vive_pose_plate = rospy.Subscriber("/vive/LHR_6727695C_pose_filt", PoseWithCovarianceStamped , self.vive_plate_callback, queue_size=1)        
        self.vive_pose_navic = rospy.Subscriber("/vive/LHR_EA821C01_pose_filt", PoseWithCovarianceStamped , self.waypoint_navic_callback, queue_size=1)



    def vive_plate_callback(self, input):
        self.plate_pose = input
        
        if self.waypoints_generated == 0:
            self.generate_waypoints()

        if self.waypoints_generated == 1:
            ################################################################## Publish the current probe target frame
            self.probe_target_transformStamped.header.stamp = rospy.Time.now()
            self.probe_target_transformStamped.header.frame_id = "LHR_6727695C_pose_filt"
            self.probe_target_transformStamped.child_frame_id = "probe_target"

            probe_wp_q = transforms3d.euler.euler2quat(
                self.probe_wp[3]*math.pi/180, 
                self.probe_wp[4]*math.pi/180, 
                self.probe_wp[5]*math.pi/180
                )

            self.probe_target_transformStamped.transform.translation.x = float(self.probe_wp[0]/1000)
            self.probe_target_transformStamped.transform.translation.y = float(self.probe_wp[1]/1000)
            self.probe_target_transformStamped.transform.translation.z = float(self.probe_wp[2]/1000)

            self.probe_target_transformStamped.transform.rotation.w = probe_wp_q[0]
            self.probe_target_transformStamped.transform.rotation.x = probe_wp_q[1]
            self.probe_target_transformStamped.transform.rotation.y = probe_wp_q[2]
            self.probe_target_transformStamped.transform.rotation.z = probe_wp_q[3]

    def waypoint_navic_callback(self, input):
        self.read_navic_position()

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

            self.start_scan()

    def start_scan(self):
        # Wait for user to start scan
        input("Press Enter to start scan...")
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
        goal.scan_length = 120
        self.act_client.send_goal(goal)
        self.act_client.wait_for_result()
        result = self.act_client.get_result()
        result_status = self.act_client.get_goal_status_text()
        rospy.logdebug(f"Scan finished with Scan Complete: {result.scan_complete}\nMsg: {result_status}.")
        rospy.sleep(3)
                
    def generate_waypoints(self):
        self.probe_wp = [80
        , -145, 0, 0, 0, 90]

        self.waypoints_generated = 1
        
    def update_speed_callback(self, timer_crawler):
        if self.waypoints_generated == 1:
            self.probe_wp_broadcaster.sendTransform(self.probe_target_transformStamped)
        scanlink_message = ScanlinkControl()
        scanlink_message.speed = int(self.speed * self.speed_ratio)
        scanlink_message.steer = int(self.steer * self.steer_ratio)
        self.navic_publisher.publish(scanlink_message)


if __name__ == '__main__':
    rospy.init_node('BAE_waypoint', anonymous=True, log_level=rospy.DEBUG)
    robot = RobotController() 
    rospy.spin()


