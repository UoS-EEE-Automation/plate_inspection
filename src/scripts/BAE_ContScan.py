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
from plate_inspection.msg import scan_approachAction, scan_approachGoal, scan_contAction, scan_contGoal
from plate_inspection.msg import scan_cont_navicAction, scan_cont_navicGoal, scan_cont_navicResult, scan_liftAction, scan_liftGoal

import time
import numpy as np
import math
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from simple_pid import PID


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
        self.navic_wp = PoseArray()
        self.probe_wp = PoseArray()
        self.num_wp = 46

        self.count = 0

        self.stop = 0
        self.target_reached = 0

        self.scan_complete = 0

        self.debug_msg_count = 0

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
        self.current_probe_target = rospy.Publisher("current_probe_target", PoseStamped, queue_size=1)
        self.current_navic_target = rospy.Publisher("current_navic_target", PoseStamped, queue_size=1)

        # Set up publisher for waypoint arrays
        self.probe_wp_publisher = rospy.Publisher('probe_targets', PoseArray, queue_size=1)
        self.navic_wp_publisher = rospy.Publisher('navic_targets', PoseArray, queue_size=1)
        # Set up subscribers
        self.vive_pose_plate = rospy.Subscriber("/vive/LHR_6727695C_pose_filt", PoseWithCovarianceStamped , self.vive_plate_callback, queue_size=1)        
        self.vive_pose_navic = rospy.Subscriber("/vive/LHR_EA821C01_pose_filt", PoseWithCovarianceStamped , self.vive_navic_callback, queue_size=1)

        # Define action server for continuous scanning
        self.act_server_cont_navic = actionlib.SimpleActionServer('cont_scan_navic', scan_cont_navicAction, self.navic_cont_scan, False)
        self.act_server_cont_navic.start()
        # Create all action clients
        # approach action
        self.act_client_app = actionlib.SimpleActionClient('scan_approach', scan_approachAction)
        self.act_client_app.wait_for_server()
        # continuous scan Meca action
        self.act_client_cont = actionlib.SimpleActionClient('cont_scan', scan_contAction)
        self.act_client_cont.wait_for_server()
        # lift action
        self.act_client_lift = actionlib.SimpleActionClient('scan_lift', scan_liftAction)
        self.act_client_lift.wait_for_server()
        # continuous scan navic action
        self.act_client_cont_navic = actionlib.SimpleActionClient('cont_scan_navic', scan_cont_navicAction)
        self.act_client_cont_navic.wait_for_server()

    def vive_plate_callback(self, input):
        self.plate_pose = input

        if self.waypoints_generated == 0:
            self.generate_waypoints(self)

        if self.waypoints_generated == 1 and len(self.navic_wp.poses) != 0 and len(self.probe_wp.poses) != 0:
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

    def vive_navic_callback(self, input):
        self.read_navic_position()        

        if self.target_reached == 1:
            self.scan_with_arm()

        if self.scan_complete == 1:
            self.go_home()
        self.debug_msg_count += 1    
        return

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

        if self.target_reached == 1:
            self.scan_with_arm()
        return

    def calculate_wheel_speeds(self):       
            if len(self.navic_wp.poses) == 0:
                return
            navic_target_x = self.navic_wp.poses[0].position.x*1000
            navic_target_y = self.navic_wp.poses[0].position.y*1000
            navic_target_z = self.navic_wp.poses[0].position.z*1000

            # Calculate Steer
            navic_skew = euler_from_quaternion([self.navic_qx, self.navic_qy, self.navic_qz, self.navic_qw])
            navic_yaw = (navic_skew[2] * 180/math.pi) % 360 - 180
            # Calculate skew based upon point 200mm in front of crawler:
            target_skew = math.atan2(navic_target_y - self.navic_y, - 200) * 180/math.pi  % 360 - 180
            theta_err = (target_skew - navic_yaw) 

            if self.navic_x > navic_target_x and self.stop == 0:
                self.speed = 0.5
                if abs(self.navic_x - navic_target_x) <= 100 and abs(self.navic_x - navic_target_x) > 10:
                    self.speed = abs(self.navic_x - navic_target_x)/100 * self.speed
                elif abs(self.navic_x - navic_target_x) < 10:
                    self.speed = 0.05

                self.steer = theta_err/-15
                if abs(self.steer) > 0.8:
                    if self.steer < 0:
                        self.steer = -0.8
                    elif self.steer > 0:
                        self.steer = 0.8
                if self.debug_msg_count%50 == 0:
                    rospy.logdebug("Moving forward...   Speed: " + str(int(self.speed*self.speed_ratio)) + "   Steer: " + str(int(self.steer*self.steer_ratio)))

            elif self.navic_x <= navic_target_x and self.stop == 0:
                self.speed = 0
                self.steer = 0
                self.stop = 1
                self.target_reached = 1
                rospy.logdebug("Stopping...")

                return
               
    def scan_with_arm(self):
        if self.scan_complete == 1:
            return

        self.target_reached = 0
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
        target_eul = np.array(target_eul)*-180/math.pi
        
        probe_target_pose = [
            self.trans_probe_2.transform.translation.x*1000-20, # Convert from m to mm and apply meca offset
            self.trans_probe_2.transform.translation.y*1000,
            self.trans_probe_2.transform.translation.z*1000,
            target_eul[0], # Convert from rad to deg
            target_eul[1],
            target_eul[2]
            ]
        ################################### Action Sequence ###################################
        rospy.logdebug("Waypoint reached... Approaching sample....")
        goal = scan_approachGoal()
        goal.scan_start = probe_target_pose
        self.act_client_app.send_goal(goal)
        self.act_client_app.wait_for_result()

        ###### Send goal to Meca: robot_controller.py
        rospy.logdebug("Starting continuous scanning....")
        goal = scan_contGoal()
        goal.probe_targets = self.probe_wp
        self.act_client_cont.send_goal(goal)
        ###### Send goal to Navic 
        goal = scan_cont_navicGoal()
        goal.navic_targets = self.navic_wp
        self.act_client_cont_navic.send_goal(goal)
        rospy.logdebug("Goal sent...")
        # Wait for both Meca and Navic to complete actions
        self.act_client_cont_navic.wait_for_result()
        self.act_client_cont.cancel_goal()
        self.act_client_cont.wait_for_result()

        ###### Send lift action to Meca 
        rospy.logdebug("Scan complete... raising probe...")
        goal = scan_liftGoal()
        goal.scan_lift_vel = 10
        self.act_client_lift.send_goal(goal)
        self.act_client_lift.wait_for_result()
        #######################################################################################
        rospy.sleep(3)
        self.scan_complete = 1
        self.stop = 0

    def navic_cont_scan(self, goal: scan_cont_navicGoal):
        rospy.logdebug("navic_cont_scan")
        # navic_targets = goal.navic_targets
        navic_targets = self.navic_wp
        probe_targets = self.probe_wp
        # y_setpoint = 0
        # pid_y = PID(0.02, 0, 0, setpoint = y_setpoint, sample_time=0.05)
        navic_target_xyz = PoseStamped()
        probe_target_xyz = PoseStamped()

        i = 0
        while True:
            if self.debug_msg_count%50 == 0:
                rospy.logdebug("Following targets...")
            if (len(navic_targets.poses) != 0):
                ####################### Create variables for probe pose and target pose #######################
                trans = self.tfBuffer.lookup_transform("LHR_6727695C_pose_filt", "navic_control_link", rospy.Time())
                navic_xyz = [
                    trans.transform.translation.x*1000,
                    trans.transform.translation.y*1000,
                    trans.transform.translation.z*1000,
                    trans.transform.rotation.w,
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z
                    ]
                navic_target_xyz.header = navic_targets.header
                navic_target_xyz.pose = navic_targets.poses[0]

                probe_target_xyz.header = probe_targets.header
                probe_target_xyz.pose = probe_targets.poses[0]
                
                # Read current Navic Position
                self.read_navic_position()
                # Calculate Steer
                navic_skew = euler_from_quaternion([self.navic_qx, self.navic_qy, self.navic_qz, self.navic_qw])
                navic_yaw = (navic_skew[2] * 180/math.pi) % 360 - 180
                # Calculate skew based upon point 200mm in front of crawler:
                target_skew = math.atan2(navic_target_xyz.pose.position.y*1000 - self.navic_y, - 200) * 180/math.pi  % 360 - 180
                theta_err = (target_skew - navic_yaw) 

            
                steer = theta_err/-15
                if abs(steer) > 0.8:
                    if steer < 0:
                        steer = -0.8
                    elif steer > 0:
                        steer = 0.8


                # navic_y_error = navic_xyz[1] - current_target_xyz[1]
                # control_y = pid_y(navic_y_error)
                                
                # steer = control_y
                # if steer > 0.20:
                #     steer = 0.20
                # elif steer < -0.20:
                #     steer = -0.20

                speed = 0.1
                if self.debug_msg_count%50 == 0:
                    rospy.logdebug("Speed: {}; Steer: {}".format(int(speed * self.speed_ratio), int(steer * self.steer_ratio)))

                if navic_xyz[0] - navic_target_xyz.pose.position.x*1000 < 10:
                    navic_targets.poses = np.delete(navic_targets.poses, (0), axis=0)
                    probe_targets.poses = np.delete(probe_targets.poses, (0), axis=0)

            else:
                if i == 10:
                    rospy.logdebug("Scan Completed.")
                    result = scan_cont_navicResult()
                    result.scan_complete = 1
                    result.updates_sent = 0
                    self.act_server_cont_navic.set_succeeded(result)
                    break
                else:
                    self.steer = 0
                    self.speed = 0
                    i += 1

            if self.debug_msg_count%50 == 0:
                rospy.logdebug("Number of waypoints remaining: {}".format(len(navic_targets.poses)))
            self.debug_msg_count += 1
            ################################## Publish Navic commands ##################################
            scanlink_message = ScanlinkControl()
            scanlink_message.speed = int(speed * self.speed_ratio)
            scanlink_message.steer = int(steer * self.steer_ratio)
            self.navic_publisher.publish(scanlink_message)

            ################################## Publish current targets ##################################
            self.current_probe_target.publish(probe_target_xyz)
            self.current_navic_target.publish(navic_target_xyz)

            rospy.sleep(0.01)

    def go_home(self):
        if self.navic_x < 1100:
            rospy.logdebug("Returning to home...")
            self.speed = -0.8
            self.steer = 0
        elif self.navic_x >= 1100:
            self.speed = 0
            self.steer = 0
            rospy.logdebug("Inspection Complete...")
                        
    def generate_waypoints(self, *input):
        self.probe_wp.header.frame_id = "LHR_6727695C_pose_filt"
        self.probe_wp.header.stamp = rospy.Time.now()
        self.navic_wp.header.frame_id = "LHR_6727695C_pose_filt"
        self.navic_wp.header.stamp = rospy.Time.now()
        scan_length = 900
        roll_corr_x = np.linspace(0,0.031,self.num_wp)
        roll_corr_y = np.linspace(0,0.031,self.num_wp)
        roll_corr_z = np.linspace(0.707,0.706,self.num_wp)
        roll_corr_w = np.linspace(0.707,0.706,self.num_wp)
        for i in range(self.num_wp):
            probe_wp_fill = Pose()
            probe_wp_fill.position.x = (scan_length-((scan_length/(self.num_wp-1))*i))/1000
            probe_wp_fill.position.y = -180/1000
            probe_wp_fill.position.z = 0
            # probe_wp_fill.orientation.x = 0
            # probe_wp_fill.orientation.y = 0
            # probe_wp_fill.orientation.z = 0.707
            # probe_wp_fill.orientation.w = 0.707 # Eul: 0, 0, 90

            probe_wp_fill.orientation.x = roll_corr_x[i]
            probe_wp_fill.orientation.y = roll_corr_y[i]
            probe_wp_fill.orientation.z = roll_corr_z[i]
            probe_wp_fill.orientation.w = roll_corr_w[i]

            self.probe_wp.poses.append(probe_wp_fill)
            
            navic_wp_fill = Pose()
            navic_wp_fill.position.x = (scan_length+20+53.19-((scan_length/(self.num_wp-1))*i))/1000
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

