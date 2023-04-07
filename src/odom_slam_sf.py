#!/usr/bin/env python
import rospy
import numpy as np
import math


from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
import ros_numpy


from drone_control.cfg import ControlConfig
import rospkg
import json

import time
import tf


rospack = rospkg.RosPack()
class odom_slam_sf(object): # visual odometry drone controler

    current_coord = np.array([0.0,0.0,0.0])

    current_pose = Pose()
    angle_pose = 0.0
    odom_to_slam_tf = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]]) # initially no transformation
    slam_coord = None
    odom_pose_correction = np.eye(4)
    odom_pose_raw_np = np.eye(4)
    odom_pose = None
    last_slam_time = 0

    def __init__(self):

        #setup node
        rospy.init_node('Odom_slam_sf', anonymous=True)
        self.rate = rospy.Rate(60)  # refresh rate (Hz)

        self.odom_topic = rospy.get_param('~odom_topic','/bebop/odom/')
        self.slam_pose_topic = rospy.get_param('~slam_pose_topic','/orb_slam2_mono/pose')
        config_path = rospy.get_param('~config_path',str(rospack.get_path('odom_slam_sensor_fusion')+'/config/maps/recorded_maps.json'))
        map_name = rospy.get_param('~map_name',"default")


        try:
            with open(config_path, 'r') as json_data_file:
                calibration_file_data = json.load(json_data_file)
        except:
            raise Exception("error openning the config file: "+config_path)

        if map_name in calibration_file_data.keys():
            calibration_data = calibration_file_data[map_name]

            self.slam_pose_correction = np.array(calibration_data["pose_correction_matrix"])
            self.scale_factor_matrix = np.array(calibration_data["scale_factor_matrix"])
            rospy.loginfo("calibration loaded")
        else:
            rospy.logerr("error loading \""+map_name+"\" calibration data")
            raise Exception( "please run calibrate_slam.py node")


        rospy.Subscriber( self.slam_pose_topic , PoseStamped, self.slam_callback)

        # rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged', Ardrone3PilotingStateAltitudeChanged, self.altitude_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback)

        self.current_odom_pub = rospy.Publisher(
            "odom_slam_sf/current_odom", Odometry, queue_size=1)
        
        self.current_coord_pub = rospy.Publisher(
            "odom_slam_sf/current_coord", Point, queue_size=1)

        self.current_pose_pub = rospy.Publisher(
            "odom_slam_sf/current_pose", Pose, queue_size=1)
        self.odom_pose_pub = rospy.Publisher(
            "odom_slam_sf/odom_pose", Pose, queue_size=1)
        rospy.loginfo("setup ok")

    # ------------ topics callbacks -----------
    def odometry_callback(self, odom):


        self.odom_pose = odom.pose.pose
        self.odom_pose_raw_np = ros_numpy.numpify(odom.pose.pose)

        self.current_pose_np = np.dot(self.odom_pose_raw_np, self.odom_pose_correction)
        self.current_pose = ros_numpy.msgify(Pose, self.current_pose_np)
        self.odom_pose_pub.publish(self.current_pose)


    def slam_callback(self,pose):
        self.slam_pose_raw = ros_numpy.numpify(pose.pose) #homogeneous transformation matrix from the origin

        self.slam_pose_np = np.dot(self.slam_pose_correction, self.slam_pose_raw)
        self.slam_pose_np = self.slam_pose_np*self.scale_factor_matrix

        self.slam_pose = ros_numpy.msgify(Pose,self.slam_pose_np)

        #bebop camera link correction and camera calibration correction
        ang_z = self.euler_from_pose(self.slam_pose)[2]
        self.slam_pose.position.x += -0.11*np.cos(ang_z)
        self.slam_pose.position.y += -0.11*np.sin(ang_z)

        self.slam_pose_np = ros_numpy.numpify(self.slam_pose)

        self.last_slam_time=time.time()
        if not self.odom_pose == None:
            #calculates the transfor matrix for the odom position to the modified slam coords system (assumed as true value)
            self.odom_pose_correction = np.dot(np.linalg.inv(self.odom_pose_raw_np),self.slam_pose_np)
        else:
            rospy.loginfo("Havent received any odom coord yet!")
        self.current_pose = self.slam_pose
        self.current_pose_np = self.slam_pose_np


    # ----------------------Sensor Fusion functions--------------------------
    def euler_from_pose(self, pose):
        quarterion = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        return tf.transformations.euler_from_quaternion(quarterion)


    def run(self):
        while not rospy.is_shutdown():
            self.current_odom_pub.publish(self.current_odom)
            self.current_pose_pub.publish(self.current_pose)
            if time.time()-self.last_slam_time > 0.5: print("NO SLAM!!!")
            else: print("SLAM")
            self.rate.sleep()


if __name__ == "__main__":
    c = odom_slam_sf()
    c.run()
