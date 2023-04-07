#!/usr/bin/env python
import rospy
import numpy as np
import math


from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Pose, Quaternion,TransformStamped, Point
import ros_numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

from scipy.spatial.transform import Rotation

from drone_control.cfg import ControlConfig
import rospkg
import json

import tf
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

from odom_slam_sensor_fusion.srv import SetScale

import os

from copy import deepcopy

rospack = rospkg.RosPack()
class odom_slam_sf(object): # visual odometry drone controler

    current_coord = np.array([0.0,0.0,0.0])

    current_odom = Odometry()
    odom = Odometry()
    current_pose = Pose()
    last_pose = Pose()

    slam_scaled_tf = TransformStamped()
    odom_tf = TransformStamped()
    slam_tf = TransformStamped()




    angle_pose = 0.0
    odom_to_slam_tf = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]]) # initially no transformation
    slam_coord = None
    odom_pose_correction = np.eye(4)
    odom_pose_raw_np = np.eye(4)
    odom_pose = None

    def __init__(self):

        #setup node
        rospy.init_node('Slam_sf', anonymous=True)
        self.rate = rospy.Rate(60)  # refresh rate (Hz)

        self.current_odom.pose.pose.orientation.w = 1.0
        self.odom.pose.pose.orientation.w = 1.0

        self.odom_topic = rospy.get_param('~odom_topic','/bebop/odom/')
        self.slam_pose_topic = rospy.get_param('~slam_pose_topic','/orb_slam3/camera_pose')
        # self.slam_pose_topic = rospy.get_param('~slam_pose_topic','/orb_slam2_mono/pose')

        self.config_path = rospy.get_param('~config_path',str(rospack.get_path('odom_slam_sensor_fusion')+'/config/maps/slam_calibration.json'))

        self.map_name = rospy.get_param('~map_name',"demo")

        self.last_slam_time = rospy.Time.now()

        self.br = tf2_ros.TransformBroadcaster()
        self.listener = tf.TransformListener()

        while not rospy.is_shutdown() and not self.load_calibration_file():
            rospy.sleep(1)      
        
        self._cached_stamp = os.stat(self.config_path).st_mtime
        rospy.Subscriber( self.slam_pose_topic , PoseStamped, self.slam_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback)
        # rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback)


        # rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged', Ardrone3PilotingStateAltitudeChanged, self.altitude_callback)

        self.current_odom_pub = rospy.Publisher("odom_slam_sf/current_odom", Odometry, queue_size=1)
        self.current_odom_open_pub = rospy.Publisher("odom_slam_sf/current_odom_open", Odometry, queue_size=1)
        self.current_coord_pub = rospy.Publisher("odom_slam_sf/current_coord", Point, queue_size=1)
        self.current_pose_pub = rospy.Publisher("odom_slam_sf/current_pose", Pose, queue_size=1)
        self.odom_pose_pub = rospy.Publisher("odom_slam_sf/odom_pose", Pose, queue_size=1)


        rospy.loginfo("setup ok")


    def load_calibration_file(self):
        calibration_loaded = False
        try:
            with open(self.config_path, 'r') as json_data_file:
                calibration_file_data = json.load(json_data_file)

        except:
            # raise Exception("error openning the config file: "+self.config_path)
            rospy.logerr("error openning the config file: "+self.config_path)
            return False

        if (self.map_name in calibration_file_data.keys() and
                    "rotation" in calibration_file_data[self.map_name].keys() and
                    "scale_factor" in calibration_file_data[self.map_name].keys() and
                    "origin_slam_pose" in calibration_file_data[self.map_name].keys()):
            
            self.calibration_data = calibration_file_data
            # self.slam_pose_correction = np.array(self.calibration_data["pose_correction_matrix"])
            # self.scale_factor_matrix = np.array(self.calibration_data["scale_factor_matrix"])

            self.slam_rotation = Rotation.from_quat(self.calibration_data[self.map_name]["rotation"])
            self.slam_scale_factor = self.calibration_data[self.map_name]["scale_factor"]
            self.slam_origin_pose = np.array(self.calibration_data[self.map_name]["origin_slam_pose"])[:3,3]

            if "odom_origin_rotation" in self.calibration_data[self.map_name].keys():
                odom_origin_q = np.array(self.calibration_data[self.map_name]["odom_origin_rotation"])
                rospy.loginfo(odom_origin_q)

            else:
                odom_origin_q = np.array([0,0,0,1])
                rospy.logerr("odom_origin_rotation not found in calibration file, using identity rotation")

            # self.slam_origin_rot = Rotation.from_matrix(np.array(self.calibration_data[self.map_name]["origin_slam_pose"])[:3,:3])

            self.odom_camera_rotation = np.array([[0,-1,0],
                                                [1,0,0],
                                                [0,0,1]])
            
            # self.odom_camera_rotation = np.array([[1,0,0],
            #                                     [0,1,0],
            #                                     [0,0,1]])
            
            self.odom_origin_rm = Rotation.from_quat(odom_origin_q).as_matrix()
            slam_origin_q = self.slam_rotation.as_quat()
            # slam_origin_q = Rotation.from_matrix(np.dot(self.slam_rotation.as_matrix(),self.odom_camera_rotation)).as_quat()


            q = self.slam_rotation.as_quat()

            self.slam_tf.header.stamp = rospy.Time.now()

            self.slam_tf.header.frame_id = "world"
            self.slam_tf.child_frame_id = "slam"
            self.slam_tf.transform.translation.x = self.slam_origin_pose[0]
            self.slam_tf.transform.translation.y = self.slam_origin_pose[1]
            self.slam_tf.transform.translation.z = self.slam_origin_pose[2]
            self.slam_tf.transform.rotation = ros_numpy.msgify(Quaternion, slam_origin_q)
            # self.slam_tf.transform.rotation.x = slam_origin_q[0]
            # self.slam_tf.transform.rotation.y = slam_origin_q[1]
            # self.slam_tf.transform.rotation.z = slam_origin_q[2]
            # self.slam_tf.transform.rotation.w = slam_origin_q[3]

            slam_origin_new = self.slam_rotation.apply(self.slam_origin_pose)

            self.slam_scaled_tf.header.stamp = rospy.Time.now()

            self.slam_scaled_tf.header.frame_id = "world"
            self.slam_scaled_tf.child_frame_id = "slam_scaled"
            self.slam_scaled_tf.transform.translation.x = -slam_origin_new[0]*self.slam_scale_factor
            self.slam_scaled_tf.transform.translation.y = -slam_origin_new[1]*self.slam_scale_factor
            self.slam_scaled_tf.transform.translation.z = -slam_origin_new[2]*self.slam_scale_factor
            self.slam_scaled_tf.transform.rotation = ros_numpy.msgify(Quaternion, slam_origin_q)
            # self.slam_scaled_tf.transform.rotation.x = slam_origin_q[0]
            # self.slam_scaled_tf.transform.rotation.y = slam_origin_q[1]
            # self.slam_scaled_tf.transform.rotation.z = slam_origin_q[2]
            # self.slam_scaled_tf.transform.rotation.w = slam_origin_q[3]


            self.odom_tf.header.stamp = rospy.Time.now()
            self.odom_tf.header.frame_id = "odom"
            self.odom_tf.child_frame_id = "world"
            self.odom_tf.transform.translation.x = 0
            self.odom_tf.transform.translation.y = 0
            self.odom_tf.transform.translation.z = 0
            self.odom_tf.transform.rotation = ros_numpy.msgify(Quaternion, odom_origin_q)
            # self.odom_tf.transform.rotation.x = odom_origin_q[0]
            # self.odom_tf.transform.rotation.y = odom_origin_q[1]
            # self.odom_tf.transform.rotation.z = odom_origin_q[2]
            # self.odom_tf.transform.rotation.w = odom_origin_q[3]


            # try:
            #     (trans,rot) = self.listener.lookupTransform('/base_link', '/camera_optical', rospy.Time(0))
            #     print(trans, rot)
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     pass

            
            # self.odom_camera_rotation = Rotation.from_quat(rot).as_matrix()

            # self.slam_scaled_tf = TransformStamped()
            # self.slam_scaled_tf.header.stamp = self.last_slam_time
            # self.slam_scaled_tf.header.frame_id = "odom"
            # self.slam_scaled_tf.child_frame_id = "slam"
            # self.slam_scaled_tf.transform.translation.x = self.slam_origin_pose[0]
            # self.slam_scaled_tf.transform.translation.y = self.slam_origin_pose[1]
            # self.slam_scaled_tf.transform.translation.z = self.slam_origin_pose[2]
            # self.slam_scaled_tf.transform.rotation.x = slam_origin_q[0]
            # self.slam_scaled_tf.transform.rotation.y = slam_origin_q[1]
            # self.slam_scaled_tf.transform.rotation.z = slam_origin_q[2]
            # self.slam_scaled_tf.transform.rotation.w = slam_origin_q[3]

            rospy.loginfo("calibration loaded")
            calibration_loaded = True

            self.update_map_scale()

        else:
            rospy.logerr("error loading \""+self.map_name+"\" calibration data")
            # raise Exception( "callibrate the map first")
        return calibration_loaded
    
    def update_map_scale(self):
        try:
            scale_map_srv = rospy.ServiceProxy('/pointcloud_scaler_all/set_scale', SetScale)
            scale_map_srv(self.slam_scale_factor)
            scale_map_srv = rospy.ServiceProxy('/pointcloud_scaler_tracked/set_scale', SetScale)
            scale_map_srv(self.slam_scale_factor)
            rospy.logout("map scale updated")

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # ------------ topics callbacks -----------
    def odometry_callback(self, odom):

        self.odom_pose = odom.pose.pose
        self.odom = odom
        # odom_pose_raw = ros_numpy.numpify(self.odom_pose) #homogeneous transformation matrix from the origin

        # p = odom_pose_raw[:3,3]

        # new_p = self.slam_rotation.apply((p -self.origin_pose))
        # new_p = new_p + np.array(self.calibration_data[self.map_name]["ref_origin_coord"])[:3]


        # self.odom_pose = odom.pose.pose
        # self.odom_pose_raw_np = ros_numpy.numpify(odom.pose.pose)

        # self.current_pose_np = np.dot(self.odom_pose_raw_np, self.odom_pose_correction)
        # self.current_pose = ros_numpy.msgify(Pose, self.current_pose_np)
        # self.odom_pose_pub.publish(self.current_pose)


    def slam_callback(self,pose):
        slam_pose_raw = ros_numpy.numpify(pose.pose)

        

        p = slam_pose_raw[:3,3]

        new_p = self.slam_rotation.apply((p -self.slam_origin_pose)* self.slam_scale_factor)
        new_p = new_p + np.array(self.calibration_data[self.map_name]["ref_origin_coord"])[:3]

        rm = self.slam_rotation.as_matrix()

        
        q = Rotation.from_matrix(np.dot(self.odom_camera_rotation,np.dot(rm,slam_pose_raw[:3,:3]))).as_quat()

        # pose.pose.position.x = new_p[0]
        # pose.pose.position.y = new_p[1]
        # pose.pose.position.z = new_p[2]
        # pose.pose.orientation.x = q[0]
        # pose.pose.orientation.y = q[1]
        # pose.pose.orientation.z = q[2]
        # pose.pose.orientation.w = q[3]
        ang_z = tf.transformations.euler_from_quaternion(q)[2]
        new_p[0] += -0.11*np.cos(ang_z)
        new_p[1] += -0.11*np.sin(ang_z)

        new_pose = Pose()

        new_pose.position = ros_numpy.msgify(Point, new_p)
        new_pose.orientation = ros_numpy.msgify(Quaternion, q)

        self.last_slam_time = rospy.Time.now()


        # #bebop camera link correction and camera calibration correction

        self.current_odom.pose.pose = new_pose
        self.current_odom.header.stamp = self.last_slam_time
        self.current_odom.header.frame_id = "world"

        self.last_pose = deepcopy(self.current_pose)
        self.current_pose = new_pose




        # self.slam_pose_np = np.dot(self.slam_pose_correction, self.slam_pose_raw)
        # self.slam_pose_np = self.slam_pose_np*self.scale_factor_matrix

        # self.slam_pose = ros_numpy.msgify(Pose,self.slam_pose_np)


        # self.slam_pose_np = ros_numpy.numpify(self.slam_pose)

        # self.last_slam_time=time.time()
        # if not self.odom_pose == None:
        #     #calculates the transfor matrix for the odom position to the modified slam coords system (assumed as true value)
        #     self.odom_pose_correction = np.dot(np.linalg.inv(self.odom_pose_raw_np),self.slam_pose_np)
        # else:
        #     rospy.loginfo("Havent received any odom coord yet!")
        # self.current_pose = self.slam_pose
        # self.current_pose_np = self.slam_pose_np


    # ----------------------Sensor Fusion functions--------------------------
    def euler_from_pose(self, pose):
        quarterion = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        return tf.transformations.euler_from_quaternion(quarterion)

    
    def transform_pose(self, input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def run(self):
        running_open_loop = False
        yaw_open_loop = 0
        coords_open_loop = np.array([0,0,0])
        while not rospy.is_shutdown():

            stamp = os.stat(self.config_path).st_mtime
            if stamp != self._cached_stamp:
                self._cached_stamp = stamp
                rospy.loginfo("Reloading calibration file")
                while not rospy.is_shutdown() and not self.load_calibration_file():
                    rospy.sleep(1)
                    # self.br.sendTransform([self.slam_scaled_tf,self.slam_tf,self.odom_tf])

                rospy.loginfo("DONE reloading calibration file")

            t = rospy.Time.now()

            
            if (t -self.last_slam_time).to_sec() < 0.1 and self.last_pose != self.current_pose:

                self.current_odom_pub.publish(self.current_odom)
                self.current_pose_pub.publish(self.current_pose)

                self.slam_tf.header.stamp = rospy.Time.now()
                self.slam_scaled_tf.header.stamp = rospy.Time.now()
                self.odom_tf.header.stamp = rospy.Time.now()

                running_open_loop = False 

            else:
                rospy.logerr("NO SLAM! running open loop")
                if not running_open_loop:
                    self.current_odom_open = self.current_odom
                    yaw_open_loop = Rotation.from_quat(ros_numpy.numpify(self.current_odom_open.pose.pose.orientation)).as_euler('xyz')[2]

                    coords_open_loop = ros_numpy.numpify(self.current_odom.pose.pose.position)
                    dt = (t - self.last_slam_time).to_sec()
                else:
                    dt = (t - last_t).to_sec()


                # new_pose = self.transform_pose(self.current_odom_open.pose.pose, "odom", "world")
                # if not new_pose is None:
                # ori = Rotation.from_quat(ros_numpy.numpify(self.current_odom_open.pose.pose.orientation)).as_matrix()

                angular_speed=ros_numpy.numpify(self.odom.twist.twist.angular)
                
                if abs(self.odom.twist.twist.angular.z)<100.0:
                    yaw_open_loop += self.odom.twist.twist.angular.z *dt

                    if yaw_open_loop > 2*np.pi:
                        yaw_open_loop= yaw_open_loop-2*np.pi
                    elif yaw_open_loop < 0:
                        yaw_open_loop= 2*np.pi+yaw_open_loop

                vel = ros_numpy.numpify(self.odom.twist.twist.linear)
                delta = np.dot(self.odom_origin_rm,vel)*dt

                coords_open_loop += delta
                self.current_odom_open.pose.pose.position = ros_numpy.msgify(Point,coords_open_loop)
                self.current_odom_open.pose.pose.orientation = ros_numpy.msgify(Quaternion,Rotation.from_euler('z',yaw_open_loop).as_quat())


                self.current_odom_open.header.stamp = rospy.Time.now()


                self.current_odom_open_pub.publish(self.current_odom_open)
                # else:
                #     rospy.logerr("Error computing current_odom_open")

                self.current_odom_pub.publish(self.current_odom_open)
                self.current_pose_pub.publish(self.current_odom_open.pose.pose)

                last_t = t
                running_open_loop = True

            # else:
                # rospy.logerr("NO SLAM")
            
            self.br.sendTransform([self.slam_scaled_tf,self.slam_tf,self.odom_tf])

            self.rate.sleep()


if __name__ == "__main__":
    c = odom_slam_sf()
    c.run()
