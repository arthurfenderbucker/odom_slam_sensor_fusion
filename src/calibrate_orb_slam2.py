#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Pose


import json

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import rospkg
import ros_numpy


import rospkg
import tf

rospack = rospkg.RosPack()
positions_path = str(rospack.get_path('odom_slam_sensor_fusion')+'/config/maps/recorded_maps.json')


try:
    with open(positions_path, 'r') as json_data_file:
        calibration_data = json.load(json_data_file)
except:
    calibration_data = {"final_poses":[],"slam_poses":[]}

map_name = "default"

decision = 0
slam_pose = []
count_callbacks = 0
def slam_pose_callback(pose):
    global slam_pose, count_callbacks
    slam_pose = ros_numpy.numpify(pose.pose)
    count_callbacks += 1
    if count_callbacks > 30 and decision == 0:
        count_callbacks =0
        print (".")
        if map_name in calibration_data.keys() and len(calibration_data[map_name])>2:
            
            new_pose = np.dot(np.array(calibration_data[map_name]["pose_correction_matrix"]),slam_pose)
            new_pose = new_pose*calibration_data[map_name]["scale_factor_matrix"]
            # print(ros_numpy.msgify(Pose,new_pose))
        
            

rospy.init_node('Vel_Control_Node', anonymous=True)
rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, slam_pose_callback)

config_file = rospy.get_param('~config_file',"default.json")


def calculate_map_tf():
    global map_name
    global calibration_data

    x = np.array(calibration_data[map_name]["slam_poses"])
    y = np.array(calibration_data[map_name]["final_poses"])
    #calculate scale factor
    total_dist_slam = 0
    total_dist_final = 0
    c = 0
    for i in range(len(x)):
        for j in range(i+1,len(x)):
            print(i,j)
            
            total_dist_slam += np.sqrt(np.dot(x[i,:3,3]-x[j,:3,3],x[i,:3,3]-x[j,:3,3]))
            total_dist_final += np.sqrt(np.dot(y[i,:3,3]-y[j,:3,3],y[i,:3,3]-y[j,:3,3]))
            c +=1
    print(total_dist_final/c, total_dist_slam/c)
    scale_factor = total_dist_final/total_dist_slam
    print("scale_factor: ",str(scale_factor))
    
    #scale factor matrix multiplies only the cartesian coords of the pose

    sf_i = np.eye(4)
    sf_i *= scale_factor
    sf_i[3,3] = 1

    sfM = np.ones([4,4])
    sfM[:3,3] = scale_factor


    #yeah... i know that this is not the best solution, but it is working, and for now it is all that matters
    # tfM = np.dot(np.dot(np.linalg.inv(y[0]*sf_m),x[0]),sf_i)

    tfM = np.dot(np.dot(np.linalg.inv(np.dot(sf_i,x[0])),y[0]),sf_i)

    return tfM, sfM


while decision != 4 :
    print("Selected map: "+map_name)
    if map_name in calibration_data.keys() and len(calibration_data[map_name])>2:
        print("map already calibrated")
        
    else:
        print("map is not calibrated! Calibration data:")
    if map_name in calibration_data.keys():
        print(calibration_data[map_name]) 
        print("please record at least "+str(2-len(calibration_data[map_name]["final_poses"]))+" more position")

    print (' 1: record this position as the new global position:\n 2: change map\n 2: reset calibration\n 4: exit')
    print("\n dots \".\" means that the Slam is publishing some pose")

    decision = 0
    decision = int(input())

    if decision == 1:
        if len(slam_pose)>0:
            coord = input("new coord (x,y,z) or (x,y,z,R,P,Y):")
            coord = [ float(i) for i in coord.split(",")]
            print(coord)
            if len(coord)==3:
                coord += [0,0,0]
                print("compliting rotation with 0s")

            if not len(coord)==6:
                print("error, make sure you are using comma separeted values")
            else:
                given_pose = Pose()
                quarterion = tf.transformations.quaternion_from_euler(coord[3], coord[4], coord[5])
                # print(quaternion)
                given_pose.orientation.x,given_pose.orientation.y,given_pose.orientation.z,given_pose.orientation.w = quarterion
                given_pose.position.x,given_pose.position.y,given_pose.position.z = coord[:3]
                
                if map_name in calibration_data.keys():

                    calibration_data[map_name]["slam_poses"] = calibration_data[map_name]["slam_poses"]+[slam_pose.tolist()]
                    calibration_data[map_name]["final_poses"] = calibration_data[map_name]["final_poses"]+[ros_numpy.numpify(given_pose).tolist()]
                    if len(calibration_data[map_name]["final_poses"])<2:
                        print("please record at least"+str(2-len(calibration_data[map_name]["final_poses"]))+"more position")
                    else:
                        print("calibrated")
                        tfM, sfM = calculate_map_tf()
                        calibration_data[map_name]["pose_correction_matrix"] = tfM.tolist()
                        calibration_data[map_name]["scale_factor_matrix"] = sfM.tolist()
                else:
                    calibration_data[map_name]= {"slam_poses":[slam_pose.tolist()],"final_poses":[ros_numpy.numpify(given_pose).tolist()]}
                    print("please record at least"+str(2-len(calibration_data[map_name]["final_poses"]))+"more position")
                print(calibration_data)
                with open(positions_path, 'w') as json_data_file:
                    json.dump(calibration_data, json_data_file)
                print("position saved")
        else:
            print("no positions have been received yet!\nplease check if the node \"orb_slam2_ros\" is running")
    if decision == 2:
        print("existing maps:")
        print(calibration_data.keys())
        map_name = str(input("please write the name of the map edit or create: "))
        if map_name in calibration_data.keys():
            print("existing map selected!")    
            print(calibration_data[map_name])
        else:
            print("new map selected!")
    if decision == 3: #delete map
        if str(input("are you sure? (y/N):")) == "y":
            calibration_data.pop(map_name,None)
            map_name = "default"
            print("map deleted")
            with open(positions_path, 'w') as json_data_file:
                        json.dump(calibration_data, json_data_file)
    print("\n\n\n")
print("bye bye")