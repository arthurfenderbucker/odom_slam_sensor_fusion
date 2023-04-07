#!/usr/bin/env python
import numpy as np
from sklearn.linear_model import LinearRegression 
import time 
from geometry_msgs.msg import Point, Pose
import ros_numpy
import tf

def array_to_pose(l):
    p = Pose()
    p.position.x = l[0]
    p.position.y = l[1]
    p.position.z = l[2]
    p.orientation.x = l[3]
    p.orientation.y = l[4]
    p.orientation.z = l[5]
    p.orientation.w = l[6]
    return p

def get_scale_factor(x,y):
    total_dist_x = 0
    total_dist_y = 0
    for i in range(len(x)):
        for j in range(i+1,len(x)):

            total_dist_x += np.sqrt(np.dot(x[i,:3,3]-x[j,:3,3],x[i,:3,3]-x[j,:3,3]))
            total_dist_y += np.sqrt(np.dot(y[i,:3,3]-y[j,:3,3],y[i,:3,3]-y[j,:3,3]))
    return total_dist_y/total_dist_x

pose1 = array_to_pose([1,0,0,0,0,0,0])
pose2 = array_to_pose([0,0,0,0,0,0,0])
orb_pose1 = array_to_pose([2,1,3,0,0,0,0])
orb_pose2 = array_to_pose([1,1,3,0,0,0,0])

x_coords = np.array([[0,0,0,0,0,0,0],[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0]],dtype=np.float64)
test_coords = np.array([[1,0,0,0,0,0,0],[2,-1,1,0,0,0,0],[0,-1,0,0,0,0,0],[1,1,1,0,0,0,0]],dtype=np.float64)
y_coords = np.array([[0,0,0,0,0,0,0],[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0]],dtype=np.float64)
factor = 2

offset = np.array([0,-2,-1,0,0,0.707,0.707])
y_coords *= factor
y_coords += offset
y_coords = y_coords.tolist()
print(y_coords)

x = []
y = []
test = []
for i,j,k in zip(x_coords, y_coords,test_coords):
    x += [ros_numpy.numpify(array_to_pose(i)).tolist()]
    y += [ros_numpy.numpify(array_to_pose(j)).tolist()]
    test += [ros_numpy.numpify(array_to_pose(k)).tolist()]
x = np.array(x)
y = np.array(y)
test = np.array(test)

sf = get_scale_factor(x,y)
sf_i = np.eye(4)
sf_i *= sf
sf_i[3,3] = 1

sf_m = np.ones([4,4])
sf_m[:3,3] = sf

for i in range(len(x)):
    tfM = np.dot(np.dot(np.linalg.inv(np.dot(sf_i,x[i])),y[i]),sf_i)
    print(tfM)
print("------------")

i = 0
for i in range(len(x)):
    z = x[i,:,3]
    print z , 
    print(np.dot(tfM,z)-offset[:4])
print(" ---- test ---- ")
for i in range(len(x)):
    z = x[i]
    print z 
    print(np.dot(tfM,z))
d = 0
while 1:
    d = list(input())
    if d == 1:
        break
    v = ros_numpy.numpify(array_to_pose(d))
    v2 = np.dot(tfM,v)
    
    print(v2)
    print(ros_numpy.msgify(Pose,v2))
# print()
# print(np.dot(tfM_1,x[:,3]))

"""

c1 = ros_numpy.numpify(pose1)
c2 = ros_numpy.numpify(pose2)

orb_c1 = ros_numpy.numpify(orb_pose1)
orb_c2 = ros_numpy.numpify(orb_pose2)

print(c1)
print(c2)

def get_scale_factor(x,y):
    total_dist_slam = 0
    total_dist_final = 0
    for i in range(len(x)):
        for j in range(i+1,len(x)):
            print(i,j)
            total_dist_slam += np.sqrt(np.dot(x[i,:3,3]-x[j,:3,3],x[i,:3,3]-x[j,:3,3]))
            total_dist_final += np.sqrt(np.dot(y[i,:3,3]-y[j,:3,3],y[i,:3,3]-y[j,:3,3]))
    return total_dist_final/total_dist_slam


dist_c = np.sqrt(np.dot(c1[:3,3]-c2[:3,3],c1[:3,3]-c2[:3,3]))
dist_orb = np.sqrt(np.dot(orb_c1[:3,3]-orb_c2[:3,3],orb_c1[:3,3]-orb_c2[:3,3]))
scale_factor = dist_c/dist_orb

scale_factor_m2 = np.eye(4)
scale_factor_m2 *= scale_factor
scale_factor_m2[3,3] = 1
# scale_factor_m2 *= scale_factor-1
# scale_factor_m2 +=1
scale_factor_m = np.ones([4,4])
scale_factor_m[:3,3] = scale_factor

scale_factor_m3 = np.eye(4)
scale_factor_m3 *= scale_factor

print(scale_factor_m)
# tfM_1 = np.dot(c1,np.linalg.inv(orb_c1))
# tfM_2 = np.dot(c2,np.linalg.inv(orb_c2))
# tfM_1 = np.dot(np.linalg.inv(np.dot(scale_factor_m2,orb_c1)),c1)#,scale_factor_m2)

tfM_1 = np.dot(np.dot(np.linalg.inv(orb_c1*scale_factor_m),c1),scale_factor_m2)
tfM_2 = np.dot(np.linalg.inv(orb_c2*scale_factor_m),c2)

print("---")
print(tfM_1)
print(tfM_2)
# print("---")
# print(np.dot(tfM_1,orb_c1[:,3]))
# print(np.dot(tfM_2,orb_c2[:,3]))
# print("-------------------")




print("===============")
# print(orb_c3)
# print(scale_factor_m2)
# print(np.dot(scale_factor_m2,orb_c3))

# x = np.array([5-1,3-1,1-1])
# y = np.array([5,3,1])
# t = time.time()
# for i in range(1000):
# M = tf(x,y)
# print(time.time()-t)
# x = np.array([2+3,1+3,-3+3,1])
# x2 = np.array([1,2,-3,1])
# x = np.array([5-1,3-1,1-1,1])
# print(np.dot(x,M.T))

# print(np.dot(x2,M.T))
# print((orb_c3)[:,3])
print(np.dot(tfM_1,(orb_c3)[:,3]))
print(np.dot(tfM_2,(orb_c3*scale_factor_m)[:,3]))
# print(tf.transformations.quaternion_from_euler(0,0,0))
"""