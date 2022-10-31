#! /usr/bin/env python


import rospy
from std_msgs.msg import String
import osqp
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
import math
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def doFem(msg_):
    # rospy.loginfo("I heard:%s",msg_)

    vecx = []
    vecy = []

    for pathmsg in msg_.poses:
        vecx.append(pathmsg.pose.position.x)
        vecy.append(pathmsg.pose.position.y)

    # print(vecx,"\n",vecy)
    referenceline(vecx,vecy)

#clculation kappa
def calcKappa(x_array,y_array):
    s_array = []
    k_array = []
    if(len(x_array) != len(y_array)):
        return(s_array , k_array)
 
    length = len(x_array)
    temp_s = 0.0
    s_array.append(temp_s)
    for i in range(1 , length):
        temp_s += np.sqrt(np.square(y_array[i] - y_array[i - 1]) + np.square(x_array[i] - x_array[i - 1]))
        s_array.append(temp_s)
 
    xds,yds,xdds,ydds = [],[],[],[]
    for i in range(length):
        if i == 0:
            xds.append((x_array[i + 1] - x_array[i]) / (s_array[i + 1] - s_array[i]))
            yds.append((y_array[i + 1] - y_array[i]) / (s_array[i + 1] - s_array[i]))
        elif i == length - 1:
            xds.append((x_array[i] - x_array[i-1]) / (s_array[i] - s_array[i-1]))
            yds.append((y_array[i] - y_array[i-1]) / (s_array[i] - s_array[i-1]))
        else:
            xds.append((x_array[i+1] - x_array[i-1]) / (s_array[i+1] - s_array[i-1]))
            yds.append((y_array[i+1] - y_array[i-1]) / (s_array[i+1] - s_array[i-1]))
    for i in range(length):
        if i == 0:
            xdds.append((xds[i + 1] - xds[i]) / (s_array[i + 1] - s_array[i]))
            ydds.append((yds[i + 1] - yds[i]) / (s_array[i + 1] - s_array[i]))
        elif i == length - 1:
            xdds.append((xds[i] - xds[i-1]) / (s_array[i] - s_array[i-1]))
            ydds.append((yds[i] - yds[i-1]) / (s_array[i] - s_array[i-1]))
        else:
            xdds.append((xds[i+1] - xds[i-1]) / (s_array[i+1] - s_array[i-1]))
            ydds.append((yds[i+1] - yds[i-1]) / (s_array[i+1] - s_array[i-1]))
    for i in range(length):
        k_array.append((xds[i] * ydds[i] - yds[i] * xdds[i]) / (np.sqrt(xds[i] * xds[i] + yds[i] * yds[i]) * (xds[i] * xds[i] + yds[i] * yds[i]) + 1e-6));
    return(s_array,k_array)
 
def referenceline(x_array_,y_array_):
 
    #add some data for test
    x_array = x_array_
    y_array = y_array_
    length = len(x_array)
 
    #weight , from config
    weight_fem_pos_deviation_ = 1e10 #cost1 - x
    weight_path_length = 1          #cost2 - y
    weight_ref_deviation = 1        #cost3 - z
 
 
    P = np.zeros((length,length))
    #set P matrix,from calculateKernel
    #add cost1
    P[0,0] = 1 * weight_fem_pos_deviation_
    P[0,1] = -2 * weight_fem_pos_deviation_
    P[1,1] = 5 * weight_fem_pos_deviation_
    P[length - 1 , length - 1] = 1 * weight_fem_pos_deviation_
    P[length - 2 , length - 1] = -2 * weight_fem_pos_deviation_
    P[length - 2 , length - 2] = 5 * weight_fem_pos_deviation_
 
    for i in range(2 , length - 2):
        P[i , i] = 6 * weight_fem_pos_deviation_
    for i in range(2 , length - 1):
        P[i - 1, i] = -4 * weight_fem_pos_deviation_
    for i in range(2 , length):
        P[i - 2, i] = 1 * weight_fem_pos_deviation_
 
    # with np.printoptions(precision=0):
    #     # print(P)
 
    P = P / weight_fem_pos_deviation_
    P = sparse.csc_matrix(P)
 
    #set q matrix , from calculateOffset
    q = np.zeros(length)
 
    #set Bound(upper/lower bound) matrix , add constraints for x
    #from CalculateAffineConstraint
 
    #In apollo , Bound is from road boundary,
    #Config limit with (0.1,0.5) , Here I set a constant 0.2
    bound = 0.2
    A = np.zeros((length,length))
    for i in range(length):
        A[i, i] = 1
    A = sparse.csc_matrix(A)
    lx = np.array(x_array) - bound
    ux = np.array(x_array) + bound
    ly = np.array(y_array) - bound
    uy = np.array(y_array) + bound
 
    #solve
    prob = osqp.OSQP()
    prob.setup(P,q,A,lx,ux)
    res = prob.solve()
    opt_x = res.x
 
    prob.update(l=ly, u=uy)
    res = prob.solve()
    opt_y = res.x
 
    #plot x - y , opt_x - opt_y , lb - ub
 
    fig1 = plt.figure(dpi = 100 , figsize=(12, 8))
    ax1_1 = fig1.add_subplot(2,1,1)
 
    ax1_1.plot(x_array,y_array , ".--", color = "grey", label="orig x-y")
    ax1_1.plot(opt_x, opt_y,".-",label = "opt x-y")
    # ax1_1.plot(x_array,ly,".--r",label = "bound")
    # ax1_1.plot(x_array,uy,".--r")
    ax1_1.legend()
    ax1_1.grid(axis="y",ls='--')
 
    #plot kappa
    ax1_2 = fig1.add_subplot(2,1,2)
    s_orig,k_orig = calcKappa(x_array,y_array)
    s_opt ,k_opt = calcKappa(opt_x,opt_y)
    ax1_2.plot(s_orig , k_orig , ".--", color = "grey", label="orig s-kappa")
    ax1_2.plot(s_opt,k_opt,".-",label="opt s-kappa")
    ax1_2.legend()
    ax1_2.grid(axis="y",ls='--')
    plt.show()
 
 
if __name__ == "__main__":
    
    rospy.init_node("fem")
    sub = rospy.Subscriber("/run_hybrid_astar/searched_path",Path,doFem)

    rospy.spin()
