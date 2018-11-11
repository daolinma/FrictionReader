#!/usr/bin/env python
# license removed for brevity

import gclib
import rospy
import roslaunch
import netft_rdt_driver
# import stage_position_node
import std_msgs.msg
import numpy as np
from netft_rdt_driver.srv import Zero
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray
from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
# import settings
import mylistener
import json
import datetime


if __name__ == "__main__":
    rospy.init_node('controller_dsg', log_level = rospy.INFO)

def initialize_the_motor():
    c('MO') #turn off all motors
    c('SH ABC')    # Servo Here: servo ABC

def set_the_speed(angle = 0.7853981633974483):
    read_vec = [np.cos(angle), np.sin(angle), 0]
    vel = 40000

    print('Setting speed to '+str(int(vel*read_vec[0]))+' ,'+str(int(vel*read_vec[1]))+' ,' +str(20000)  )
    c('SPA='+str(int(vel*read_vec[0])))     #speed, ? cts/sec
    c('SPB='+str(int(vel*read_vec[1])))     #speed, ? cts/sec
    c('SPC='+str(str(20000)))               #speed, ? cts/sec

def mm2count(read_vec_mm = [0.1, 0.2, 0.3]):
    global ScaleLinearStage
    global ScaleRotaryStage
    global ScaleRotaryStageEncoder
    return  [int(read_vec_mm[0]/ScaleLinearStage), int(read_vec_mm[1]/ScaleLinearStage),int(read_vec_mm[2]/ScaleRotaryStage)]

def check_motion_complete(end_point = [0,0,0], epsilon = 1200):
    global ScaleLinearStage
    global ScaleRotaryStage
    global ScaleRotaryStageEncoder

    from numpy import linalg as LA
    flag = True
    while flag:
        pos = [int(c('TPA')), int(c('TPB')), int(c('TPC'))*ScaleRotaryStageEncoder/ScaleRotaryStage]
        dis_error = np.array(pos) - np.array(end_point)
        print(pos)
        print('     dis_error',dis_error)
        print('     dis_error norm',LA.norm(dis_error))
        if LA.norm(dis_error) < epsilon:
            flag = False
        rospy.sleep(0.5)


def move_stages(angle = 0.7853981633974483, rot = 0):
    # initialize_the_motor()
    c('SH ABC')
    pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')]
    readlength = 65         # unit: mm
    read_vec = [np.cos(angle), np.sin(angle), 0]
    start_point = np.array(pos_reader) - np.array(read_vec)*0.5*readlength
    start_point[2] = rot*180/np.pi
    end_point = np.array(pos_reader) + np.array(read_vec)*0.5*readlength
    end_point[2] = rot*180/np.pi
    print('start_point = '+str(start_point))
    print('end_point = '+str(end_point))
    start_point_count = mm2count(start_point)
    end_point_count = mm2count(end_point)
    set_the_speed()
    print('[MOTOR] Speed has been reset')
    print('PAA='+str(start_point_count[0]))
    print('PAB='+str(start_point_count[1]))
    print('PAC='+str(start_point_count[2]))

    c('SH ABC')
    c('PAA='+str(start_point_count[0])) #relative move, 3000 cts
    c('PAB='+str(start_point_count[1])) #relative move, 3000 ctsc
    c('PAC='+str(start_point_count[2])) #relative move, 3000 cts
    # c('PA '+str(start_point_count[0])+' ,'+str(start_point_count[1])+' ,'+str(start_point_count[2])) #relative move, 3000 cts
    # c('TW 10000,10000,10000')  # 10s
    c('BG ABC') #begin motion
    print(' Moving to start_point for this line...')
    check_motion_complete([start_point_count[0],start_point_count[1],start_point_count[2]])
    print('sending ST')
    c('ST')
    print('[MOTOR] Motion Complete')
    rospy.sleep(1)

    ######### Generate Motion of the stage ##########
    rospy.sleep(.5)
    c('PAA='+str(end_point_count[0])) #relative move, 3000 cts
    c('PAB='+str(end_point_count[1])) #relative move, 3000 ctsc
    c('PAC='+str(end_point_count[2])) #relative move, 3000 cts
    set_the_speed(angle)
    print(' Starting move...')
    print(' Moving to end_point for this line...')
    read_data()
    rospy.sleep(.5)
    print('[MOTOR] Speed has been reset')
    c('BG ABC') #begin motion
    # g.GMotionComplete('ABC')
    check_motion_complete([end_point_count[0],end_point_count[1],end_point_count[2]])
    print('sending ST')
    c('ST')
    print('[MOTOR] Motion Complete')
    rospy.sleep(1)
def go_to_center(rot = 0):

    tell_pos()
    pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')]
    end_point = np.array(pos_reader)
    print('end_point = '+str(end_point))
    end_point_count = mm2count(end_point)

    ScaleRotaryStage = rospy.get_param('/ScaleRotaryStage')
    end_point_count[2] = int(rot*180/np.pi/ScaleRotaryStage)
    print(end_point_count[2], rot)
    c('PAA= '+str(end_point_count[0]))
    c('PAB= '+str(end_point_count[1]))
    c('PAC= '+str(end_point_count[2]))

    print('####################################################')
    print('[MOTOR] Moving to center of the sample...',end_point_count)
    print('####################################################')

    c('TW 10000,10000,10000')  # 10s
    c('BG ABC') #begin motion
    check_motion_complete([end_point_count[0],end_point_count[1],end_point_count[2]])
    print('sending ST')
    c('ST')
    print('[MOTOR] Motion Complete')

def scanthesample(angle,rot):
    # initialize_the_motor()
    c('SH ABC')
    pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')]
    readlength = 65         # 80mm
    read_vec = [np.cos(angle), np.sin(angle), 0]
    start_point = np.array(pos_reader) - np.array(read_vec)*0.5*readlength
    start_point[2] = rot*180/np.pi
    end_point = np.array(pos_reader) + np.array(read_vec)*0.5*readlength
    end_point[2] = rot*180/np.pi
    print('start_point = '+str(start_point))
    print('end_point = '+str(end_point))
    start_point_count = mm2count(start_point)
    end_point_count = mm2count(end_point)
    set_the_speed()
    print('[MOTOR] Speed has been reset')
    print('PAA='+str(start_point_count[0]))
    print('PAB='+str(start_point_count[1]))
    print('PAC='+str(start_point_count[2]))

    set_the_speed()
    go_to_pose(start_point_count)
    rospy.sleep(1)

    ######### Generate Motion of the stage ##########
    read_data()
    set_the_speed(angle)
    go_to_pose(end_point_count)
    rospy.sleep(1)

def go_to_pose(end_pose=[0,0,0]):
    tell_pos()
    pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')]
    end_point = np.array(pos_reader)
    print('end_point = '+str(end_point))
    end_point_count = mm2count(end_point)

    ScaleRotaryStage = rospy.get_param('/ScaleRotaryStage')
    end_point_count[2] = int(rot*180/np.pi/ScaleRotaryStage)
    print(end_point_count[2], rot)
    c('PAA= '+str(end_point_count[0]))
    c('PAB= '+str(end_point_count[1]))
    c('PAC= '+str(end_point_count[2]))

    print('####################################################')
    print('[MOTOR] Moving to target pose...',end_point_count)
    print('####################################################')

    c('TW 10000,10000,10000')  # 10s
    c('BG ABC') #begin motion
    check_motion_complete([end_point_count[0],end_point_count[1],end_point_count[2]])
    print('sending ST command')
    c('ST')
    print('[MOTOR] Motion Complete')

def tell_pos():
    pos = c('TP')
    pos_i = pos.split(',')
    # pos_x = c('TPA')
    # pos_y = c('TPB')
    # pos_z = c('TPC')
    ScaleLinearStage = rospy.get_param('/ScaleLinearStage')
    ScaleRotaryStage = rospy.get_param('/ScaleRotaryStage')
    return [int(pos_i[0])*ScaleLinearStage, int(pos_i[1])*ScaleLinearStage, int(pos_i[2])*ScaleRotaryStage]

start_read_data = rospy.ServiceProxy('start_read', Empty)
def read_data():
    print 'calling start_read service to initialize datalist'
    start_read_data()
    print("data_list initialized")

save_read_data = rospy.ServiceProxy('save_readings', Empty)
def save_data():
    print 'calling save_reading service'
    save_read_data()
    print("file saved")

rospy.sleep(0.5)
setZero = rospy.ServiceProxy('/zero', Zero)
# rospy.sleep(3)
# setZero()
# rospy.sleep(30)

####################### Establish the connection to controller through Ethernet #########################
g = gclib.py()
g.GOpen('169.254.93.220 --direct')
print(g.GInfo())
c = g.GCommand      # alias the command callable

ScaleLinearStage = rospy.get_param('/ScaleLinearStage')
ScaleRotaryStage = rospy.get_param('/ScaleRotaryStage')
ScaleRotaryStageEncoder = rospy.get_param('/ScaleRotaryStageEncoder')

rot = 0.0/6*np.pi

initialize_the_motor()      # initalization
set_the_speed()             # set the motion stage Speed
go_to_center(rot)

####################### Calibrate F/T sensor #########################
rospy.sleep(1)
rospy.sleep(0.5)
print('sleeping for 8s, Please make sure all masses removed before calibration')
rospy.sleep(3) #changed from 8 for testing
setZero()
rospy.sleep(3)
print('sleeping for 15s, waiting for mass to be added')
rospy.sleep(3) #changed from 15 for testing

#defining file_num


# c('MO') #turn off all motors
#variables to be saved
nrep = 2 #used to be 36 changed to 4 for debugging purposes
nrep_rot = 1
surface_id = 1 # parallel
shape_id = 1   # ball
delta = 30     # 30um (w)
height = 30     #30um (d)
vel = 30        #30mm/s
mat = '61D'     #material

angle_step = 2.0*np.pi/nrep
# rospy.sleep(30)
# rot_list = [0, 0.5*np.pi, 0, 0.5*np.pi]
# angle_list = [0, 0.5*np.pi, 0.5*np.pi, 0]

# for rep in range(4):
#     print('rep = '+str(rep))
#     angle = angle_list[rep]
#     rot = rot_list[rep]
#     expfilename = 'record_surface=%s_shape=%s_delta=%.0f_height=%.0f_vel=%.0f_rot=%.2f__angle=%.2f_rep=%.0f.json' % (surface_id, shape_id,delta, height, vel, rot,angle, rep)
#     rospy.set_param('save_file_name', expfilename)
#     print (expfilename)
#     set_the_speed(angle = angle)
#     move_motor(angle,rot)
#     # rospy.sleep(30)
#     save_data()
#     print('saved')
#     rospy.sleep(0.5)
#     if rep == nrep -1:
#         rospy.sleep(1)   # make sure record is terminated completely
#     # tell current position
#     print(tell_pos())




for rot_rep in xrange(nrep_rot):
    rot = (rot_rep)*1.0/6*np.pi
    for rep in xrange(nrep):
        print('rep = '+str(rep))
        angle = rep*angle_step
        now = str(datetime.datetime.now()) #current time
        inst_date = now.split("-")
        year = inst_date[0] #find the year
        month = inst_date[1] #find the month
        day1 = inst_date[2] #pull out the day from the time
        dayt = day1.split(" ")
        day = dayt[0] #find the day
        print("This is the year from the filename:" + " " + year)
        print("This is the month from the filename:" + " " + month)
        print("This is the day from the filename:" + " " + day)
        expfilename = 'record_surface=%s_shape=%s_delta=%.0f_height=%.0f_vel=%.0f_rot=%.2f__angle=%.2f_rep=%.0f_mat=%s.json' % (surface_id, shape_id,delta, height, vel, rot,angle, rep, mat)
        expfilename = year + "-" + month + "-" + day + "-" + expfilename
        rospy.set_param('save_file_name', expfilename)
        print ("expfilename: " + expfilename)
        set_the_speed(angle = angle)
        scanthesample(angle,rot)
        save_data()
        print('saved')
        rospy.sleep(0.5)
        if rep == nrep -1:
            rospy.sleep(1)   # make sure record is terminated completely
        # tell current position
        print(tell_pos())
    ######### Record the force and torque ##########

############# ?? ################

c('MO') #turn off all motors
del c #delete the alias
