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

# waiting for netft
# def wait_for_ft_calib():
#     from roshelper import ROS_Wait_For_Msg
#     ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

if __name__ == "__main__":

    rospy.init_node('controller_dsg', log_level = rospy.INFO)

def initialize_the_motor():
    c('MO') #turn off all motors
    c('SH ABC')    # Servo Here: servo A
    # c('SHB')    # Servo Here: servo A
def set_the_speed(angle = 0.707):
    read_vec = [np.cos(angle), np.sin(angle), 0]
    vel = 40000
    # print(str(int(vel*read_vec[0])), str(int(vel*read_vec[1])),str(int(vel*read_vec[2])))
    print('setting speed to '+str(int(vel*read_vec[0]))+' ,'+str(int(vel*read_vec[1]))+' ,' +str(int(vel*read_vec[2]))  )
    c('SPA='+str(int(vel*read_vec[0]))) #speead, 1000 cts/sec
    c('SPB='+str(int(vel*read_vec[1]))) #speead, 1000 cts/sec
    c('SPC='+str(int(vel*read_vec[2]))) #speead, 1000 cts/sec

def mm2count(read_vec_mm = [0.1, 0.2, 0.3]):
    ScaleLinearStage = rospy.get_param('/ScaleLinearStage')
    ScaleRotaryStage = rospy.get_param('/ScaleRotaryStage')
    # print(read_vec_mm)
    # print(type(read_vec_mm))
    return  [int(read_vec_mm[0]/ScaleLinearStage), int(read_vec_mm[1]/ScaleLinearStage),int(read_vec_mm[2]/ScaleRotaryStage)]

def check_motion_complete(end_point = [0,0,0], epsilon = 1200):
    from numpy import linalg as LA
    flag = True
    # epsilon_sqr = epsilon*epsilon;
    while flag:
        rospy.sleep(0.5)
        pos = [int(c('TPA')), int(c('TPB')), int(c('TPC'))]
        # pos[1] = int(c('TPB'))
        # pos[2] = int(c('TPC'))
        dis_error = np.array(pos) - np.array(end_point)
        # dis_error_sqr = dis_error[0]*dis_error[0] + dis_error[1]*dis_error[1] + dis_error[2]*dis_error[2]
        if LA.norm(dis_error) < epsilon:
            flag = False



def move_motor(angle = 0.707):
    # initialize_the_motor()
    c('SH ABC')
    pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')]
    readlength = 65         # 80mm
    read_vec = [np.cos(angle), np.sin(angle), 0]
    start_point = np.array(pos_reader) - np.array(read_vec)*0.5*readlength
    end_point = np.array(pos_reader) + np.array(read_vec)*0.5*readlength
    print('start_point = '+str(start_point))
    print('end_point = '+str(end_point))
    start_point_count = mm2count(start_point)
    end_point_count = mm2count(end_point)
    set_the_speed()
    print('[MOTOR] Speed has been reset')
    print('PAA='+str(start_point_count[0]))
    print('PAB='+str(start_point_count[1]))
    print('PAC='+str(start_point_count[2]))
    # c('PAA=660000')
    # c('PAB=-256190')
    # c('PAC=0')
    c('SH ABC')
    c('PAA='+str(start_point_count[0])) #relative move, 3000 cts
    c('PAB='+str(start_point_count[1])) #relative move, 3000 ctsc
    c('PAC='+str(start_point_count[2])) #relative move, 3000 cts
    # c('PA '+str(start_point_count[0])+' ,'+str(start_point_count[1])+' ,'+str(start_point_count[2])) #relative move, 3000 cts
    c('TW 10000,10000,10000')  # 10s
    c('BG ABC') #begin motion
    print(' Moving to start_point for this line...')
    # g.GMotionComplete('ABC')
    check_motion_complete([start_point_count[0],start_point_count[1],start_point_count[2]])
    # rospy.sleep(5)
    print('sending ST')
    c('ST')
    # print('sent ST')
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
    print('[MOTOR] Speed has been reset')
    c('BG ABC') #begin motion
    # g.GMotionComplete('ABC')
    check_motion_complete([end_point_count[0],end_point_count[1],end_point_count[2]])
    print('sending ST')
    c('ST')
    print('[MOTOR] Motion Complete')
    rospy.sleep(1)
def go_to_center():

    tell_pos()
    pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')]
    end_point = np.array(pos_reader)
    print('end_point = '+str(end_point))
    end_point_count = mm2count(end_point)

    c('PAA= 660000')
    c('PAB= 0')
    # c('PAC=-1') #relative move, 3000 cts
    print('####################################################')
    print('[MOTOR] Moving to center...')
    print('####################################################')

    c('TW 10000,10000,10000')  # 10s
    c('BG AB') #begin motion
    # c('MC AB') #begin motion
    check_motion_complete([end_point_count[0],end_point_count[1],end_point_count[2]])
    # g.GMotionComplete('ABC')
    print('sending ST')
    c('ST')
    print('[MOTOR] Motion Complete')

def tell_pos():
    pos_x = c('TPA')
    pos_y = c('TPB')
    pos_z = c('TPC')
    ScaleLinearStage = rospy.get_param('/ScaleLinearStage')
    ScaleRotaryStage = rospy.get_param('/ScaleRotaryStage')
    return [int(pos_x)*ScaleLinearStage, int(pos_y)*ScaleLinearStage, int(pos_z)*ScaleRotaryStage]


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





################### Launch the F/T sensor ros node ######################
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mcube-daolin/catkin_ws/src/SysConfig/rosnodes.launch"])
launch.start()


####################### Establish the connection to controller through Ethernet #########################
g = gclib.py()
g.GOpen('169.254.93.220 --direct')
print(g.GInfo())
c = g.GCommand      # alias the command callable

go_to_center()
initialize_the_motor()      # initalization
set_the_speed()             # set the motion stage Speed

####################### Calibrate F/T sensor #########################
rospy.sleep(1)
setZero = rospy.ServiceProxy('/zero', Zero)
rospy.sleep(0.5)
print('sleeping for 8s, Please make sure all masses removed before calibration')
rospy.sleep(8)
setZero()
rospy.sleep(3)
print('sleeping for 15s, waiting for mass to be added')
rospy.sleep(15)

# c('MO') #turn off all motors

nrep = 20
surface_id = 1; # parallel
shape_id = 1;   # ball
delta = 30;     # 30um
height = 30     #30um
vel = 30        #30mm/s
rot = 1.0/6*np.pi

# rospy.sleep(30)

angle_step = 1.0*np.pi/nrep
for rep in xrange(nrep):
    print('rep = '+str(rep) )
    angle = rep*angle_step
    expfilename = 'record_surface=%s_shape=%s_delta=%.0f_height=%.0f_vel=%.0f_rot%.0f__angle=%.2f_rep=%.0f.json' % (surface_id, shape_id,delta, height, vel, rot,angle, rep)
    rospy.set_param('save_file_name', expfilename)
    print (expfilename)
    set_the_speed(angle = 0)
    move_motor(angle)
    # rospy.sleep(30)
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
