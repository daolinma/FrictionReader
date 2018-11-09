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
    c('SH ABC')    # Servo Here: servo A
    # c('SHB')    # Servo Here: servo A
def set_the_speed(angle = 0.7853981633974483):
    read_vec = [np.cos(angle), np.sin(angle), 0] #the  current direction of the vector
    print('This the the read_vec variable: ')
    print(read_vec)
    vel = 40000
    # print(str(int(vel*read_vec[0])), str(int(vel*read_vec[1])),str(int(vel*read_vec[2])))
    print('setting speed to '+str(int(vel*read_vec[0]))+' ,'+str(int(vel*read_vec[1]))+' ,' +str(20000)  ) ####why is the speed the direction times a velocity
    cmd_send = str(int(vel*read_vec[0]))
    print(cmd_send)
    cmd_send_ = 'SPA='+str(int(vel*read_vec[0]))
    print(cmd_send_)
    print(type(cmd_send_))
    c(cmd_send_)
    c('SPA='+str(int(vel*read_vec[0]))) #speed, 1000 cts/sec
    c('SPB='+str(int(vel*read_vec[1]))) #speed, 1000 cts/sec
    c('SPC='+str(str(20000))) #speed, 1000 cts/sec

def mm2count(read_vec_mm = [0.1, 0.2, 0.3]):
    global ScaleLinearStage
    global ScaleRotaryStage
    global ScaleRotaryStageEncoder
    # print(read_vec_mm)
    # print(type(read_vec_mm))
    return  [int(read_vec_mm[0]/ScaleLinearStage), int(read_vec_mm[1]/ScaleLinearStage),int(read_vec_mm[2]/ScaleRotaryStage)]

def check_motion_complete(end_point = [0,0,0], epsilon = 1200):
    global ScaleLinearStage
    global ScaleRotaryStage
    global ScaleRotaryStageEncoder

    from numpy import linalg as LA
    flag = True
    # epsilon_sqr = epsilon*epsilon;
    while flag:

        pos = [int(c('TPA')), int(c('TPB')), int(c('TPC'))*ScaleRotaryStageEncoder/ScaleRotaryStage]

        dis_error = np.array(pos) - np.array(end_point)
        print(pos)
        print('dis_error',dis_error)
        print('dis_error norm',LA.norm(dis_error))
        # dis_error_sqr = dis_error[0]*dis_error[0] + dis_error[1]*dis_error[1] + dis_error[2]*dis_error[2]
        if LA.norm(dis_error) < epsilon:
            flag = False
        rospy.sleep(0.5)

######Questions for check_motion_complete function#####
#does the above code know the motion is complete by checking to see how far away you are from a preset end point?

######Questions for move_motor function############
#where did the value for angle come from? Should I change this?
#are my comments right?
#where is '/pos_reader/x' and pos_reader being set
#when defining start_point why are you subtracting the direction from the positon? is this accounting for the ball radius?

#add an agrument for startpoint and the diameter
#offset the circle
##############add an argument for the new center giving the original center as the default to define the function
    #having problems putting in the original values as a list
#original start position off center defined by sys_param: [ 49.952  -12.9721   0.    ]
def move_motor(angle = 0.7853981633974483, rot = 0, diameter = 20, new_center = [49.952, -12.9721, 0]): #angle is the change in the angle for each pass
    # initialize_the_motor()
    c('SH ABC') #servo here, A B and C, tells the controller to use the current positon as the command position and enable servo control here so in this case all motors can move
    #pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')] #list of the position of the center of the micro-texture?
    readlength = diameter         # unit: mm diameter of the circle (will change with bigger texture) (for single was 65)
    read_vec = [np.cos(angle), np.sin(angle), 0]#gives you the unit direction of the path
    start_point = np.array(new_center) - np.array(read_vec)*0.5*readlength #makes an array of the positions and direction 
    start_point[2] = rot*180/np.pi#convert to degrees
    end_point = np.array(new_center) + np.array(read_vec)*0.5*readlength
    end_point[2] = rot*180/np.pi#convert to degrees
    print('start_point = '+str(start_point))
    print('end_point = '+str(end_point))
    start_point_count = mm2count(start_point)#converts the start point to counts for the encoder
    end_point_count = mm2count(end_point)#converts the end point to counts for the encoder
    set_the_speed()
    print('[MOTOR] Speed has been reset')
    print('PAA='+str(start_point_count[0]))#prints where the absolute position on the start point is for motor A
    print('PAB='+str(start_point_count[1]))
    print('PAC='+str(start_point_count[2]))
    # c('PAA=660000')
    # c('PAB=-256190')
    # c('PAC=0')
  #  c('SH ABC')#why is this here isn't it enought to say it once sinc eyou havne't actually moved the servo
    c('PAA='+str(start_point_count[0])) #relative move, 3000 cts
    c('PAB='+str(start_point_count[1])) #relative move, 3000 ctsc
    c('PAC='+str(start_point_count[2])) #relative move, 3000 cts
    # c('PA '+str(start_point_count[0])+' ,'+str(start_point_count[1])+' ,'+str(start_point_count[2])) #relative move, 3000 cts
    # c('TW 10000,10000,10000')  # 10s
    c('BG ABC') #begin motion
    print(' Moving to start_point for this line...')
    # g.GMotionComplete('ABC')
    check_motion_complete([start_point_count[0],start_point_count[1],start_point_count[2]])
    # rospy.sleep(5)
    print('sending ST')
    c('ST')#stop
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

    ScaleRotaryStage = rospy.get_param('/ScaleRotaryStage')#converts from counts to mm
    end_point_count[2] = int(rot*180/np.pi/ScaleRotaryStage)#degrees
    print(end_point_count[2], rot)
    c('PAA= '+str(end_point_count[0]))
    c('PAB= '+str(end_point_count[1]))
    c('PAC= '+str(end_point_count[2]))

    # c('PAC=-1') #relative move, 3000 cts
    print('####################################################')
    print('[MOTOR] Moving to center...',end_point_count)
    print('####################################################')

    #c('TW 10000,10000,10000')  # 10s, time out for IN position
    c('BG ABC') #begin motion
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
rospy.sleep(2) #changed from 8 for testing
setZero()
rospy.sleep(3)
print('sleeping for 15s, waiting for mass to be added')
rospy.sleep(2) #changed from 15 for testing



# c('MO') #turn off all motors
#variables to be saved
nrep = 36 #used to be 36 changed to 4 for debugging purposes
nrep_rot = 1
surface_id = 1 # parallel
shape_id = 1   # ball
delta = 72     # unit: um (w)
height = 36     #unit: um (d)
vel = 40        #unit: mm/s
# mat = '61D'     #material
mat = 'Task16'     #material
diameter = 20   #diameter of the testing circle
texture_num = 4 #number of microtextures on a sample

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


#I need to change the loop to go 4 times for each microtexture, if single micro-texture then the for loop is a range 2 else range 5. I need to embed another for loop
for rot_rep in xrange(nrep_rot):
    rot = (rot_rep)*1.0/6*np.pi#rotate by a constant value
    theta = 45 #the angle of the first microtexture (degrees)
    pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')]
    print("This is the pos_reader: ")
    print(pos_reader)
    theta_step = 360/texture_num #90 (degrees) for 4 microtextures
    for texture in xrange(texture_num + 1):#loop for each micro-texture
        if texture_num != 1:#defines a new center for each micro-texture if there is more than 1 texture
            # theta += 360/texture_num -45 #define the new theta to plug into the rotation matrix (convert to radians)
            
            theta = theta*np.pi/180 #convert to radians
            print('This is theta: ')
            print(theta)
            c, s = np.cos(theta), np.sin(theta) #define the cosine and sine for the new degree
            print("This is cosine: ")
            print(c)
            print('This is sine: ')
            print(s)
            offset = 15 #the distance of the center of microtextures to the center of the sample
            dir_to_center = np.array([c,s]) #make a unit vector from the center to the new degree
            vec_ncenter = dir_to_center*offset#give the vector a magnitude of the offset
            print('This is vec_ncenter')
            print(vec_ncenter)
            x_cord_ncenter = vec_ncenter[0]
            y_cord_ncenter = vec_ncenter[1]
            new_center = np.array(pos_reader) + np.array([x_cord_ncenter, y_cord_ncenter, 0])#move to the new center by adding specimen center to vector
            print('This is new_center: ')
            print(new_center)
            theta += theta_step #change theta to the theta of the next microtexture
            #R_2D = np.array([[c,-s], [s, c]])#create a 2D rotation matrix
            #vec_ncenter = vec_ncenter*R_2D#rotate the vector to the next microtexture


        ######

            #R = [[c,-s], [s, c]]
            # R = [[c,-s,0], [s, c,0],[0,0,1]] #create a new rotation matrix, currently this formula is for degrees
            # R = np.array(R) #this is a 3x3
            # print("This is the rotation matrix: ")
            # print(R)
            # pos_reader = [rospy.get_param('/pos_reader/x'),rospy.get_param('/pos_reader/y'),rospy.get_param('/pos_reader/z')] #list of the position of the center of the micro-texture?
            # print("this is the pos_reader value: ")
            # print(pos_reader)
            # offset = 15         # unit: mm diameter of the circle (will change with bigger texture) (for single was 65)
            # read_vec_rot = R*np.array([np.cos(theta), np.sin(theta), 0]) # (3x3) multiply this time a rotation matrix (this used to be angle not theta)
            # print("This is the read_vec_rot variable: ")
            # print(read_vec_rot)
            # read_vec_rot = read_vec_rot[0,:]
            # new_center = np.array(pos_reader) + np.array(read_vec_rot)*offset #this is a 3x3 matrix
            # print("This is the original new_center value: ")
            # print(new_center)
            #new_center = new_center[0,:] #pulling the first row
            #print("This is the new new_center value: ")
            #print(new_center)
#                start_point = np.array(new_center) - np.array(read_vec)*0.5*readlength #makes an array of the positions and direction 
#                start_point[2] = rot*180/np.pi#convert to degrees
#                end_point = np.array(new_center) + np.array(read_vec)*0.5*readlength
#                end_point[2] = rot*180/np.pi#convert to degrees
        for rep in xrange(nrep):#peforms the moving and saving data operation
            print('rep = '+str(rep))
            angle = 1.0*rep*angle_step
            print('angle: ',angle)
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
            set_the_speed(angle)
            move_motor(angle,rot,20, new_center)############# add an argument to plug in the new center
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
