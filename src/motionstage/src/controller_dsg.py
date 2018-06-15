import gclib
import rospy
import roslaunch
import json
import std_msgs.msg
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray
from geometry_msgs.msg import WrenchStamped

import netft_rdt_driver
from netft_rdt_driver.srv import Zero
# import stage_position_node
import mylistener
import settings

# waiting for netft
#def wait_for_ft_calib():
#    from roshelper import ROS_Wait_For_Msg
#    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

def initialize_the_motor():
    c('MO') #turn off all motors
    c('SH ABC')    # Servo Here: servo A
    # c('SHB')    # Servo Here: servo A
def set_the_speed():
    c('SPA=-30000') #speead, 1000 cts/sec
    c('SPB=30000') #speead, 1000 cts/sec
    c('SPC=100000') #speead, 1000 cts/sec
def move_motor():
    ######### Generate Motion of the stage ##########
    c('PRA=-300000') #relative move, 3000 cts
    c('PRB=300000') #relative move, 3000 ctsc
    c('PRC=0') #relative move, 3000 cts
    print(' Starting move...')
    c('BG ABC') #begin motion

# mylistener.init()

################### Launch the F/T sensor ros node ######################
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mcube-daolin/catkin_ws/src/SysConfig/rosnodes.launch"])
launch.start()

rospy.sleep(1)
setZero = rospy.ServiceProxy('/zero', Zero)
rospy.sleep(0.5)

setZero()
rospy.sleep(1)
# wait_for_ft_calib()

####################### Establish the connection to controller through Ethernet #########################
g = gclib.py()
g.GOpen('169.254.93.220 --direct')
print(g.GInfo())
c = g.GCommand      # alias the command callable

initialize_the_motor()      # initalization
set_the_speed()             # set the motion stage Speed
# c('MO') #turn off all motors

####################### Scan the surface ######################
# mylistener.init()
# exp_listener()
# rospy.sleep(100)


nrep =1
surface_id = 1; # parallel
shape_id = 1;   # ball
delta = 30;     # 30um
height = 30     #30um
vel = 30        #30mm/s

print('pos_record',mylistener.pos_record)
print('wrench_record',mylistener.wrench_record)
# rospy.sleep(30)

for rep in xrange(nrep):
    expfilename = 'record_surface=%s_shape=%s_delta=%.0f_height=%.0f_vel=%.0f.csv' % (surface_id, shape_id,delta, height, vel)
    # rospy.sleep(30)
    mylistener.start_read()
    move_motor()
    print('hello 1')
    g.GMotionComplete('A')
    g.GMotionComplete('B')
    g.GMotionComplete('C')
    print('Motion Complete')
    print(' done.')
    mylistener.readandsave(filename = expfilename)
    if rep == nrep -1:
        rospy.sleep(1)   # make sure record is terminated completely

    ######### Record the force and torque ##########


############# ?? ################

c('MO') #turn off all motors
del c #delete the alias
