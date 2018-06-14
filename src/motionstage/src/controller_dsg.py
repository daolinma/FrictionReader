import gclib
import rospy
import roslaunch
import netft_rdt_driver
# import stage_position_node
import std_msgs.msg
from netft_rdt_driver.srv import Zero
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray


# waiting for netft
def wait_for_ft_calib():
    from ik.roshelper import ROS_Wait_For_Msg
    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

# write the subscriber of stage position  # TODO: Add subscriber

def callback(data):
    rospy.loginfo("I heard %s",data.data)

def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("stage_pos", Float32MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


################### Launch the F/T sensor ros node ######################
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mcube-daolin/catkin_ws/src/SysConfig/rosnodes.launch"])
launch.start()

setZero = rospy.ServiceProxy('/zero', Zero)
setZero()
wait_for_ft_calib()

####################### Establish the connection to controller through Ethernet #########################
g = gclib.py()
g.GOpen('169.254.93.220 --direct')
print(g.GInfo())
c = g.GCommand      # alias the command callable
c('ABC') #abort motion and program
c('MO') #turn off all motors

c('SH ABC')    # Servo Here: servo A
# c('SHB')    # Servo Here: servo A

c('SPA=10000') #speead, 1000 cts/sec
c('SPB=10000') #speead, 1000 cts/sec
c('SPC=10000') #speead, 1000 cts/sec

####################### Scan the surface ######################
# set the motion stage Speed

for rep in xrange(nrep):
    expfilename = 'record_surface=%s_shape=%s_a=%.0f_v=%.0f_rep=%03d.bag' % (opt.surface_id, shape_id, acc*1000, vel, rep)

    ######### Generate Motion of the stage ##########
    c('PRA=300000') #relative move, 3000 cts
    c('PRB=300000') #relative move, 3000 cts
    print(' Starting move...')
    c('BG AB') #begin motion
    # c('BGA') #begin motion
    # c('BGB') #begin motion
    listener()

    if rep == nrep -1:
        rospy.sleep(1)   # make sure record is terminated completely

    ######### Record the force and torque ##########


############# ?? ################
g.GMotionComplete('A')
g.GMotionComplete('B')
print('Motion Complete')
print(' done.')
c('MO') #turn off all motors
del c #delete the alias
