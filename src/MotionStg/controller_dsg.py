import gclib
import rospy
import roslaunch
import netft_rdt_driver
import std_msgs.msg
from netft_rdt_driver.srv import Zero
from std_msgs.msg import String
from config.helper import pause
import config.helper as helper


topics = ['-a']
setZero = rospy.ServiceProxy('/zero', Zero)

def wait_for_ft_calib():
    from ik.roshelper import ROS_Wait_For_Msg
    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

###################Launch the F/T sensor ros node  ######################
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mcube-daolin/catkin_ws/src/SysConfig/rosnodes.launch"])
launch.start()



####################### Establish the connection to controller through Ethernet #########################
g = gclib.py()
g.GOpen('169.254.93.220 --direct')
print(g.GInfo())




setSpeed(tcp=vel, ori=1000)

dir_save_bagfile = os.environ['PNPUSHDATA_BASE'] + '/friction_scan/%s/%s/' % (opt.surface_id,shape_id)
helper.make_sure_path_exists(dir_save_bagfile)

for rep in xrange(nrep):
    bagfilename = 'record_surface=%s_shape=%s_a=%.0f_v=%.0f_rep=%03d.bag' % (opt.surface_id, shape_id, acc*1000, vel, rep)
    print bagfilename
    bagfilepath = dir_save_bagfile+bagfilename

    if skip_when_exists and os.path.isfile(bagfilepath):
        #print bagfilepath, 'exits', 'skip'
        continue

    setCart([range_x[0], max_y, z], ori)
    setZero()
    wait_for_ft_calib()
    setCart([range_x[0], max_y, z], ori)
    rosbag_proc = helper.start_ros_bag(bagfilename, topics, dir_save_bagfile)

    helper.terminate_ros_node("/record")
        if rep == nrep -1:
            rospy.sleep(1)   # make sure record is terminated completely


######### Generate Motion of the stage ##########
print('Motion Complete')
c = g.GCommand #alias the command callable
c('AB') #abort motion and program
c('MO') #turn off all motors
c('SHA') #servo A
c('SHB') #servo A
c('SPA=10000') #speead, 1000 cts/sec
c('PRA=300000') #relative move, 3000 cts
c('SPB=10000') #speead, 1000 cts/sec
c('PRB=300000') #relative move, 3000 cts
print(' Starting move...')
c('BGA') #begin motion
c('BGB') #begin motion

######### Record the force and torque ##########




############# ?? ################
g.GMotionComplete('A')
g.GMotionComplete('B')
print(' done.')
c('MO') #turn off all motors
del c #delete the alias
