import rospy
import roslaunch
import netft_rdt_driver
# import std_msgs.msg
from netft_rdt_driver.srv import Zero
from std_msgs.msg import String
# from config.helper import pause
# import config.helper as helper


# waiting for ft_net
def wait_for_ft_calib():
    from ik.roshelper import ROS_Wait_For_Msg
    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

################### Launch the F/T sensor ros node ######################
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mcube-daolin/catkin_ws/src/SysConfig/rosnodes.launch"])
launch.start()

topics = ['-a']
setZero = rospy.ServiceProxy('/zero', Zero)

setZero()
wait_for_ft_calib()
