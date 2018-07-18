#!/usr/bin/env python
# license removed for brevity

import rospy,numpy
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray,Bool
from std_srvs.srv import Empty,EmptyResponse
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import json
import matplotlib.pyplot as plt
import roslaunch
import netft_rdt_driver

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mcube-daolin/catkin_ws/src/SysConfig/rosnodes.launch"])
launch.start()

idx = 0
def ftmsg2listandflip(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z,
            ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,ftmsg.wrench.torque.z]

def ftmsg2listandflip(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z,
            ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,ftmsg.wrench.torque.z]


def callback_wrench(data):
    global idx
    idx = idx+1
    # rospy.loginfo("I heard %s",data)
    # ft = ftmsg2listandflip(data)
    plt.scatter(idx,data.wrench.force.z)
    print idx
    print data.wrench.force.z
    plt.pause(0.01)

    # wrench_record.append([data.header.stamp.to_sec()] + ft)
    # wrench_record.append(data)

def exp_listener():
    global idx
    while True:
        idx = idx+1
        # rospy.Subscriber("netft_data", WrenchStamped, callback_wrench)
        msg = rospy.wait_for_message("/netft_data" , WrenchStamped, 3)
        plt.scatter(idx,msg.wrench.force.z)
        print idx
        print msg.wrench.force.z
        plt.pause(0.01)
        # plt.show()
        # print 'sss', idx
        # rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('read_FT_plot', log_level = rospy.INFO)
        print ('reading ready!')
        exp_listener()
    except rospy.ROSInterruptException:
        pass