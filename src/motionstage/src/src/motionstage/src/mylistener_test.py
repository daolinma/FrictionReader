#!/usr/bin/env python
# license removed for brevity

import rospy,numpy
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray,Bool
from std_srvs.srv import Empty,EmptyResponse
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import json
# import settings
pos_record = []
wrench_record = []

def ftmsg2listandflip(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z,
            ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,ftmsg.wrench.torque.z]

def start_read():
    global pos_record
    global wrench_record
    pos_record = []
    wrench_record = []

def save_readings():
    global pos_record
    global wrench_record
    rospy.init_node('lisener_test_node')
    # print pos_record
    print 'waiting for publisher'
    data1=rospy.wait_for_message("/stage_pos", Float32MultiArray, 15)
    data2=rospy.wait_for_message("/netft_data", WrenchStamped, 3)
    pos_record.append(data1.data)
    rospy.loginfo("I heard %s",data2)

    ft = ftmsg2listandflip(data2)
    wrench_record.append([data2.header.stamp.to_sec()] + ft)

    filename = rospy.get_param('save_file_name')
    test = {'pos_list':pos_record, 'wrench_list': wrench_record }
    print test
    with open(filename, 'w') as outfile:  # write data to 'data.json'
        json.dump( {'pos_list':pos_record, 'wrench_list': wrench_record }, outfile)   #TODO: find out why failing to save the file.
    rospy.sleep(.3)

if __name__ == '__main__':
    try:
        # s_1 = rospy.Service('start_read', Empty, start_read)
        # s_1 = rospy.Service('save_readings', Empty, save_readings)
        save_readings()
        # exp_listener()
        print ('mylistener ready!')
    except rospy.ROSInterruptException:
        pass
