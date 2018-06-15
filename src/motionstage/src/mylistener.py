#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray,Bool
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import json
# import settings
# exp_record = {'pos':[],'ft':[]}


# def init():
global pos_record
global wrench_record
pos_record = []
wrench_record = []

# settings.init()

def callback_pos(data):
    rospy.loginfo("I heard %s",data.data)
    pos_record.append(data.data)

def callback_wrench(data):
    # global wrench_record
    rospy.loginfo("I heard %s",data)
    wrench_record.append(data)

def exp_listener():
    stop_sign = False
    # while stop_sign = False
    rospy.init_node('lisener_node')
    rospy.Subscriber("stage_pos", Float32MultiArray, callback_pos)
    rospy.Subscriber("netft_data", WrenchStamped, callback_wrench)
    # stop_sign = rospy.wait_for_message("reading_status", Bool, 3);
    rospy.spin()

def start_read():
    # global pos_record
    # global wrench_record
    pos_record = []
    wrench_record = []

def readandsave(filename = 'output.json'):
    print pos_record
    with open(filename, 'w') as outfile:  # write data to 'data.json'
        json.dump({'pos_list':pos_record, 'wrench_list': wrench_record}, outfile)


if __name__ == '__main__':
    try:
        exp_listener()
    except rospy.ROSInterruptException:
        pass
