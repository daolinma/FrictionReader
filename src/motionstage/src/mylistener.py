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

# settings.init()

# class MyEncoder(json.JSONEncoder):
#     def default(self, obj):
#         if isinstance(obj, numpy.integer):
#             return int(obj)
#         elif isinstance(obj, numpy.floating):
#             return float(obj)
#         elif isinstance(obj, numpy.ndarray):
#             return obj.tolist()
#         else:
#             return super(MyEncoder, self).default(obj)
def ftmsg2listandflip(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z,
            ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,ftmsg.wrench.torque.z]


def callback_pos(data):
    global pos_record
    rospy.loginfo("I heard %s",data.data)
    pos_record.append(data.data)
    # print pos_record

def callback_wrench(data):
    global wrench_record
    # global wrench_record
    rospy.loginfo("I heard %s",data)
    ft = ftmsg2listandflip(data)
    wrench_record.append([data.header.stamp.to_sec()] + ft)
    # wrench_record.append(data)


def exp_listener():
    stop_sign = False
    # while stop_sign = False
    rospy.init_node('lisener_node')
    rospy.Subscriber("stage_pos", Float32MultiArray, callback_pos)
    rospy.Subscriber("netft_data", WrenchStamped, callback_wrench)
    rospy.spin()

def start_read(req):
    global pos_record
    global wrench_record
    pos_record = []
    wrench_record = []
    return EmptyResponse()

def save_readings(req):
    global pos_record
    global wrench_record
    # print pos_record
    filename = rospy.get_param('save_file_name')
    output_data = {'pos_list':pos_record, 'wrench_list': wrench_record }
    print output_data
    with open(filename, 'w') as outfile:  # write data to 'data.json'
        # json.dump( {'pos_list':pos_record, 'wrench_list': wrench_record }, outfile, cls=MyEncoder)   #TODO: find out why failing to save the file.
        json.dump(output_data, outfile)   #TODO: find out why failing to save the file.
    rospy.sleep(.3)
    return EmptyResponse()


if __name__ == '__main__':
    try:
        s_1 = rospy.Service('start_read', Empty, start_read)
        s_1 = rospy.Service('save_readings', Empty, save_readings)
        exp_listener()
        print ('mylistener ready!')
    except rospy.ROSInterruptException:
        pass
