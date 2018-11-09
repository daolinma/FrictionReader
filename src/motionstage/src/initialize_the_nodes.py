#!/usr/bin/env python
# license removed for brevity

import gclib
import roslaunch

if __name__ == "__main__":
    rospy.init_node('intialize_the_nodes', log_level = rospy.INFO)

################### Launch the F/T sensor ros node & stage nodes ######################
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mcube-daolin/catkin_ws/src/SysConfig/rosnodes.launch"])
launch.start()

# g = gclib.py()
# g.GOpen('169.254.93.220 --direct')
# print(g.GInfo())
# c = g.GCommand      # alias the command callable

