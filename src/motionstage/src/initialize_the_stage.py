#!/usr/bin/env python
# license removed for brevity

import gclib

if __name__ == "__main__":
    rospy.init_node('intialize_the_stage', log_level = rospy.INFO)

g = gclib.py()
g.GOpen('169.254.93.220 --direct')
print(g.GInfo())
c = g.GCommand      # alias the command callable