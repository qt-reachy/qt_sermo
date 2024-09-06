#!/usr/bin/env python3

import rospy
from sermo import sermo_handler
from qt_sermo.srv import *

if __name__ == '__main__':
    rospy.init_node('qt_sermo')
    rospy.loginfo("qt_sermo_node started!")

    # init classes

    service = rospy.Service(
        'qt_robot/sermo_chat', sermo_chat, sermo_handler
    )

    rospy.spin()
    

    rospy.loginfo("Finished interaction")