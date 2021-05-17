#!/usr/bin/env python3

import rospy
from daly_bms import DalyBMS


def main():

    rospy.init_node("daly_bms_node")

    rc_node = DalyBMS()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()