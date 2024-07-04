#!/usr/bin/env python


def cb(data):
    print_joint_state(data)
    rospy.signal_shutdown('time is up')

if __name__ == '__main__':
    rospy.init_node('muh', anonymous=True)

    rospy.Subscriber('joint_states', JointState, cb)

    rospy.spin()