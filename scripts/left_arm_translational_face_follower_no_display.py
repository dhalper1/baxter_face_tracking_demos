#!/usr/bin/env python
import rospy
from translational_follow_class_no_display import TranslationalFollowArmNoDisplay


def setup():
    rospy.init_node('left_arm_translational_face_follower', anonymous=True)
    RotationalFollowArmNoDisplay('left', '0.276241 0.652758 0.235748 -0.039501 ' +
                                '0.559008 -0.013021 0.828118')
    rospy.spin()


if __name__ == '__main__':
    setup()
