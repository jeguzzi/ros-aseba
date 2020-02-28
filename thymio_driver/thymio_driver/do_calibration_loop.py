import math

import geometry_msgs.msg
import rospy


def main():
    rospy.init_node("open_loop_square")
    rospy.loginfo('Init open loop square')
    pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rospy.sleep(1)
    speed = rospy.get_param('~speed', 0.05)
    axis_length = rospy.get_param('~axis_length', 0.0935)
    side_length = rospy.get_param('~side', 0.6)
    rospy.loginfo('Start square with speed %.3f m/s and side %.3f m', speed, side_length)
    for _ in range(4):
        rospy.loginfo('Side')
        advance(pub, length=side_length, speed=speed)
        rospy.loginfo('Turn')
        turn(pub, math.pi / 2, angular_speed=(speed / axis_length))
        rospy.sleep(1)
    rospy.loginfo('Done')


def stop(pub):
    pub.publish(geometry_msgs.msg.Twist())


def advance(pub, length, speed):
    msg = geometry_msgs.msg.Twist()
    msg.linear.x = speed
    pub.publish(msg)
    rospy.sleep(length / speed)
    stop(pub)


def turn(pub, angle, angular_speed):
    msg = geometry_msgs.msg.Twist()
    msg.angular.z = angular_speed
    pub.publish(msg)
    rospy.sleep(angle / angular_speed)
    stop(pub)


if __name__ == '__main__':
    main()
