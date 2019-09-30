import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def init():
    rospy.init_node("homework1_driver")

def pose_callback(msg):
    print msg

def main():
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    subscriber = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    vel_msg = Twist()
    vel_msg.linear.x = 0.2

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        publisher.publish(vel_msg)
        rate.sleep()


if __name__ == "__main__":
    init()
    main()