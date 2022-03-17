#!usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

# z ve w degerleri ayni isaretli olursa sol yani bati, ters isaretli olursa dogu yonde oluyor robot.

def callback(mesaj):
    print("X: " + str(mesaj.orientation.x))
    print("Y: " + str(mesaj.orientation.y))
    print("Z: " + str(mesaj.orientation.z))
    print("W: " + str(mesaj.orientation.w))
    print("------------------------------")

rospy.init_node("imu_deneme")
rate = rospy.Rate(10)

rospy.Subscriber("imu", Imu, callback)
rate.sleep()
rospy.spin()