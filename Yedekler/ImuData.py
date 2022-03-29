#!/usr/bin/env python3

import rospy
from yarisma_simulasyon.msg import ImuVeri
from sensor_msgs.msg import Imu

class ImuDataClass():
    def __init__(self):
        rospy.init_node("imu_veri")

        rospy.Subscriber("/imu", Imu, self.imuCallback)

        self.pub = rospy.Publisher("imu_veri", ImuVeri, queue_size=10)
        self.imu_mesaji = ImuVeri()
        rospy.spin()

    def imuCallback(self, mesaj):
        imu = mesaj.orientation.z
        acisalImu = mesaj.orientation.w

        self.imu_mesaji.yon = imu
        self.imu_mesaji.aciYon = acisalImu
        self.pub.publish(self.imu_mesaji)
        print("Imu verisi yayinlaniyor...")


imu = ImuDataClass()
