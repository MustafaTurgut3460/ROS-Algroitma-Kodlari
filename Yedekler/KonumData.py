#!/usr/bin/env python3

import rospy
from yarisma_uygulama.msg import KonumVeri
from nav_msgs.msg import Odometry

class KonumData():
    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.konumCallback)

        self.pub = rospy.Publisher("konum_veri", KonumVeri, queue_size=10)
        self.konum_mesaji = KonumVeri()

        rospy.spin()

    def konumCallback(self, mesaj):
        konumX = round(mesaj.pose.pose.position.x, 4)
        konumY = round(mesaj.pose.pose.position.y, 4)

        self.konum_mesaji.konumX = konumX
        self.konum_mesaji.konumY = konumY

        self.pub.publish(self.konum_mesaji)
        print("Konum verisi yayinlaniyor...")
        print("------------------------------")


rospy.init_node("konum_data")
konum = KonumData()
