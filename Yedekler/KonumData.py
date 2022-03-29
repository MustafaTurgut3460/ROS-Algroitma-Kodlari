#!/usr/bin/env python3

import rospy
from yarisma_simulasyon.msg import KonumVeri
from nav_msgs.msg import Odometry

class KonumData():
    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.konumCallback)

        self.pub = rospy.Publisher("konum_veri", KonumVeri, queue_size=10)
        self.konum_mesaji = KonumVeri()

        rospy.spin()

    def konumCallback(self, mesaj):
        # burada kendime gore eksenleri ayarlamak icin deneme yoluyla degisiklikler yaptim, simulasyon icin...
        konumX = round(mesaj.pose.pose.position.x, 4)
        konumY = round(mesaj.pose.pose.position.y, 4)
        konumY = -konumY
        konumX = -konumX

        self.konum_mesaji.konumY = konumX
        self.konum_mesaji.konumX = konumY

        self.pub.publish(self.konum_mesaji)
        print(str(self.konum_mesaji.konumX) + " / " + str(-self.konum_mesaji.konumY))
        print("------------------------------")


rospy.init_node("konum_data")
konum = KonumData()
