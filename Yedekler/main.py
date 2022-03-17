#!/usr/bin/env python3

import rospy
from yarisma_uygulama.msg import ImuVeri, KonumVeri
from geometry_msgs.msg import Twist
from SeritTakip import SeritTakip
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class GetData():
    def __init__(self):
        rospy.init_node("main")
        self.konumX = 0.0
        self.konumY = 0.0
        self.hedefYon = 0
        self.yon = 0.0
        self.acisalYon = 0.0
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        self.kuzey = [0.001, 0.71]
        self.guney = [0.71, 0.99]
        self.dogu = False

        # verileri abone olarak aldik
        rospy.Subscriber("imu_veri", ImuVeri, self.imuCallback)
        rospy.Subscriber("konum_veri", KonumVeri, self.konumCallback)
        rospy.Subscriber("camera/rgb/image_raw", Image, self.kameraCallback)
        # hiz yayincisi
        self.hizPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.hiz_mesaji = Twist()
        # cizgi takip durumu yayincisi

        # dugum sonlanana kadar devam
        while not rospy.is_shutdown():
            

            # ortadan bolunce robot ne tarafa bakiyor?
            if (self.yon > 0 and self.acisalYon < 0) or (self.yon < 0 and self.acisalYon > 0):
                self.dogu = True
            else:
                self.dogu = False


            # hedef konumu alip hedefin hangi yonde oldugunu bulalim
            inputX = float(input("X konumu: "))
            inputY = float(input("Y konumu: "))

            # Verilen konumun robota gore konumu bulma
            # hedef konum: KD:1, KB:2, GB:3, GD:4 degerleri verilmistir.
            if inputX > self.konumX and inputY > self.konumY:
                print("Hedef Kuzeydoguda!")
                self.hedefYon = 1
            elif inputX > self.konumX and inputY < self.konumY:
                print("Hedef Guneydoguda!")
                self.hedefYon = 4
            elif inputX < self.konumX and inputY > self.konumY:
                print("Hedef Kuzeybatida!")
                self.hedefYon = 2
            else:
                print("Hedef Guneybatida!")
                self.hedefYon = 3

            self.robotuDondur(self.hedefYon)

            if self.hedefYon == 1 or self.hedefYon == 2:
                self.araliktaDondur(self.kuzey)
            else:
                self.araliktaDondur(self.guney)

           

            """Geldik zurnanin zort dedigi yere...
            Simdi algoritma su sekilde:
            Bize bir hedef nokta verilecek ve biz bu hedef noktaya giden yolu bulacagiz. Bunu seu sekilde yapabilirim:
            Hedef nokta girilecek ve bu nokta hangi yonde oldugu bulunacak. Mesela kuzeybati olsun. Daha sonra robot yon
            verisine gore kuzeybati yonune donecek yani belirli bir araliga. Burada yol bulabilirse bulacak bulamazsa baska
            bir yolu takip edecek."""

            

            self.rate.sleep()

    def imuCallback(self, mesaj):
        self.yon = mesaj.yon
        self.acisalYon = mesaj.aciYon

    def konumCallback(self, mesaj):
        self.konumX = mesaj.konumX
        self.konumY = mesaj.konumY

    def kameraCallback(self, mesaj):
        self.img = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")

    def robotuDondur(self, yon:int):
        self.hiz_mesaji.angular.z = 0.2
        if yon == 1:
            # Kuzeydogu
            while not(abs(self.yon) >= self.kuzey[0] and abs(self.yon) <= self.kuzey[1] and self.dogu): # kuzeydogu araligina kadar donmeye devam et
                if (self.yon > 0 and self.acisalYon < 0) or (self.yon < 0 and self.acisalYon > 0):
                    self.dogu = True
                else:
                    self.dogu = False

                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Kuzeydoguya donuldu!")

        elif yon == 2:
            # Kuzeybati
            while not(abs(self.yon) >= self.kuzey[0] and abs(self.yon) <= self.kuzey[1] and not self.dogu):
                if (self.yon > 0 and self.acisalYon < 0) or (self.yon < 0 and self.acisalYon > 0):
                    self.dogu = True
                else:
                    self.dogu = False
                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Kuzeybatiya donuldu!")

        elif yon == 3:
            # Guneybati
            while not(abs(self.yon) >= self.kuzey[1] and not self.dogu):
                if (self.yon > 0 and self.acisalYon < 0) or (self.yon < 0 and self.acisalYon > 0):
                    self.dogu = True
                else:
                    self.dogu = False
                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Guneybatiya donuldu!")

        elif yon == 4:
            # Guneydogu
            while not(abs(self.yon) >= self.kuzey[1] and self.dogu):
                if (self.yon > 0 and self.acisalYon < 0) or (self.yon < 0 and self.acisalYon > 0):
                    self.dogu = True
                else:
                    self.dogu = False
                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Guneydoguya donuldu!")

        else:
            self.hiz_mesaji.angular.z = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Yon bulunamadi!")

    def araliktaDondur(self, aralik:list):
        # belirlenen aralikta donerken cizgi yani yol aramaya devam edecegiz
        seritTakip = SeritTakip()
        
        while abs(self.yon) >= aralik[0] and abs(self.yon) <= aralik[1]:
            # robotun yonu aralikta iken donmeye devam et
            acisal_hiz = seritTakip.acisalHizHesapla(self.img)

            if acisal_hiz != -1:
                # serit bulundu takip edelim, hiz verelim
                self.hiz_mesaji.linear.x = 0.2
                self.hiz_mesaji.angular.z = acisal_hiz
                self.hizPub.publish(self.hiz_mesaji)
            else:
                # serit bulunamadi, aramaya devam edelim
                self.hiz_mesaji.linear.x = 0.0
                self.hiz_mesaji.angular.z = 0.2
                self.hizPub.publish(self.hiz_mesaji)
                print("Aralikta serit araniyor...")

        self.hiz_mesaji.angular.z = 0.0
        self.hizPub.publish(self.hiz_mesaji)

        print("Aralik donuldu!")
            



data = GetData()