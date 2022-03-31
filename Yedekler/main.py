#!/usr/bin/env python3

import rospy
from yarisma_simulasyon.msg import ImuVeri, KonumVeri
from geometry_msgs.msg import Twist
from CizgiTakip import SeritTakip
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
        self.seritVarmi = False

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
            self.inputX = float(input("X konumu: "))
            self.inputY = float(input("Y konumu: "))

            # Verilen konumun robota gore konumu bulma
            # hedef konum: KD:1, KB:2, GB:3, GD:4 - K:5, G:6, D:7, B:8 degerleri verilmistir.
            if abs(self.inputX - self.konumX) <= 0.2: # eger aralarinda kucuk bir fark var ise ayni olarak alalim
                if self.inputY > self.konumY:
                    print("Hedef Kuzeyde, donuluyor...")
                    self.hedefYon = 5
                else:
                    print("Hedef Guneyde, donuluyor...")
                    self.hedefYon = 6

            elif abs(self.inputY - self.konumY) <= 0.2: # ayni sekilde kucuk farklari yok sayiyiyoruz
                if self.inputX > self.konumX:
                    print("Hedef Doguda, donuluyor...")
                    self.hedefYon = 7
                else:
                    print("Hedef Batida, donuluyor...")
                    self.hedefYon = 8

            elif self.inputX > self.konumX and self.inputY > self.konumY:
                print("Hedef Kuzeydoguda, donuluyor...")
                self.hedefYon = 1

            elif self.inputX > self.konumX and self.inputY < self.konumY:
                print("Hedef Guneydoguda, donuluyor...")
                self.hedefYon = 4

            elif self.inputX < self.konumX and self.inputY > self.konumY:
                print("Hedef Kuzeybatida, donuluyor...")
                self.hedefYon = 2

            else:
                print("Hedef Guneybatida, donuluyor...")
                self.hedefYon = 3

            self.robotuDondur(self.hedefYon)

            if self.hedefYon == 1 or self.hedefYon == 2 or self.hedefYon == 5 or self.hedefYon == 7 or self.hedefYon == 8:
                # kuzey, kuzeydogu, kuzeybati, dogu, bati yonleri
                self.araliktaYolBul(self.kuzey)
            else:
                # guney, guneybati, guneydogu yonleri
                self.araliktaYolBul(self.guney)

            self.rate.sleep()

    def imuCallback(self, mesaj):
        self.yon = mesaj.yon
        self.acisalYon = mesaj.aciYon

    def konumCallback(self, mesaj):
        self.konumX = mesaj.konumX
        self.konumY = -(mesaj.konumY) # kendime gore ekseni ayarlamak icin tersini aldik
        #print(self.konumY)

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
            self.hiz_mesaji.linear.x = 0.0
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
            self.hiz_mesaji.linear.x = 0.0
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
            self.hiz_mesaji.linear.x = 0.0
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
            self.hiz_mesaji.linear.x = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Guneydoguya donuldu!")

        elif yon == 5:
            # Kuzey
            # kuzey aci araligi pozitif olarak 0-0.1 araligidir.
            while not abs(self.yon) <= 0.1:
                # kuzey olana kadar donmeye devam et
                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hiz_mesaji.linear.x = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Kuzeye donuldu!")

        elif yon == 6:
            # Guney
            # guney aci araligi pozitif olarak 0.97-0.99
            while not abs(self.yon) >= 0.97:
                # guneye gelene kadar don
                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hiz_mesaji.linear.x = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Guneye donuldu!")
            
        elif yon == 7:
            # Dogu
            # dogu aci araligi pozitif olarak 0.68-0.71 araligidir.
            while not(abs(self.yon) >= 0.68 and abs(self.yon) <= 0.71 and self.dogu):
                if (self.yon > 0 and self.acisalYon < 0) or (self.yon < 0 and self.acisalYon > 0):
                    self.dogu = True
                else:
                    self.dogu = False
                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hiz_mesaji.linear.x = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Doguya donuldu!")
            
        elif yon == 8:
            # Bati
            # ayni sekilde aralik 0.68-0.71 dir.
            while not(abs(self.yon) >= 0.68 and abs(self.yon) <= 0.71 and not self.dogu):
                if (self.yon > 0 and self.acisalYon < 0) or (self.yon < 0 and self.acisalYon > 0):
                    self.dogu = True
                else:
                    self.dogu = False
                self.hizPub.publish(self.hiz_mesaji)

            self.hiz_mesaji.angular.z = 0.0
            self.hiz_mesaji.linear.x = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Doguya donuldu!")

        else:
            self.hiz_mesaji.angular.z = 0.0
            self.hiz_mesaji.linear.x = 0.0
            self.hizPub.publish(self.hiz_mesaji)
            print("Yon bulunamadi!")

    def araliktaYolBul(self, aralik:list):
        # belirlenen aralikta donerken cizgi yani yol aramaya devam edecegiz
        seritTakip = SeritTakip()

        xAralik = [self.inputX - 0.1, self.inputX + 0.1]
        yAralik = [self.inputY - 0.1, self.inputY + 0.1]

        
        while (abs(self.yon) >= aralik[0] and abs(self.yon) <= aralik[1]) or self.seritVarmi:
            # aralikta serit aramaya devam et bulamadigin surece
            acisal_hiz = seritTakip.acisalHizHesapla(self.img)
            #cv2.imshow("Kamera", self.img)

            if self.konumKontrol(xAralik, yAralik): # hedefe varidli mi kontrol et
                print("Hedefe Ulasildi!")
                break

            elif acisal_hiz != -1:
                # serit bulundu takip edelim, hiz verelim
                self.seritVarmi = True
                self.hiz_mesaji.linear.x = 0.2
                self.hiz_mesaji.angular.z = acisal_hiz
                self.hizPub.publish(self.hiz_mesaji)
            else:
                # serit bulunamadi, aramaya devam edelim
                self.seritVarmi = False
                self.hiz_mesaji.linear.x = 0.0
                self.hiz_mesaji.angular.z = 0.2
                self.hizPub.publish(self.hiz_mesaji)
                print("Aralikta serit araniyor...")

        self.hiz_mesaji.angular.z = 0.0
        self.hiz_mesaji.linear.x = 0.0
        self.hizPub.publish(self.hiz_mesaji)

        print("Aralik donuldu!")

    def konumKontrol(self, xAralik:list, yAralik:list):
        # konuma ulasildi mi?
        control = (self.konumX >= xAralik[0] and self.konumX <= xAralik[1]) and (self.konumY >= yAralik[0] and self.konumY <= yAralik[1])

        return control



data = GetData()
