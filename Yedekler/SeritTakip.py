#!/usr/bin/env python3

import numpy as np
import cv2

class SeritTakip():

    def acisalHizHesapla(self, img):
        h,w,d = img.shape

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        alt_sari = np.array([20, 100, 100])
        ust_sari = np.array([40, 255, 255])
        maske = cv2.inRange(hsv, alt_sari, ust_sari)
        sonuc = cv2.bitwise_and(img, img, mask=maske)

        cv2.circle(img, (int(w/2), int(h/2)), 5, (0, 0, 255), -1)
        M = cv2.moments(maske)

        if M["m00"] > 0:
            # Serit var
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            cv2.circle(img, (cx,cy), 5, (255, 0, 0), -1)
            sapma = cx - w/2

            cv2.imshow("Orijinal", img)
            print("Aci: " + str(-sapma/100))
            return -sapma/100
        else:
            return -1