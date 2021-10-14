#!/usr/bin/env python3

import cv2
import numpy as np
from imutils import contours
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, Range
from MRS_MAV import MRS_MAV
from geometry_msgs.msg import Vector3
import time
# import imutils

DEBUG = False
DIGITS_LOOKUP = {
            (1, 1, 1, 0, 1, 1, 1): 0,
            # (0, 0, 1, 0, 0, 1, 0): 1,
            (1, 0, 1, 1, 1, 0, 1): 2,
            (1, 0, 1, 1, 1, 1, 1): 3,
            (1, 0, 1, 1, 0, 1, 1): 3,
            (0, 1, 1, 1, 0, 1, 0): 4,
            (1, 1, 0, 1, 0, 1, 1): 5,
            (1, 1, 0, 1, 1, 1, 1): 6,
            # (0, 1, 0, 1, 1, 1, 1): 6,
            (1, 0, 1, 0, 0, 1, 0): 7,
            (1, 1, 1, 1, 1, 1, 1): 8,
            (1, 1, 1, 1, 0, 1, 1): 9
        }

class display_cv:
    def __init__(self):
        # self.cap = cv2.VideoCapture(0)
        self.rate = rospy.Rate(60)
        self.bridge_object = CvBridge()
        self.cam_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw/", Image, self.cam_callback)
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw/", Image)

    def cam_callback(self, data):
        try:
            self.cam_frame = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

    def main_loop(self):
        while not rospy.is_shutdown():
            # sucess, frame = self.cap.read()
            # frame = np.array(frame)
            #print(frame.shape) #cursed BGR
            frame = self.cam_frame
            # print(np.shape(frame))
            # frame = np.rot90(frame, k=3, axes=(0,1))
            # cv2.imshow("digit_recog test", frame)
            # cv2.waitKey(0)
            #Tratamento de imagem
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # blurred = cv2.GaussianBlur(gray, (3, 3), 0)

            #test begin

            blur = cv2.GaussianBlur(frame, (3, 3), 0)

            # convert to hsv and get saturation channel
            sat = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)[:,:,1]

            # threshold saturation channel
            thresh = cv2.threshold(sat, 50, 255, cv2.THRESH_BINARY)[1] #50 255

            # apply morphology close and open to make mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
            morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
            mask = cv2.morphologyEx(morph, cv2.MORPH_OPEN, kernel, iterations=1)

            # do OTSU threshold to get circuit image
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]

            # write black to otsu image where mask is black
            otsu_result = otsu.copy()
            otsu_result[mask==0] = 0

            # write black to input image where mask is black
            img_result = frame.copy()
            img_result[mask==0] = 0

            # cv2.imshow("otsu_result", otsu_result)
            # cv2.imshow("img_result", img_result)
            # cv2.imshow("otsu", otsu)
            # cv2.imshow("frame", frame)
            # cv2.waitKey(30) #pro pc do igor n morrer

            #end test

            # thresh = cv2.threshold(blurred, 150, 255,	cv2.THRESH_BINARY)[1]           
            if DEBUG:
                cv2.imshow("threshold", thresh)
                cv2.waitKey(30) #pro pc do igor n morrer
            # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            # thresh = cv2.dilate(thresh,kernel,iterations = 1)
            if DEBUG:
                cv2.imshow("tratada", thresh)
                print(np.shape(thresh))
                cv2.waitKey(30) #pro pc do igor n morrer
            # cnts = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # cnts = cv2.findContours(otsu.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # cnts = cv2.findContours(otsu.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            cnts, hierarchy = cv2.findContours(otsu.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # cnts = imutils.grab_contours(cnts)
            # warped = frame
            
            for c in cnts:
                (x1, y1, w1, h1) = cv2.boundingRect(c)
                i = 0
                if (h1)/(w1) >= 0.85 and (h1)/(w1) <= 1.15 and cv2.contourArea(c) <= 20000:
                    for c2 in cnts:
                        (x2, y2, w2, h2) = cv2.boundingRect(c2)
                        if( (x2 > x1) and (y2 > y1) and (x1 + w1 > x2 + w2) and (y1 + h1 > y2 + h2) ):
                            if i >= 8:
                                #cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0,255,0))
                                toBeWarped  = self.four_point_transform(frame, np.array([[x1, y1], [x1, y1 + h1], [x1 + w1, y1], [x1 + w1, y1 + h1]], dtype="float32"))
                                # toBeWarped  = self.four_point_transform(otsu, np.array([[x1, y1], [x1, y1 + h1], [x1 + w1, y1], [x1 + w1, y1 + h1]], dtype="float32"))
                                # cv2.imshow("frame inside for", toBeWarped)
                                # warped_alpha = self.four_point_transform(thresh, np.array([[x1, y1], [x1, y1 + h1], [x1 + w1, y1], [x1 + w1, y1 + h1]], dtype="float32"))
                                warped_alpha = self.four_point_transform(otsu, np.array([[x1, y1], [x1, y1 + h1], [x1 + w1, y1], [x1 + w1, y1 + h1]], dtype="float32"))
                                points = self.getPoints(warped_alpha)
                                warped = self.four_point_transform(toBeWarped, np.array(points, dtype="float32"))
                                #cv2.imshow("warped_alpha",warped_alpha)
                                #cv2.imshow("warped",warped)
                                #cv2.waitKey(15)
                                #cv2.waitKey(0)
                                if self.checkOrientation(warped):
                                    if self.digit_recog(warped) == True:
                                        #break
                                        return True
                                else:
                                    return 0
                            else:
                                i += 1
                                #cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0,255,0))

                if i >= 8:
                    break
            return 0
            #cv2.imshow("teste webcam", warped)
            #cv2.waitKey(30) #pro pc do igor n morrer
    def digit_recog(self, image):
        # duration = 3  # seconds
        # freq = 440  # Hz

        # DIGITS_LOOKUP = {
        #     (1, 1, 1, 0, 1, 1, 1): 0,
        #     (0, 0, 1, 0, 0, 1, 0): 1,
        #     (1, 0, 1, 1, 1, 1, 0): 2,
        #     (1, 0, 1, 1, 0, 1, 1): 3,
        #     (0, 1, 1, 1, 0, 1, 0): 4,
        #     (1, 1, 0, 1, 0, 1, 1): 5,
        #     (1, 1, 0, 1, 1, 1, 1): 6,
        #     (1, 0, 1, 0, 0, 1, 0): 7,
        #     (1, 1, 1, 1, 1, 1, 1): 8,
        #     (1, 1, 1, 1, 0, 1, 1): 9
        # }

        # path = "display_metano0.png"
        # # #Baixando a imagem
        # image = cv2.imread("" + path)
        #Tratamento de imagem
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (1, 1), 0)
        thresh = cv2.threshold(blurred, 0, 255,	cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 4))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        # thresh = cv2.dilate(thresh,kernel,iterations = 3)
        thresh = cv2.dilate(thresh,kernel,iterations = 1)
        #cv2.imshow("digit_recog test", thresh)
        cv2.waitKey(15)

        cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = imutils.grab_contours(cnts)
        # cnts = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        digitCnts = []

        # if(len(cnts) >= 20):
        #     for c in cnts:
        #         (x, y, w, h) = cv2.boundingRect(c)
        #         #cv2.rectangle(image, (x, y), (x + w, y + h), (0,255,0))
        #         #digitCnts.append(c)
        #     cv2.imshow("teste_02", image)
        #     cv2.waitKey(15)

        for c in cnts:
            (x, y, w, h) = cv2.boundingRect(c)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (0,255,0))
            # print(x, y, h, w)
            #print(np.shape(image)[0], np.shape(image)[1])
            # print(float((h*w))/float((np.shape(image)[0]*np.shape(image)[1]))) ((h)/(w) >= 1.5 and (h)/(w) <= 2.5)
            if ((h/w <= 8 and (h)/(w) >= 3 and (float(h*w))/(float(np.shape(image)[0]*np.shape(image)[1])) >= 0.019) or 
            (float(h)/float(w) >= 1.7 and float(h)/float(w) <= 2.8 and (float(h*w))/(float(np.shape(image)[0]*np.shape(image)[1])) >= 0.045) or 
            (float(h)/float(w) <= 0.90 and float(h)/float(w) >= 0.1 and (float(h*w))/(float(np.shape(image)[0]*np.shape(image)[1])) >= 0.01)):# and ((w)/(h) >=1.5 or (w)/(h)<=0.7):
            # if (((h)/(w) <= 0.7 and (h)/(w) >= 0.35) or ((h)/(w) >= 1.5 and (h)/(w) <= 2.5)):# and ((w)/(h) >=1.5 or (w)/(h)<=0.7):
            #if((w<=(np.shape(image)[1])/23 and h<=(np.shape(image)[0])*(15/46)) or (w<=(np.shape(image)[1])*(7/46) and h<=(np.shape(image)[0])*(7/92)) or ((w<=np.shape(image)[1]*(19/92) and w>=(np.shape(image)[1])*(15/92)) and (h<=(np.shape(image)[0])*(17/46) and h>=(np.shape(image)[0])*(31/92)))):
            #if((w<=20 and h<=150) or (w<=70 and h<=35) or ((w<=95 and w>=75) and (h<=170 and h>=155))):
                if (x + w) < (5*(np.shape(image)[1]))/6:
                    # print(x, y, h, w)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0,255,0))
                    digitCnts.append(c)
        #cv2.imshow("teste_02", image)
        # cv2.waitKey(0)

        # digitCnts.sort(reverse=True, key=self.sorter)
        linha1 = []
        linha2 = []

        for c in digitCnts:
            (x, y, w, h) = cv2.boundingRect(c)
            if y + 15 < (np.shape(image)[0])/2:
                linha1.append(c)
            else:
                linha2.append(c)

        linha1.sort(reverse=True, key=self.sorter)
        linha2.sort(reverse=False, key=self.sorter)
        
        if len(linha1) > 0 and len(linha2) > 0:
            contours.sort_contours(linha1)
            contours.sort_contours(linha2)
        else:
            print("No contours")
            return False


        Digits = []
        first_number_digit_count = 0
        second_number_digit_count = 0

        for c in linha1:
            if first_number_digit_count < 2:
                (x, y, w, h) = cv2.boundingRect(c)
                roi = thresh[y:y+h, x:x+w]
                # roi = gray[y:y+h, x:x+w]
                (roiH, roiW) = roi.shape
                # if (((h)/(w) <= 8 and (h)/(w) >= 2)):
                if ((h)/(w) <= 8 and (h)/(w) >= 3):
                # if (not (float(h)/float(w) >= 1.8 and float(h)/float(w) <= 2.2)):
                # if ((h/w) >= 4 and (w/h)>=0.06):
                #if (roiW<=(np.shape(image)[1])/23 and roiH<=(np.shape(image)[0])*(15/46)): #digit one
                    segROI = roi[0:h, 0:w]
                    total = cv2.countNonZero(segROI)
                    area = (h) * (w)
                    if float(area) > 0 and total/float(area) > 0.5:
                        Digits.append("1")
                        first_number_digit_count += 1
                        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
                        cv2.putText(image,"1", (x - 10, y + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                    continue
                elif (float(h)/float(w) >= 1.7 and float(h)/float(w) <= 2.8): #any other digit
                    (dW, dH) = (int(roiW * 0.25), int(roiH * 0.15))
                    dHC = int(roiH * 0.05)
                    segments = [
                        ((0, 0), (w, dH)),	# top
                        ((0, 0), (dW, h // 2)),	# top-left
                        ((w - dW, 0), (w, h // 2)),	# top-right
                        ((0, (h // 2) - dHC) , (w, (h // 2) + dHC)), # center
                        ((0, h // 2), (dW, h)),	# bottom-left
                        ((w - dW, h // 2), (w, h)),	# bottom-right
                        ((0, h - dH), (w, h))	# bottom
                    ]
                    on = [0] * len(segments)
                    for (i, ((xA, yA), (xB, yB))) in enumerate(segments):
                        segROI = roi[yA:yB, xA:xB]
                        # cv2.imshow("segROI", segROI)
                        # cv2.waitKey(15)
                        # print(type(segROI))
                        total = cv2.countNonZero(segROI)
                        area = (xB - xA) * (yB - yA)
                        if float(area) > 0 and (total / float(area) > 0.50) and segROI[int(np.shape(segROI)[0]/2), int(np.shape(segROI)[1]/2)] != 0:
                        # if segROI[(yB - yA)/2, (xB - xA)/2]:
                            on[i]= 1
                    try:
                        digit = DIGITS_LOOKUP[tuple(on)]
                        if(digit == 8 and float(h)/float(w) > 2.2):
                            digit = 3
                        Digits.append(str(digit))
                        first_number_digit_count += 1
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 1)
                        cv2.putText(image, str(digit), (x - 10, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                    except KeyError:
                        # print("first number error")
                        continue
                        # return False

                
        for c in linha2:
            if second_number_digit_count < 3:
                (x, y, w, h) = cv2.boundingRect(c)
                roi = thresh[y:y+h, x:x+w]
                # roi = image[y:y+h, x:x+w]
                (roiH, roiW) = roi.shape
                # h do menos = w do um
                # w do menos = h/2 do um
                if (float(h)/float(w) <= 0.9 and float(h)/float(w) >= 0.1):
                # if ((w/h) >= 2 and (h/w)>=0.12): #-
                    segROI = roi[0:h, 0:w]
                    total = cv2.countNonZero(segROI)
                    area = (h) * (w)
                    if float(area) > 0 and total/float(area) > 0.5:
                        Digits.append('-')
                        second_number_digit_count += 1
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 1)
                        cv2.putText(image,"minus sign", (x - 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
                    continue
                # elif (((h)/(w) <= 8 and (h)/(w) >= 2)):
                elif ((h)/(w) <= 8 and (h)/(w) >= 3):
                # elif ((h/w) >= 4 and (w/h)>=0.06): #digit one
                    segROI = roi[0:h, 0:w]
                    total = cv2.countNonZero(segROI)
                    area = (h) * (w)
                    if float(area) > 0 and total/float(area) > 0.5:
                        Digits.append("1")
                        second_number_digit_count += 1
                        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
                        cv2.putText(image,"1", (x - 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                    continue
                elif (float(h)/float(w) >= 1.7 and float(h)/float(w) <= 2.8): #any other digit
                    (dW, dH) = (int(roiW * 0.25), int(roiH * 0.15))
                    dHC = int(roiH * 0.05)
                    segments = [
                        ((0, 0), (w, dH)),	# top
                        ((0, 0), (dW, h // 2)),	# top-left
                        ((w - dW, 0), (w, h // 2)),	# top-right
                        ((0, (h // 2) - dHC) , (w, (h // 2) + dHC)), # center
                        ((0, h // 2), (dW, h)),	# bottom-left
                        ((w - dW, h // 2), (w, h)),	# bottom-right
                        ((0, h - dH), (w, h))	# bottom
                    ]
                    on = [0] * len(segments)
                    for (i, ((xA, yA), (xB, yB))) in enumerate(segments):
                        segROI = roi[yA:yB, xA:xB]
                        # cv2.imshow("segROI", segROI)
                        # cv2.waitKey(15)
                        # print(type(segROI))
                        total = cv2.countNonZero(segROI)
                        area = (xB - xA) * (yB - yA)
                        if float(area) > 0 and (total / float(area) > 0.50) and segROI[int(np.shape(segROI)[0]/2), int(np.shape(segROI)[1]/2)] != 0:
                        # if segROI[(yB - yA)/2, (xB - xA)/2]:
                            on[i]= 1
                    # print(tuple(on))
                    try:
                        digit = DIGITS_LOOKUP[tuple(on)]
                        Digits.append(str(digit))
                        second_number_digit_count += 1
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 1)
                        cv2.putText(image, str(digit), (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                    except KeyError:
                        # print("second number error")
                        continue
                        # return False
        # try:
        #     primeiro_digito = int(Digits[0]+Digits[1])
        # except ValueError:
        #     # print("first number's digit error")
        #     return False
        
        primeiro_digito = 0
        i = 0
        # print(Digits)
        try:
            if first_number_digit_count > 0:
                while i < first_number_digit_count:
                    primeiro_digito += int(Digits[i]) * (10**i)
                    i += 1

            segundo_digito = ''
            if (i < first_number_digit_count + second_number_digit_count) and (Digits[i] == '-'):
                while(i < first_number_digit_count + second_number_digit_count):
                    segundo_digito += Digits[i]
                    i += 1
                segundo_digito = int(''.join(segundo_digito))
                # segundo_digito *= -1
            else:
                while(i < first_number_digit_count + second_number_digit_count):
                    segundo_digito += Digits[i]
                    i += 1
                segundo_digito = int(''.join(segundo_digito))
        except ValueError:
            print("error when joining digits")
            return False
        # segundo_digito = ''
        # if Digits[2] == "-":
        #     i = -1
        #     while(Digits[i]!=Digits[2]):
        #         segundo_digito += Digits[i]
        #         i -= 1
        #     try:
        #         segundo_digito = int(''.join(reversed(segundo_digito)))
        #         segundo_digito *= -1
        #     except ValueError:
        #         print("second number's digit value error (if)")
        #         return False
        # else:
        #     i = -1
        #     while(Digits[i]!=Digits[2]):
        #         segundo_digito += Digits[i]
        #         i -= 1
        #     try:
        #         segundo_digito = int(''.join(reversed(segundo_digito)))
        #     except ValueError:
        #         print("second number's digit value error (else)")
        #         return False
        print(primeiro_digito)
        # print(primeiro_digito, end="%\n")

        if primeiro_digito >= 55 or primeiro_digito <= 45:
            print('\033[1;37;41m PERCENTUAL DE GAS FORA DE CONFORMIDADE \033[0;0m')
            # print('\a')
        else:
            print('\033[1;37;42m PERCENTUAL DE GAS DENTRO DOS CONFORMES \033[0;0m')

        # print("\n(Apos 30 segundos) Procedendo para leitura do ajuste de zero...\n")

        # print(segundo_digito, end="%\n")
        print(segundo_digito)
        if segundo_digito >= 5 or segundo_digito <= -5:
            print('\033[1;37;41m AJUSTE DE ZERO FORA DE CONFORMIDADE \033[0;0m')
            # print('\a')
        else:
            print('\033[1;37;42m AJUSTE DE ZERO DENTRO DOS CONFORMES \033[0;0m')

        
        #cv2.imshow("teste retangulos", image)
        # if segundo_digito == 101:
        #     cv2.waitKey(0)
        # else:
        cv2.waitKey(15)       
        return True 
        #Teste de imagem
    
    def getPoints(self, image):
	
        #Bloco 1

        x = 0
        y = 0
        
        while (x < np.shape(image)[1] and y < np.shape(image)[0]) and (image[5][x] < 250 and image[y][5] < 250):
            x += 1
            y += 1

        if x < np.shape(image)[1] and image[5][x] >= 250:
            x1 = x
            y1 = 5
        elif y < np.shape(image)[0] and image[y][5] >= 250 :
            x1 = 5
            y1 = y
        else:
            x1 = 0
            y1 = 0

        #Bloco 2

        x = 0
        y = np.shape(image)[0] - 1
        
        while (x < np.shape(image)[1] and y >= 0) and (image[np.shape(image)[0] - 5][x] < 250 and image[y][5] < 250):
            x += 1
            y -= 1

        if x < np.shape(image)[1] and image[np.shape(image)[0] - 5][x] >= 250:
            x2 = x
            y2 = np.shape(image)[0] - 5
        elif y >= 0 and image[y][5] >= 250 :
            x2 = 5
            y2 = y
        else:
            x2 = 0
            y2 = np.shape(image)[0] - 1



        #Bloco 3

        x = np.shape(image)[1] - 1
        y = 0
        
        while (x >= 0 and y < np.shape(image)[0]) and (image[5][x] < 250 and image[y][np.shape(image)[1] - 5] < 250):
            x -= 1
            y += 1

        if x >= 0 and image[5][x] >= 250:
            x3 = x
            y3 = 5
        elif y < np.shape(image)[0] and image[y][np.shape(image)[1] - 5] >= 250 :
            x3 = np.shape(image)[1] - 5
            y3 = y
        else:
            x3 = np.shape(image)[1] - 1
            y3 = 0


        # Bloco 4

        x = np.shape(image)[1] - 1
        y = np.shape(image)[0] - 1
        
        while (x >= 0 and y >= 0) and (image[np.shape(image)[0] - 5][x] < 250 and image[y][np.shape(image)[1] - 5] < 250):
            x -= 1
            y -= 1

        if x >= 0 and image[np.shape(image)[0] - 5][x] >= 250:
            x4 = x
            y4 = np.shape(image)[0] - 5
        elif y >= 0 and image[y][np.shape(image)[1] - 5] >= 250 :
            x4 = np.shape(image)[1] - 5
            y4 = y
        else:
            x4 = np.shape(image)[1] - 1
            y4 = np.shape(image)[0] - 1



        return [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]


    def orderPoints(self, pts):
        rect=np.zeros((4,2), dtype="float32")
        s= pts.sum(axis = 1)
        rect[0]=pts[np.argmin(s)]
        rect[2]=pts[np.argmax(s)]
        diff=np.diff(pts, axis = 1)
        rect[1]=pts[np.argmin(diff)]
        rect[3]=pts[np.argmax(diff)]
        return rect

        
    def four_point_transform(self, image, pts):
        rect=self.orderPoints(pts)
        (tl, tr, bl, br)=rect
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        absMax = max(maxHeight, maxWidth)
        dst = np.array([
            [0, 0],
            [absMax - 1, 0],
            [absMax - 1, absMax - 1],
            [0, absMax - 1]], dtype = "float32")
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (absMax, absMax))
        return warped

    def sorter(self, c):
        (x, y, w, h) = cv2.boundingRect(c)
        return x

    def checkOrientation(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (1, 1), 0)
        thresh = cv2.threshold(blurred, 0, 255,	cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 4))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        # thresh = cv2.dilate(thresh,kernel,iterations = 3)
        thresh = cv2.dilate(thresh,kernel,iterations = 1)
        #cv2.imshow("thresh_orientation", thresh)
        #cv2.waitKey(15)
        # cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = imutils.grab_contours(cnts)

        cima = False
        baixo = False
        esquerda = False
        direita = False
        for c in cnts:
            (x, y, w, h) = cv2.boundingRect(c)
            roi = thresh[y:y+h, x:x+w]
            
            area = cv2.contourArea(c)
            if (h)/(w) > 0.85 and h/w < 1.15 and area < 5*float(np.shape(image)[0]*np.shape(image)[1])/100:
                segROI = roi[0:h, 0:w]
                total = cv2.countNonZero(segROI)
                if float(area) > 0 and total/float(area) > 0.7:
                    cv2.rectangle(image, (x,y), (x+w,y+h), (0, 0, 255), 2)
                    if(x < np.shape(image)[1]/2):
                        esquerda = True
                    else:
                        direita = True
                    if(y < np.shape(image)[0]/2):
                        cima = True
                    else:
                        baixo = True
            else:
                continue

        if (cima and direita) and (baixo and direita):
            return True
        else:
            return False

class center_display:
    def __init__(self, mav, cv):
        self.cv = cv
        self.mav = mav
        self.frame = cv.cam_frame
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.velocity = Vector3()
        self.area_ratio = 0
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10)
   
    def is_there_a_display(self):
         self.display_center()
         if sum(sum(self.mask_preto)) > 0:
             print("MOSTRADOR DETECTADO")
             return True
         else:
             return False
    
    def display_center(self):
        self.frame = self.cv.cam_frame
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        lowerbpreto = np.array([0, 0, 0])
        upperbpreto = np.array([4, 4, 4])
        self.mask_preto = cv2.inRange(self.hsv, lowerbpreto, upperbpreto)
        kernel = np.ones((5,5), np.uint8)
        self.mask_preto = cv2.dilate(self.mask_preto ,kernel,iterations = 3)
        contours, hierarchy = cv2.findContours(self.mask_preto, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            print("mais de um contorno")
        for cnt in contours:
            M = cv2.moments(cnt)
            area = cv2.contourArea(cnt)
            self.area_ratio = (area/(np.shape(self.frame)[0]*np.shape(self.frame)[1]))
            try: 
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            except ZeroDivisionError:
                continue
            return cx, cy
        return -1,-1

    def centralize(self):
        self.display_center()
        #while self.area_ratio < 0.003:
        while not rospy.is_shutdown():
            x,y = self.display_center()
            if x == -1 or y == -1:
                continue
            erro_x = y - 480/2
            erro_y = x - 752/2
            if erro_x < 20 and erro_y < 20 and self.area_ratio < 0.002:
                self.velocity.x = 0
                self.velocity.y = 0 
                self.velocity.z = -1
            elif erro_x < 20 and erro_y < 20:
                if self.cv.main_loop():
                    self.velocity.x = 0
                    self.velocity.y = 0
                    self.velocity.z = 0
                    for j in range(20):
                        self.vel_publisher.publish(self.velocity)
                        self.cv.rate.sleep()
                    return
                #else:
                #    self.mav.set_position(0,0, hdg = 0.1, relative_to_drone=True)
            else:
                p = 0.01
                self.velocity.x= -erro_x * p
                self.velocity.y= -erro_y * p
                if self.velocity.x > 1:
                    self.velocity.x =1
                if self.velocity.x < -1:
                    self.velocity.x =-1
                if self.velocity.y > 1:
                    self.velocity.y =1
                if self.velocity.y < -1:
                    self.velocity.y =-1
                self.velocity.z = 0

            for j in range(20):
                self.vel_publisher.publish(self.velocity)
                self.cv.rate.sleep()

class trajectory:
    def __init__(self, mavbase, detector, display): #, display
        self.mav = mavbase
        self.detector = detector
        self.displayPresence = display
        self.lidar_sub = rospy.Subscriber("/uav1/garmin/range", Range, self.lidar_callback)

    def lidar_callback(self,data):
        self.lidar_range = data.range
    
    def time(self, t):
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.detector.rate.sleep()

    def go_to_fix(self, base):
        if base == "offshore1":
            #self.mav.altitude_estimator("BARO")
            self.mav.set_position(-19.10, -21.1, 4, hdg= 1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(-19.10, -21.1, 0.55)

        if base == "offshore2":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(-50, -21, 4, hdg=1.57)
            self.mav.set_position(-53.7, -35.2, 4, hdg=1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(-53.7, -35.2, 0.55)

        if base == "movel1":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(-30, 30, 4, hdg=1.57)
            self.mav.set_position(-30, 30, -6, hdg=1.57)

        if base == "movel2":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(60, 0, -6, hdg=1.57)

        if base == "movel3":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(35, -55, -6, hdg=1.57) #30 -55

    def mission_start(self):
        display_found = False

        if not display_found:
            rospy.loginfo("Indo para a primeira base movel")
            self.go_to_fix("movel1")
            self.time(1)
            if self.displayPresence.is_there_a_display():
                display_found = True
                self.time(5)
                #self.displayPresence.centralize()
                #while not (self.detector.main_loop()):
                #    self.mav.set_position(0, 0, 0, hdg=0.15, relative_to_drone=True)
        
        if not display_found:
            rospy.loginfo("Indo para a segunda base movel")
            self.go_to_fix("movel2")
            self.time(1)
            if self.displayPresence.is_there_a_display():
                display_found = True
                self.time(5)
                #self.displayPresence.centralize()
            #    while not (self.detector.main_loop()):
            #        self.mav.set_position(0, 0, 0, hdg=0.15, relative_to_drone=True)
        
        if not display_found:
            rospy.loginfo("Indo para a terceira base movel")
            self.go_to_fix("movel3")
            self.time(1)
            if self.displayPresence.is_there_a_display():
                display_found = True
                self.time(5)
                #self.displayPresence.centralize()
            #    while not (self.detector.main_loop()):
            #        self.mav.set_position(0, 0, 0, hdg=0.15, relative_to_drone=True)
            #    rospy.loginfo("aguardando os 5 segundos")
            #    self.time(5)

        rospy.loginfo("Indo para a primeira base do offshore")
        self.go_to_fix("offshore1")
        self.displayPresence.is_there_a_display()
        self.time(5)
        now = rospy.get_rostime()
        while not self.detector.main_loop():
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)):
               print("Desisto")
        rospy.loginfo("aguardando os 5 segundos")
        self.time(5)
        
        rospy.loginfo("Indo para a segunda base do offshore")
        self.go_to_fix("offshore2")
        self.displayPresence.is_there_a_display()
        self.time(5)
        now = rospy.get_rostime()
        while not self.detector.main_loop():
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)):
               print("Desisto")
        rospy.loginfo("aguardando os 5 segundos")
        self.time(5)

        rospy.loginfo("Missao concluida, retornando para a base costeira")
        self.mav.altitude_estimator("BARO")
        self.mav.set_position(10, 90, 4, hdg= 1.57)
        self.mav.altitude_estimator("HEIGHT")
        self.mav.set_position(10, 90, 1, hdg= 1.57)
        rospy.loginfo("Pousando")
        self.mav.land()
        while self.lidar_range > 0.25:
            pass
        self.mav.disarm()


if __name__ == "__main__":
    rospy.init_node("display_recognition")
    mav = MRS_MAV("uav1")
    detector = display_cv()
    display = center_display(mav, detector)
    controller = trajectory(mav, detector, display)
    controller.mission_start()
    #display.centralize()
    #controller.go_to_fix("movel3")
    #while True:
    #    print(display.is_there_a_display())
    #detector.main_loop()