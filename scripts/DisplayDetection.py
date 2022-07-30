import cv2
import numpy as np
import mediapipe as mp
import easyocr
import time

class displayDetection:
    def __init__(self):

        self.cap = cv2.VideoCapture(0)
        self.squares = []
        self.reader = easyocr.Reader(['pt'])

    def find_squares(self, contours):

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h

                if aspectRatio >= 0.95 and aspectRatio < 1.05 and cv2.contourArea(contour) > 600:
                    self.squares.append(contour)
                    cv2.drawContours(self.image, [approx], 0, (255, 0, 0), 4)

    def crop_image(self, mask):

        x = np.where(mask > 0)[0]
        y = np.where(mask > 0)[1]
        x1 = np.min(x)
        x2 = np.max(x)
        y1 = np.min(y)
        y2 = np.max(y)

        image_copy = self.image.copy()
        self.cropped_image = image_copy[x1:x2, y1:y2]
        self.cropped_image1 = self.cropped_image[round(self.cropped_image.shape[0] * 0.03):round(self.cropped_image.shape[0] / 2),
                         round(self.cropped_image.shape[1] * 0.01):round(self.cropped_image.shape[1] * 0.65)]

        self.cropped_image2 = self.cropped_image[round(self.cropped_image.shape[0] / 2):round(self.cropped_image.shape[0]),
                         round(self.cropped_image.shape[1] * 0.07):round(self.cropped_image.shape[1] * 0.65)]

    def OCR(self, image):

        result = self.reader.readtext(image)
        return result

    def detection_loop(self):
        i = 0

        while self.cap.isOpened():
            self.squares = []
            success, self.image = self.cap.read()

            kernel = np.ones((5, 5), np.uint8)
            self.image = cv2.dilate(self.image, kernel, iterations = 2)
            self.image = cv2.erode(self.image, kernel)
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray, 200, 255, cv2.CHAIN_APPROX_NONE)

            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            self.find_squares(contours)

            if self.squares:


                sorted_squares = sorted(self.squares, key = cv2.contourArea, reverse = True )
                mask = np.zeros(self.image.shape, np.uint8)
                cv2.drawContours(mask, [sorted_squares[0]], 0, (0,0, 255), -1, )
                cv2.imshow("mask", mask)

                self.crop_image(mask)


                if(i < 10):

                    result1 = self.OCR(self.cropped_image1)
                    result2 = self.OCR(self.cropped_image2)

                    print(result1)
                    print(result2)
                    i = i + 1

            cv2.imshow("image", self.image)

            if cv2.waitKey(5) & 0xFF == 27:
                break

    def main_interface(self):
        #time.sleep(3)
        self.detection_loop()

detection = displayDetection()
detection.main_interface()