#!/usr/bin/env python
import cv2
import sys

print(sys.version)
print(cv2.__version__)
import numpy as np
import math
from datetime import datetime
import os
import rospy
from std_msgs.msg import String, Float64, Float32MultiArray
import time
from threading import Thread

DIM = (640, 480)
K = np.array(
    [[401.85180238132534, 0.0, 315.0881937529724], [0.0, 534.1449296513911, 267.5899112187299], [0.0, 0.0, 1.0]])
D = np.array([[-0.044447931423351454], [0.09001009612247628], [-0.017793512771069935], [-0.25484017856839847]])


# data = {'Color Range': ([30, 21, 10], [109, 255, 255])}
# lower_color, upper_color = data['Color Range']
# lower_color = np.array(lower_color)
# upper_color = np.array(upper_color)

def threaded(fn):
    def wrapper(*args, **kwargs):
        Thread(target=fn, args=args, kwargs=kwargs).start()

    return wrapper


def gstreamer_pipeline(
        capture_width=640,
        capture_height=480,
        display_width=640,
        display_height=480,
        framerate=24,
        flip_method=0,
):
    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )


def warp_image(img, pts1, width=640, height=480):
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgOutput = cv2.warpPerspective(img, matrix, (width, height))

    return imgOutput


def brightness_contrast(img, brightness=0):
    # getTrackbarPos returns the current
    # position of the specified trackbar.
    brightness = cv2.getTrackbarPos('Brightness', 'Image')

    contrast = cv2.getTrackbarPos('Contrast', 'Image')
    # print(brightness, contrast)

    effect = controller(img, brightness, contrast)

    # The function imshow displays an image
    # in the specified window
    # cv2.imshow('Effect', effect)
    return effect


def controller(img, brightness=255, contrast=127):
    brightness = int((brightness - 0) * (255 - (-255)) / (510 - 0) + (-255))

    contrast = int((contrast - 0) * (127 - (-127)) / (254 - 0) + (-127))
    # cv2.imshow('img', img)

    if brightness != 0:

        if brightness > 0:

            shadow = brightness

            max = 255

        else:

            shadow = 0
            max = 255 + brightness

        al_pha = (max - shadow) / 255.0
        ga_mma = shadow
        # print('ag',al_pha, ga_mma)
        # The function addWeighted calculates
        # the weighted sum of two arrays
        cal = cv2.addWeighted(img, al_pha,
                              img, 0, ga_mma)

    else:
        cal = img

    if contrast != 0:
        Alpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
        Gamma = 127 * (1 - Alpha)

        # The function addWeighted calculates
        # the weighted sum of two arrays
        cal = cv2.addWeighted(cal, Alpha,
                              cal, 0, Gamma)

        # putText renders the specified text string in the image.
    cv2.putText(cal, 'B:{},C:{}'.format(brightness,
                                        contrast), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return cal


cv2.namedWindow('Image')
# createTrackbar(trackbarName,
# windowName, value, count, onChange)
# Brightness range -255 to 255
cv2.createTrackbar('Brightness',
                   'Image', 170, 2 * 255,
                   brightness_contrast)

# Contrast range -127 to 127
cv2.createTrackbar('Contrast', 'Image',
                   240, 2 * 127,
                   brightness_contrast)

map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)


class DetectContour:
    def __init__(self):
        rospy.init_node('image_processing', anonymous=True)

        self.pub = rospy.Publisher('Error_msg', Float32MultiArray, queue_size=0)

        keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
        rospy.Subscriber(keyTopic, String, self.startCallback)

        # *ADD* create the subscriber to the keyboard node
        # keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
        # rospy.Subscriber(keyTopic, String, keyCallback)
        # gpu_frame = cv2.cuda_GpuMat()

        # rate = rospy.Rate(10)
        self.cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        self.warped_img_show = []

        self.prev_error_distance = 0
        self.prev_msg = Float32MultiArray(data=[0, 0])

    def startCallback(self,msg):
        if msg.data == "i":
            pass
        elif msg.data == "n":
            rospy.loginfo("prev_error_distance reset")
            self.prev_error_distance = 0

    @staticmethod
    def get_heading_error(line):

        # bottom point
        x1 = line[0]
        y1 = line[1]
        # Top point (centre)
        x2 = line[2]
        y2 = line[3]

        num = y2 - y1
        if num > 0:
            den = x2 - x1
        else:
            num = y1 - y2
            den = x1 - x2
        if den is not 0:
            slope = num / den
            heading = math.degrees(math.atan(slope))
            # if abs(heading)>90:
            if heading == 0:
                return 0
            sign = -heading / abs(heading)
            heading = abs(90 - (abs(heading))) * sign
        else:
            heading = 0

        # slope = (y2 - y1) / (x2 - x1)
        return heading

    # @threaded
    def process_frame(self, frame):
        start_time = time.time()
        height = frame.shape[0]
        width = frame.shape[1]

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = brightness_contrast(frame, 0)

        undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)
        pts1 = np.float32([[180, 180], [460, 180], [0, 480], [640, 480]])

        warped_img = warp_image(undistorted_img, pts1, 640, 480)

        # print dimensions of warped image frame
        # print(warped_img.shape[0], warped_img.shape[1])
        # bil_img = cv2.bilateralFilter(warped_img,9,75,75)
        # gray_image = cv2.cvtColor(warped_img, cv2.COLOR_RGB2GRAY)

        gauss_img = cv2.GaussianBlur(warped_img, (1, 1), 0)
        # median_img = cv2.medianBlur(warped_img,1)

        # hsv_img = cv2.cvtColor(gauss_img, cv2.COLOR_BGR2HSV)
        region_of_interest_vertices = [
            (0, 480),
            (width / 2, 0),
            (640, 480)]
        # mask = cv2.inRange(hsv_img, lower_color, upper_color)
        # cv2.imshow('A',mask)
        gray_image = cv2.cvtColor(gauss_img, cv2.COLOR_RGB2GRAY)

        # gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        # kernel = np.ones((15, 15), np.uint8)
        # thresh = cv2.erode(mask, kernel, iterations=2)
        # thresh = cv2.dilate(thresh, kernel, iterations=2)
        # canny_image = cv2.Canny(thresh, 100, 200)
        #
        # cropped_image = region_of_interest(thresh,
        #                                         np.array([region_of_interest_vertices], np.int32), )

        contours, hierarchy = cv2.findContours(gray_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # print(len(contours))
        # areas = [cv2.contourArea(c) for c in contours]
        # max_index = np.argmax(areas)
        # cnt = contours[max_index]
        if len(contours) > 0:
            # print("len  ",len(contours))

            c = max(contours, key=cv2.contourArea)
            # print("max contour ", c)
            x, y, w, h = cv2.boundingRect(c)

            rows, cols = gray_image.shape[:2]
            [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)

            lefty = int((-x * vy / (vx)) + y)
            righty = int(((cols - x) * vy / (vx)) + y)
            # print("line ", (cols - 1, righty), (0, lefty))
            line = np.array([0, lefty, cols - 1, righty])
            heading = self.get_heading_error(line.astype(np.float32))

            # img = cv2.line(warped_img, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
            # cv2.imshow('fitline', img)

            # cv2.rectangle(warped_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # cv2.drawContours(warped_img, c, -1, (0, 255, 0), thickness=1)

            M = cv2.moments(c)
            if M["m00"] == 0:
                return
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            error_distance = cX - 320
            # print("Error: %.2f, Heading: %.2f" % (error_distance, heading))

            # cv2.circle(warped_img, (cX, cY), 5, (36, 255, 12), -1)

            # To draw line you can use cv2.line or numpy slicing
            # cv2.line(warped_img, (x + int(w / 2), y), (x + int(w / 2), y + h), (0, 0, 255), 3)
            # image[int(cY - h/2):int(cY+h/2), cX] = (36, 255, 12)
            # cv2.line(warped_img, (320, cY), (cX, cY), (255, 0, 0), thickness=1)
            width1 = 200
            # [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            # lefty = int((-x * vy / vx) + y)  # n valuue in y = vy/vx*x + n y(x=0)
            # righty = int(((width1 - x) * vy / vx) + y)
            # img = cv2.line(warped_img, (width1, righty), (0, lefty), (255, 0, 0), thickness=1)

            if abs(self.prev_error_distance - error_distance) > 320:
                rospy.loginfo("Max Limit in detection")
                error_distance = self.prev_error_distance
            self.prev_error_distance = error_distance

            msg = Float32MultiArray(data=[error_distance, heading])

            self.prev_msg = msg

            self.pub.publish(msg)

            # rospy.loginfo(cX-320)

        else:
            self.pub.publish(self.prev_msg)
            rospy.loginfo("No line detected")
            # self.pub.publish(1234)

        end_time = time.time()
        # print('duration1',(end_time-start_time)*1000)

        self.warped_img_show.append(warped_img)

        # cv2.imshow('Image', warped_img)
        # cv2.imshow('undistorted', frame)

        # cv2.imshow('Crop Image', cropped_image)
        # cv2.imshow('Gray Image', gray_image)

    def run(self):
        prev_time = time.time()
        try:

            while not rospy.is_shutdown():

                current_time = time.time()
                # print((current_time - prev_time) * 1000, "img new statr")
                prev_time = current_time

                if not self.cam.isOpened():
                    print("camera not ready")
                    continue

                # print(cam.isOpened())
                ret, frame = self.cam.read()
                # gpu_frame.upload(frame)
                # cv2.waitKey(1)

                self.process_frame(frame)

                for image in self.warped_img_show:
                    cv2.imshow('Image', self.warped_img_show.pop(0))

                if (cv2.waitKey(1) & 0xFF == 27):
                    break
        except KeyboardInterrupt:
            print("keyborad interrupt")

        finally:
            self.cam.release()
            print('cam_release')
            cv2.destroyAllWindows()


if __name__ == '__main__':
    imag_processing = DetectContour()
    imag_processing.run()
