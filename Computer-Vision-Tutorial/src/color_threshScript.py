#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

# -------------------CALIBRAR   COLORES  ->  Jala bien
contador = 0

def Hminimo(val):
    pass


def Hmaximo(val):
    pass


def createTrackbars():
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("Hmax", "Trackbars", 0, 255, Hminimo)
    cv2.createTrackbar("Hmin", "Trackbars", 0, 255, Hmaximo)
    cv2.createTrackbar("Smax", "Trackbars", 0, 255, Hminimo)
    cv2.createTrackbar("Smin", "Trackbars", 0, 255, Hmaximo)
    cv2.createTrackbar("Vmax", "Trackbars", 0, 255, Hminimo)
    cv2.createTrackbar("Vmin", "Trackbars", 0, 255, Hmaximo)

def image_callback(data):
    global cam_w, cam_h, contador
    global frame, thresh

    if contador ==  0:
        createTrackbars()
        contador = 1

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    Bmax = cv2.getTrackbarPos("Hmax", "Trackbars")
    Gmax = cv2.getTrackbarPos("Smax", "Trackbars")
    Rmax = cv2.getTrackbarPos("Vmax", "Trackbars")

    Bmin = cv2.getTrackbarPos("Hmin", "Trackbars")
    Gmin = cv2.getTrackbarPos("Smin", "Trackbars")
    Rmin = cv2.getTrackbarPos("Vmin", "Trackbars")

    MaskLI = np.array([Bmin, Gmin, Rmin], np.uint8)
    MaskLS = np.array([Bmax, Gmax, Rmax], np.uint8)

    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frameHSV, MaskLI, MaskLS)

    frameFinal = cv2.bitwise_and(frame,frame,mask= mask)

    print("Max: ", Bmax, Gmax, Rmax)
    print("Min: ", Bmin, Gmin, Rmin)

    cv2.imshow('Mask Calibration Color', frameFinal)
    #cv2.imshow('Frame Calib', frame)
    # cv2.imshow('FrameHSV',frameHSV)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    #print ('receive msg')
    rospy.init_node('colorCalib_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()
