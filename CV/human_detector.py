from __future__ import print_function
import rospy
import numpy as np
import argparse
import imutils
import cv2
from imutils.object_detection import non_max_suppression
from imutils import paths
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

class HumanDetector:
    def __init__(self):
        rospy.init_node('human_detector')
        
        self.image_sub = rospy.Subscriber('/raspicam_node/compressed', CompressedImage, callback)
        self.detected_pub = rospy.Publisher('/humans_detected', Bool)
        
        rospy.spin()

    def callback(data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image = imutils.resize(image, width=min(400, image.shape[1]))
        orig = image.copy()

        # detect people in the image
        (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
            padding=(8, 8), scale=1.05)

        # draw the original bounding boxes
        for (x, y, w, h) in rects:
            cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        # draw the final bounding boxes
        # TODO (rwales): Do we want to draw the bounding boxes on the image and stream it to the user?
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

        if len(pick) > 0:
            detected = Bool()
            detected.data = True
            self.detected_pub.publish(detected)

if __name__ == '__main__':
    hd = HumanDetector()    