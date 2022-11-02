import time
import cv2 as cv
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image


class Follower:
	def __init__(self):
		self.low = np.array([0, 80, 0])
		self.high = np.array([80, 255, 80])

		self.latest_image = None
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

		cv.namedWindow("camera_window", cv.WINDOW_NORMAL)
		cv.namedWindow("segment_window", cv.WINDOW_NORMAL)
		rospy.spin()
	
	def __wait_for_image_initialization(self):
		while self.latest_image is None:
			time.sleep(.01)

	def __color_mask(self, image):
		image = image.astype(np.float32)
		mask = cv.inRange(image, self.low, self.high)
		result = cv.bitwise_and(image, image, mask=mask)
		result = result.astype(np.uint8)
		return result


	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		self.latest_image = image
		cv.imshow("camera_window", image)
		cv.imshow("segment_window", self.__color_mask(image))
		cv.waitKey(3)

if __name__ == "__main__":
	rospy.init_node('follower')
	follower = Follower()

