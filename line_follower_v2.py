import time
import cv2 as cv
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from threading import Timer

DRAW_CONTOUR_FREQUENCY_HZ = 5

class Follower:
	def __init__(self):
		self.latest_image = None
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

		cv.namedWindow("camera_window", cv.WINDOW_NORMAL)
		cv.namedWindow("contour_window", cv.WINDOW_NORMAL)
		
		Timer(1.0 / DRAW_CONTOUR_FREQUENCY_HZ, self.draw_contour).start()
		rospy.spin()
	
	def __wait_for_image_initialization(self):
		while self.latest_image is None:
			time.sleep(.01)

	def find_largest_line_contour(self, image):
		# convert to grayscale
		gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
		# apply gaussian blur
		blur = cv.GaussianBlur(gray, (5, 5), 0)
		# color thresholding
		ret, thresh = cv.threshold(blur, 60, 255, cv.THRESH_BINARY)
		# find contours
		image, contours, heirarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

		if len(contours) > 0:
			largest_contour = max(contours, key=cv.contourArea)
			moments = cv.moments(largest_contour)

			x = int(moments['m10'] / moments['m00'])
			y = int(moments['m01'] / moments['m00'])

			return largest_contour, x, y
		else:
			return None, None, None


	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		self.latest_image = image
		cv.imshow("camera_window", image)
		cv.waitKey(3)
	
	def draw_contour(self):
		self.__wait_for_image_initialization()
		contour, contour_x, contour_y = self.find_largest_line_contour(self.latest_image)
		print(contour)
		if contour is not None:
			contour_img = self.latest_image.copy()
			cv.drawContours(contour_img, contour, -1, (0, 255, 0), 3)
			cv.imshow("contour_window", contour_img)
			cv.waitKey(3)
			Timer(1.0 / DRAW_CONTOUR_FREQUENCY_HZ, self.draw_contour).start()

if __name__ == "__main__":
	rospy.init_node('follower')
	follower = Follower()

