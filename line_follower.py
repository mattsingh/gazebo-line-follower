import time
import cv2 as cv
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from threading import Timer
from geometry_msgs.msg import Twist

DRAW_CONTOUR_FREQUENCY_HZ = 5

class Follower:
	def __init__(self):
		self.latest_image = None
		self.line_coordinates = None
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

		cv.namedWindow("camera_window", cv.WINDOW_NORMAL)
		cv.namedWindow("contour_window", cv.WINDOW_NORMAL)
		
		# Start a timer to periodically draw the contour
		Timer(1.0 / DRAW_CONTOUR_FREQUENCY_HZ, self.draw_contour).start()
		self.drive_robot()
		rospy.spin()

	def drive_robot(self):
		'''Drive the robot based on the line coordinates'''
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rate = rospy.Rate(10)

		straight_twist = Twist()
		straight_twist.linear.x = 0.2

		left_twist = Twist()
		left_twist.linear.x = 0.2
		left_twist.angular.z = 0.2

		right_twist = Twist()
		right_twist.linear.x = 0.2
		right_twist.angular.z = -0.2

		self.__wait_for_image_initialization()
		while not rospy.is_shutdown():
			if self.line_coordinates is not None:
				x, y = self.line_coordinates
				if x < 200:
					print("turn left")
					pub.publish(left_twist)
				elif x > 440:
					print("turn right")
					pub.publish(right_twist)
				else:
					print("go straight")
					pub.publish(straight_twist)

			rate.sleep()

	def __wait_for_image_initialization(self):
		while self.latest_image is None:
			time.sleep(.01)

	def __find_largest_line_contour(self, image):
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
		self.latest_image = self.crop_image(image)
		cv.imshow("camera_window", self.latest_image)
		cv.waitKey(3)
	
	def draw_contour(self):
		self.__wait_for_image_initialization()
		contour, contour_x, contour_y = self.__find_largest_line_contour(self.latest_image)
		self.line_coordinates = (contour_x, contour_y)
		print(contour_x, contour_y)
		if contour is not None:
			contour_img = self.latest_image.copy()
			# draw contour lines
			cv.line(contour_img, (contour_x, 0), (contour_x, 480), (255, 0, 0), 3)
			cv.line(contour_img, (0, contour_y), (640, contour_y), (255, 0, 0), 3)
			cv.drawContours(contour_img, contour, -1, (0, 255, 0), 1)
			cv.imshow("contour_window", contour_img)
			cv.waitKey(3)
			Timer(1.0 / DRAW_CONTOUR_FREQUENCY_HZ, self.draw_contour).start()

	def crop_image(self, image):
		'''Crop image to only show the bottom half of the image'''
		height, width, channels = image.shape
		return image[int(height/2):height, 0:width]

if __name__ == "__main__":
	rospy.init_node('line_follower')
	follower = Follower()

