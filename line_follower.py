import time
import cv2 as cv
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from threading import Timer, Thread
from geometry_msgs.msg import Twist

DRAW_CONTOUR_FREQUENCY_HZ = 5
DRAW_SIGN_FREQUENCY_HZ = 5
IMAGE_LEFT_THRESHOLD = 200
IMAGE_RIGHT_THRESHOLD = 440

class Follower:
	def __init__(self):
		self.latest_image = None
		self.latest_image_cropped = None
		self.line_coordinates = None

		self.driver_thread = Thread(target=self.drive_robot)
		self.line_contour_thread = Thread(target=self.find_line_contour)
		self.sign_contour_thread = Thread(target=self.find_sign_contour)

		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

		cv.namedWindow("camera_window", cv.WINDOW_NORMAL)
		cv.namedWindow("robot_vision_window", cv.WINDOW_NORMAL)
		cv.namedWindow("sign_vision_window", cv.WINDOW_NORMAL)
		
		# Start a timer to periodically draw the contour
		self.line_contour_thread.start()
		# self.driver_thread.start()
		rospy.spin()

	def drive_robot(self):
		'''Drive the robot based on the line coordinates'''
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rate = rospy.Rate(20)

		straight_twist = Twist()
		straight_twist.linear.x = 0.2

		left_twist = Twist()
		left_twist.linear.x = 0.1
		left_twist.angular.z = 0.2

		right_twist = Twist()
		right_twist.linear.x = 0.1
		right_twist.angular.z = -0.2

		stop_twist = Twist()

		self.__wait_for_image_initialization()
		while not rospy.is_shutdown():
			if self.line_coordinates is not None:
				x, y = self.line_coordinates
				if x < IMAGE_LEFT_THRESHOLD:
					print("turn left")
					pub.publish(left_twist)
				elif x > IMAGE_RIGHT_THRESHOLD:
					print("turn right")
					pub.publish(right_twist)
				else:
					print("go straight")
					pub.publish(straight_twist)
			rate.sleep()

	def __wait_for_image_initialization(self):
		while self.latest_image_cropped is None:
			time.sleep(.01)

	def __find_largest_line_contour(self, image):
		# convert to grayscale
		gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
		# apply gaussian blur
		blur = cv.GaussianBlur(gray, (5, 5), 0)
		# color thresholding
		# ret, thresh = cv.threshold(blur, 120, 255, cv.THRESH_BINARY)
		mask = cv.inRange(blur, 115, 130) # New threshold for yellow line on new map
		# find contours
		image, contours, heirarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

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
		self.latest_image_cropped = self.crop_image(image)
		cv.imshow("camera_window", self.latest_image)
		cv.waitKey(3)
	
	def find_line_contour(self):
		'''Draw the contour of the line'''
		while not rospy.is_shutdown():
			self.__wait_for_image_initialization()
			contour, contour_x, contour_y = self.__find_largest_line_contour(self.latest_image_cropped)
			print(contour_x, contour_y)
			self.line_coordinates = (contour_x, contour_y)
			if contour is not None:
				self.draw_robot_perception(self.latest_image, contour, contour_x, contour_y)
			time.sleep(1.0 / DRAW_CONTOUR_FREQUENCY_HZ)

	def find_sign_contour(self):
		'''Draw the contour of the sign'''
		while not rospy.is_shutdown():
			self.__wait_for_image_initialization()
			x, y, w, h = self.__find_sign_contour(self.latest_image_cropped)
			if x is not None:
				self.draw_sign_detection(self.latest_image_cropped, x, y, w, h)
			time.sleep(1.0 / DRAW_SIGN_FREQUENCY_HZ)

	def __find_sign_contour(self, image):
		hsv = cv.cvtColor(self.latest_image, cv.COLOR_BGR2HSV)
		lower_red = np.array([0, 100, 100])
		upper_red = np.array([10, 255, 255])
		mask = cv.inRange(hsv, lower_red, upper_red)

		image, contours, heirarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

		if len(contours) > 0:
			red_area = max(contours, key=cv.contourArea)
			x, y, w, h = cv.boundingRect(red_area)
			return x, y, w, h
		else:
			return None, None, None, None

	def draw_sign_detection(self, image, x, y, w, h):
		'''Draw the sign detection'''
		cv.boundingRect(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
		cv.imshow("sign_vision_window", image)
		cv.waitKey(3)

	def draw_robot_perception(self, image, contour, contour_x, contour_y):
		contour_img = self.latest_image_cropped.copy()
		# draw contour lines
		cv.circle(contour_img, (contour_x, contour_y), 5, (0, 0, 255), -1)
		# draw vertical line
		cv.line(contour_img, (IMAGE_LEFT_THRESHOLD, 0), (IMAGE_LEFT_THRESHOLD, 480), (255, 0, 0), 5)
		cv.line(contour_img, (IMAGE_RIGHT_THRESHOLD, 0), (IMAGE_RIGHT_THRESHOLD, 480), (255, 0, 0), 5)

		cv.drawContours(contour_img, contour, -1, (0, 255, 0), 1)
		cv.imshow("robot_vision_window", contour_img)
		cv.waitKey(3)

	def crop_image(self, image):
		'''Crop image to only show the bottom half of the image'''
		height, width, channels = image.shape
		return image[int(height/2):height, 0:width]

if __name__ == "__main__":
	rospy.init_node('line_follower')
	follower = Follower()

