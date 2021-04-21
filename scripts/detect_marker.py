from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='udp:127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

	print "Basic pre-arm checks"
	# Don't let the user try to arm until autopilot is ready
	while not vehicle.is_armable:
		print " Waiting for vehicle to initialise..."
		time.sleep(1)

	print "Arming motors"
	# Copter should arm in GUIDED mode
	vehicle.mode    = VehicleMode("GUIDED")
	vehicle.armed   = True

	while not vehicle.armed:
		print " Waiting for arming..."
		time.sleep(1)

	print "Taking off!"
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	  # Check that vehicle has reached takeoff altitude
	while True:
		print " Altitude: ", vehicle.location.global_relative_frame.alt 
		#Break and return from function just below target altitude.        
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
			print "Reached target altitude"
			break
		time.sleep(1)
class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test', anonymous=True, disable_signals=False) #Initialise rosnode
		#rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
		rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.image_callback)
		self.image = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.land = False
		self.recieved = False
		img_width = 400
		hfov_rad = 1.04  #horizontal field of view
		self.focal_length = (img_width/2)/math.tan(hfov_rad/2)

	def image_callback(self, data):
		#print("callback")
		try:
			self.image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8') # Converting the image to OpenCV standard image
			self.recieved = True
		except CvBridgeError as e:
			print(e)
			return

	def obstacle_segmentation(self):
		if self.recieved:
			img=self.image
			#image=cv2.resize(image,(400,400))
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			print(img[316:364,182:300,:])
			'''print(img.shape)
			print(img.dtype)
			hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
			#defining the range of red Colour
			red_lower=np.array([20,40,65],dtype=np.uint8)
			red_upper=np.array([40,50,85],dtype=np.uint8)
			#defining the range of yellow color
			yellow_lower=np.array([20,40,60],dtype=np.uint8)
			yellow_upper=np.array([40,50,85],dtype=np.uint8)
			#finding range for red and yellow colour
			red=cv2.inRange(img,red_lower,red_upper)
			yellow=cv2.inRange(img,yellow_lower,yellow_upper)

			kernal = np.ones((5,5), "uint8")
			red = cv2.dilate(red,kernal)
			res = cv2.bitwise_and(img, img, mask = red)
			yellow = cv2.dilate(yellow,kernal)
			res4 = cv2.bitwise_and(img, img, mask = yellow)

			#Tracking the Red Colour
			_, contours,hierarchy=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

			for pic, contour in enumerate(contours):
				area = cv2.contourArea(contour)
				if(area>300):

					x,y,w,h = cv2.boundingRect(contour)
					img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
					cv2.putText(img,"RED",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
			#Tracking the Yellow Colour
			_, contours,hierarchy=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

			for pic, contour in enumerate(contours):
				area = cv2.contourArea(contour)
				if(area>300):

					x,y,w,h = cv2.boundingRect(contour)
					img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
					cv2.putText(img,"Yellow",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))

			cv2.imshow('hsv format',hsv)
			cv2.imshow("Depth Image",img)
			cv2.waitKey(2)'''
	def detect_marker(self):

		# Our operations on the frame come here
		if self.recieved:
			image=self.image
			image=cv2.resize(image,(400,400))
			[r,c,h]=image.shape
			print(r,c,h)
			print(image.dtype)
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


			aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
			parameters =  aruco.DetectorParameters_create()
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
			#frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
			# verify *at least* one ArUco marker was detected
			if len(corners) > 0:
				# flatten the ArUco IDs list
				ids = ids.flatten()
				# loop over the detected ArUCo corners
				for (markerCorner, markerID) in zip(corners, ids):
					# extract the marker corners (which are always returned in
					# top-left, top-right, bottom-right, and bottom-left order)
					corners = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corners
					# convert each of the (x, y)-coordinate pairs to integers
					topRight = (int(topRight[0]), int(topRight[1]))
					bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
					bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
					topLeft = (int(topLeft[0]), int(topLeft[1]))
					# draw the bounding box of the ArUCo detection
					cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
					cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
					cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
					cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
					# compute and draw the center (x, y)-coordinates of the ArUco
					# marker
					cX = int((topLeft[0] + bottomRight[0]) / 2.0)
					cY = int((topLeft[1] + bottomRight[1]) / 2.0)
					print(cX,cY)
					err_x_m = ((cX-200)*(5.0)/self.focal_length) #calculating the x_error from drone to marker
					err_y_m = ((cY-200)*(5.0)/self.focal_length) #calculating the y_error from drone to marker
					print(err_x_m,err_y_m)
					cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
					# draw the ArUco marker ID on the image
					cv2.putText(image, str(markerID),
						(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (0, 255, 0), 2)
					print("[INFO] ArUco marker ID: {}".format(markerID))
					#if (int(markerID)==0):
					#	print("Now let's land")
					#	vehicle.mode = VehicleMode("LAND")
			cv2.imshow("Image", image)
			#cv2.imshow('frame_marker', frame_markers)#display the detected image
			cv2.waitKey(2)#wait for 2 millisecond before closing the output

if __name__ == '__main__':
	arm_and_takeoff(3)
	print("Set default/target airspeed to 3")
	vehicle.airspeed = 3
	#print("Going towards first point for 30 seconds ...")
	#point1 = LocationGlobalRelative(-35.3632585, 149.16510311637,8)
	#vehicle.simple_goto(point1)
	#time.sleep(30)
	#print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
	#point2 = LocationGlobalRelative(-35.3632585, 149.16510311637,3)
	#vehicle.simple_goto(point2)
	img = image_proc()
	r = rospy.Rate(5.5) # Rate at which the node runs
	while not rospy.is_shutdown():
		img.obstacle_segmentation()
		#img.detect_marker()
		r.sleep()
		if img.land:
			break