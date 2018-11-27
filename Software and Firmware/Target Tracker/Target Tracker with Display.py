## IMPORTS ----------------------------------------------------------------------------------------------------------------------------------------------

import RPi.GPIO as GPIO
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import numpy as np
import time
import serial
import imutils
import cv2
import sys

## FUNCTIONS --------------------------------------------------------------------------------------------------------------------------------------------

def nothing(x):
	pass

def writePositions(cx, cy, targetAquired):

	cx_copy = cx
	cy_copy = cy
	
	xhigh = ((cx_copy >> 8) & 0xFF)
	xlow = (cx & 0xFF)

	yhigh = ((cy_copy >> 8) & 0xFF)
	ylow = (cy & 0xFF)

	GPIO.output(31,GPIO.HIGH) #Casues EXTI interrup on PA4 on STM32

	while GPIO.input(37) == 1: #wait for STM32 of acknowledge data is ready to be sent: STM32 lowers PB2
		nothing
		if ((GPIO.input(33) == 0) or (GPIO.input(35) == 1)):
				break
	
	if targetAquired == 1:
		targetPos = bytearray([xhigh, xlow, yhigh, ylow])
		ser.write(targetPos)
		print('cx = {0}, cy = {1}, xhigh = {2}, xlow = {3}, yhigh = {4}, ylow = {5}'.format(cx, cy, xhigh, xlow, yhigh, ylow))
	else:
		targetPos = bytearray([((0 >> 8) & 0xFF), (0 & 0xFF), ((0 >> 8) & 0xFF), (0 & 0xFF)])
		ser.write(targetPos)
		print('Target Lost')

	GPIO.output(31,GPIO.LOW)

## INITIALISATIONS --------------------------------------------------------------------------------------------------------------------------------------

#IMAGE PROCESSING KERNELS
kernel = np.ones((5,5),np.uint8)
ekernel = np.ones((2,2),np.uint8)
dkernel = np.ones((8,8),np.uint8)

#DISPLAY FRAME
cv2.namedWindow('Image')
# Trackbars
cv2.createTrackbar('H_Min','Image',50,179,nothing)
cv2.createTrackbar('H_Max','Image',120,179,nothing)
cv2.createTrackbar('S_Min','Image',150,255,nothing)
cv2.createTrackbar('S_Max','Image',255,255,nothing)
cv2.createTrackbar('V_Min','Image',190,255,nothing)
cv2.createTrackbar('V_Max','Image',255,255,nothing)
cv2.createTrackbar('Calibrated?','Image',0,1,nothing)

lower_bright = np.array([cv2.getTrackbarPos('H_Min','Image'),cv2.getTrackbarPos('S_Min','Image'),cv2.getTrackbarPos('V_Min','Image')])
upper_bright = np.array([cv2.getTrackbarPos('H_Max','Image'),cv2.getTrackbarPos('S_Max','Image'),cv2.getTrackbarPos('V_Max','Image')])

calibrationMode = cv2.getTrackbarPos('Calibrated?','Image')

#SERIAL COMMS SETUP
ser = serial.Serial(port='/dev/ttyS0', baudrate = 230400, parity = serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

#GPIO SETUP
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37,GPIO.IN)
GPIO.setup(35,GPIO.IN)
GPIO.setup(33,GPIO.IN)
GPIO.setup(31,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.IN)

GPIO.output(11,GPIO.HIGH)
GPIO.output(31,GPIO.LOW)

cx = 0
cy = 0
state = 0
fps_run = 0
targetAquired = 0

## MAIN -------------------------------------------------------------------------------------------------------------------------------------------------

# created a *threaded* video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src=0, width=640, height=480).startgrab()

while calibrationMode == 0:
        im = vs.readframe()
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_bright, upper_bright)
        #blur = cv2.GaussianBlur(mask, (1, 1), 1)

        cv2.imshow('Image',mask)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
                break

        lower_bright = np.array([cv2.getTrackbarPos('H_Min','Image'),cv2.getTrackbarPos('S_Min','Image'),cv2.getTrackbarPos('V_Min','Image')])
        upper_bright = np.array([cv2.getTrackbarPos('H_Max','Image'),cv2.getTrackbarPos('S_Max','Image'),cv2.getTrackbarPos('V_Max','Image')])
        calibrationMode = cv2.getTrackbarPos('Calibrated?','Image')

cv2.destroyAllWindows()
#vs.startdisplay(0, 0)        

while (1): 
	
		if (GPIO.input(33) == 0): 
			if state == 1:
				fps.stop()
				print('\nIm process FPS = {}'.format(fps.fps()))
				state = 0
				
			# grab the frame 
			im = vs.readframe()
			
			im2 = im.copy()
			
			# Convert BGR to HSV
			hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

			#if calibratationMode == 0:

			# Threshold the HSV image to get only bright colors
			mask = cv2.inRange(hsv, lower_bright, upper_bright)

			#blur = cv2.GaussianBlur(mask, (3, 3), 1)

			eroded = cv2.erode(mask,ekernel,iterations = 1)
			#dilation = cv2.dilate(eroded,dkernel,iterations = 1)
			#morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

			contours = cv2.findContours(eroded,1,2)[-2]

			if len(contours) > 0:
					c = max(contours,key=cv2.contourArea)
					M = cv2.moments(c)
					#print M

					if M['m00'] != 0:
						cx = int(M['m10']/M['m00'])
						cy = int(M['m01']/M['m00'])
						cv2.circle(im,(cx,cy),5,(0,255,0), -1)   
						print("Target Aquired")

			cv2.imshow('frame',im)
			#cv2.imshow('Mask',mask)
			#cv2.imshow('eroded',dilation)

                
		else:
			if state == 0:
				fps = FPS().start()
				state = 1
				fps_run = 1
				
			# grab the frame 
			im = vs.readframe()
			
			im2 = im.copy()
			
			# Convert BGR to HSV
			hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

			#if calibratationMode == 0:

			# Threshold the HSV image to get only bright colors
			mask = cv2.inRange(hsv, lower_bright, upper_bright)

			#blur = cv2.GaussianBlur(mask, (3, 3), 1)

			eroded = cv2.erode(mask,ekernel,iterations = 1)
			#dilation = cv2.dilate(eroded,dkernel,iterations = 1)
			#morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

			contours = cv2.findContours(eroded,1,2)[-2]

			if len(contours) > 0:
					c = max(contours,key=cv2.contourArea)
					M = cv2.moments(c)
					#print M

					if M['m00'] != 0:
							cx = int(M['m10']/M['m00'])
							cy = int(M['m01']/M['m00'])
							targetAquired = 1
							cv2.circle(im,(cx,cy),5,(0,255,0), -1)   
			else:
				targetAquired = 0


			writePositions(cx, cy, targetAquired)
			
			cv2.imshow('frame',im)
			#cv2.imshow('Mask',mask)
			#cv2.imshow('eroded',dilation)
	 
			# update the FPS counter
			fps.update()
								
		k = cv2.waitKey(1) & 0xFF
		if ((k == 27) or (GPIO.input(35) == 1)):
				break
				

# stop the timer and display FPS information
if fps_run == 1:
	fps.stop()
	print('\nIm process FPS = {}'.format(fps.fps()))
	
cv2.destroyAllWindows()
GPIO.output(11,GPIO.LOW)
time.sleep(0.5)
GPIO.output(11,GPIO.HIGH)
 
## END --------------------------------------------------------------------------------------------------------------------------------------------------
#CLOSE THREADS
vs.stop()

