#import the necessary packages
from threading import Thread
from imutils.video import FPS
import cv2

class WebcamVideoStream:
	def __init__(self, src=0, width=640, height=480):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv2.VideoCapture(src)
		self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
		(self.grabbed, self.frame) = self.stream.read()

		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False
		self.closeDisplay = False

	def startgrab(self):
		# start the thread to read frames from the video stream
		t = Thread(target=self.update, args=())
		t.start()
		return self
	
	def startdisplay(self, cx, cy):
		# start the thread to read frames from the video stream
		s = Thread(target=self.display, args=(cx, cy))
		s.start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		self.fps = FPS().start()
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				self.stream.release()
				# stop the timer and display FPS information
				self.fps.stop()
				print('\nIm grab FPS = {}'.format(self.fps.fps()))
				self.closeDisplay = True
				return

			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()
			self.fps.update()

	def display(self, cx, cy):
		# keep looping infinitely until the thread is stopped
		while self.closeDisplay == False:                    
			im = WebcamVideoStream.readframe(self)
			cv2.circle(im,(cx,cy),10,(0,255,0), -1)
			cv2.imshow('Stream',im)
			cv2.waitKey(1)
			
		cv2.destroyAllWindows() 

	def readframe(self):
		# return the frame most recently read
		return self.frame

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
