import numpy as np
import cv2

frame = None
roiPts = []
iLowH = 0;
iHighH = 179;
iLowS = 0; 
iHighS = 255;
iLowV = 0;
iHighV = 255;


def main():
	global frame, roiPts, inputMode
	
	camera = cv2.VideoCapture(0)
	width, height = 600 , 480
	camera.set(3, width)
	camera.set(4, height)
	

	cv2.namedWindow("frame")
	
  
 	cv2.cv.CreateTrackbar("LowH", "frame", iLowH, 179, main);
 	cv2.cv.CreateTrackbar("HighH", "frame", iHighH, 179, main);

  	cv2.cv.CreateTrackbar("LowS", "frame", iLowS, 255, main); 
 	cv2.cv.CreateTrackbar("HighS", "frame", iHighS, 255, main);

  	cv2.cv.CreateTrackbar("LowV", "frame", iLowV, 255, main);
 	cv2.cv.CreateTrackbar("HighV", "frame", iHighV, 255, main);
	
	termination = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0)
	roiBox = None
	while True:
		(grabbed, frame) = camera.read()
		if not grabbed:
			break
		if roiBox is not None:
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			
			backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)
			

			(r, roiBox) = cv2.meanShift(backProj, roiBox, termination)
			if not r:
				print "error"
				cv2.putText(frame, "Object Out of Frame!", (7, 25), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
			else:
				(r, roiBox) = cv2.CamShift(backProj, roiBox, termination)
				pts = np.int0(cv2.cv.BoxPoints(r))
			# print pts
				cPtx = (pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) / 4
				cPty = (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) / 4
				cPt = (cPtx, cPty)
				boxArea = .5 * (
				(pts[0][0] * pts[1][1]) + (pts[1][0] * pts[2][1]) + (pts[2][0] * pts[3][1]) + (pts[3][0] * pts[0][1]) - (
				pts[1][0] * pts[0][1]) - (pts[2][0] * pts[1][1]) - (pts[3][0] * pts[2][1]) - (pts[0][0] * pts[3][1]))
				print cPt
				cv2.polylines(frame, [pts], True, (255, 204, 0), 2)
				#cv2.circle(frame, cPt, 1, (255, 0, 0), 2)
				#cv2.putText(frame, 'target', (cPtx, (cPty - 7)), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
				#cv2.putText(frame, str(cPt), (7, 25), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
				#cv2.putText(frame, str(boxArea), (7, 55), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

				#print camera.get(3)
				#print camera.get(4)

				#print xPid.GenOut(cPtx-camera.get(3)/2)
				#print yPid.GenOut(cPty-camera.get(4)/2)
		cv2.imshow("frame", frame)
		key = cv2.waitKey(1) & 0xFF

		if key == 32 and len(roiPts) < 4:
			inputMode = True
			orig = frame.copy()

			while len(roiPts) < 4:
				cv2.imshow("frame", frame)
				cv2.waitKey(0)

			roiPts = np.array(roiPts)
			s = roiPts.sum(axis=1)
			tl = roiPts[np.argmin(s)]
			br = roiPts[np.argmax(s)]
			print roiPts

			roi = orig[tl[1]:br[1], tl[0]:br[0]]
			roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

			roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
			roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)

			roiBox = (tl[0], tl[1], br[0], br[1])
			print roiBox
		
		
		elif key == ord("q"):
			break


	camera.release()
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()