import numpy as np
import cv2


frame = None
roiPts = []
inputMode = False
xPts = []
yPts = []
def selectROI(event, x, y, flags, param):
    global frame, roiPts, inputMode, yPts, xPts
    if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts) < 4:
		roiPts.append((x, y))
		xPts.append(x)
		yPts.append(y)
		cv2.circle(frame, (x, y), 4, (255, 204, 0), 2)
		cv2.imshow("frame", frame)
		print roiPts
		if len(roiPts) == 4:
			cv2.circle(frame, ((sum(xPts) / 4), (sum(yPts) / 4)), 1, (255, 0, 0), 2)
			cv2.imshow("frame", frame)


def main():
	global frame, roiPts, inputMode
	camera = cv2.VideoCapture(0)

	cv2.namedWindow("frame")
	#cv2.namedWindow("HSV")
	#cv2.namedWindow("Back Projection")
	cv2.setMouseCallback("frame", selectROI)
	termination = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0)
	roiBox = None
	while True:
		(grabbed, frame) = camera.read()
		if not grabbed:
			break
		if roiBox is not None:
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			#cv2.imshow("HSV", hsv)
			backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)
			#cv2.imshow("Back Projection", backProj)

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
				cv2.circle(frame, cPt, 1, (255, 0, 0), 2)
				cv2.putText(frame, 'target', (cPtx, (cPty - 7)), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
				cv2.putText(frame, str(cPt), (7, 25), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
				cv2.putText(frame, str(boxArea), (7, 55), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

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
			# roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
			mask = cv2.inRange(roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
			roiHist = cv2.calcHist([roi], [0], mask, [180], [0, 180])
			roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)

			roiBox = (tl[0], tl[1], br[0], br[1])
			print roiBox
		# choose new target without resetting
		elif key == ord("r"):
			frame = None
			roiPts = []
			inputMode = False
			xPts = []
			yPts = []
		elif key == ord("q"):
			break

	camera.release()
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()