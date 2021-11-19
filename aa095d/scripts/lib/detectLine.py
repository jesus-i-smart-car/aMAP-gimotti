import cv2
import numpy as np
import math


class RoadLineDetector:
    def __init__(self, img):
        self.img = img
        self.processedImg = img
        self.leftFitLine = None
        self.rightFitLine = None

    def grayScale(self):
        self.processedImg = cv2.cvtColor(self.processedImg, cv2.COLOR_RGB2GRAY)
        return self.processedImg

    def canny(self, lowThreshold, highThreshold):
        self.processedImg = cv2.Canny(
            self.processedImg, lowThreshold, highThreshold)
        return self.processedImg

    def gaussianBlur(self, kernelSize):
        self.processedImg = cv2.GaussianBlur(
            self.processedImg, (kernelSize, kernelSize), 0)
        return self.processedImg

    def regionOfInterest(self, vertices, color3=(255, 255, 255), color1=255):
        mask = np.zeros_like(self.processedImg)
        if len(self.processedImg.shape) > 2:
            color = color3
        else:
            color = color1
            cv2.fillPoly(mask, vertices, color)
        self.processedImg = cv2.bitwise_and(self.processedImg, mask)

    def drawLine(self, lines, color=[255, 0, 0], thickness=2):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(self.processedImg, (x1, y1),
                         (x2, y2), color, thickness)

    def drawFitLine(self, img, lines, color=[255, 0, 0], thickness=10):
        cv2.line(img, (lines[0], lines[1]),
                 (lines[2], lines[3]), color, thickness)

    def houghLines(self, rho, theta, threshold, minLineLen, maxLineGap):
        lines = cv2.HoughLinesP(self.processedImg, rho, theta, threshold, np.array(
            []), minLineLength=minLineLen, maxLineGap=maxLineGap)
        return lines

    def getFitLine(self, fLines):
        lines = np.squeeze(fLines)
        lines = lines.reshape(lines.shape[0] * 2, 2)
        rows, cols = self.img.shape[:2]
        output = cv2.fitLine(lines, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x, y = output[0], output[1], output[2], output[3]
        x1, y1 = int(((self.img.shape[0] - 1) - y) /
                     vy * vx + x), self.img.shape[0] - 1
        x2, y2 = int(((self.img.shape[0] / 2 + 100) - y) /
                     vy * vx + x), int(self.img.shape[0] / 2 + 100)
        result = [x1, y1, x2, y2]
        return result

    def weightedImg(self, img, alpha=1, beta=1., gamma=0.):
        self.processedImg = cv2.addWeighted(self.img, alpha, img, beta, gamma)

    @staticmethod
    def process(image):
        rld = RoadLineDetector(image)
        h, w = image.shape[:2]
        rld.grayScale()
        rld.gaussianBlur(3)
        rld.canny(70, 210)
        vertices = np.array([[(50, h), (w / 2 - 45, h / 2 + 60),
                            (w / 2 + 45, h / 2 + 60), (w - 50, h)]], dtype=np.int32)
        rld.regionOfInterest(vertices, vertices)
        lineArr = rld.houghLines(1, 1 * np.pi / 180, 30, 10, 20)
        lineArr = np.squeeze(lineArr)
        slopeDegree = (np.arctan2(
            lineArr[:, 1] - lineArr[:, 3], lineArr[:, 0] - lineArr[:, 2]) * 180) / np.pi
        lineArr = lineArr[np.abs(slopeDegree) < 160]
        slopeDegree = slopeDegree[np.abs(slopeDegree) < 160]
        lineArr = lineArr[np.abs(slopeDegree) > 95]
        slopeDegree = slopeDegree[np.abs(slopeDegree) > 95]
        lLines, rLines = lineArr[(slopeDegree > 0),
                                 :], lineArr[(slopeDegree < 0), :]
        temp = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        lLines, rLines = lLines[:, None], rLines[:, None]
        if len(lLines) < 1 or len(rLines) < 1:
            raise IndexError
        rld.leftFitLine = rld.getFitLine(lLines)
        rld.rightFitLine = rld.getFitLine(rLines)
        rld.drawFitLine(temp, rld.leftFitLine)
        rld.drawFitLine(temp, rld.rightFitLine)
        rld.weightedImg(temp)
        return (rld.processedImg, rld.leftFitLine, rld.rightFitLine)

def runDetector(vidStream):
    cap = cv2.VideoCapture(vidStream, cv2.CAP_GSTREAMER)

    while cap.isOpened:
        ret, frame = cap.read()
        if ret:
            try:
                linedImg, leftFitLine, rightFitLine = RoadLineDetector.process(frame)
                cv2.imshow('Yee ~', linedImg)
                print('Left lane: ' + str(leftFitLine))
                print('Right lane: ' + str(rightFitLine))
            except (IndexError, ValueError):
               cv2.imshow('Yee ~', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()

