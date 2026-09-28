import cv2

class FractalDetector:
    def __init__(self, config=None, marker_size=None):
        self.marker_size = marker_size
        try:
            self.detector = cv2.aruco.FractalMarkerDetector()
        except AttributeError:
            self.detector = None

    def detect(self, image):
        if self.detector:
            return self.detector.detectMarkers(image)
        return [], [], []
