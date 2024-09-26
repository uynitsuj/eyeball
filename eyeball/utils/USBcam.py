import numpy as np
import cv2

class USBCamera(object):
    """
    Driver for v4l2 USB camera devices that support cv2 video stream reading
    
    Parameters
    ----------
    device : int
        v4l2 port (/dev/video<#>) for camera to connect to
    """
    
    def __init__(self, device, crop = (0,0,0,0)):
        self._device = int(device)
        self._is_running = False
        self.height = 1600
        self.width = 1920
        
        self.initialize()
        
    def initialize(self):
        self.cap = cv2.VideoCapture(self._device)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FPS, 90)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
    @property
    def device_name(self):
        """int : The number of the PhoXi camera to connect to.
        """
        return self._device

    @property
    def is_running(self):
        """bool : True if the stream is running, or false otherwise.
        """
        return self._is_running

    def frames(self):
        #need this to work with CameraChessboardRegistration
        im = self.read()
        return im

    def read(self):
        """Read data from the sensor and return it.
        Returns
        -------
        data : :class:`.Image`
            The rgb image from the sensor.
        """
        # print("Reading from USB Camera")

        ret, frame = self.cap.read()
        self._is_running = ret
        if not ret:
            print("Re-initializing USB camera...")
            self.cap.release()
            self._is_running = False
            cv2.destroyAllWindows()
            self.initialize()
            if not self.video:
                ret, frame = self.cap.read()
                
            ret, frame = self.cap.read()

        # dst = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        # x, y, w, h = self.roi
        # color = dst[y:y+h, x:x+w] # (2119, 3804, 3)
        # color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        
        return frame
    
def __del__(self):
        """Automatically stop the sensor for safety.
        """
        if self._is_running:
            self.stop()