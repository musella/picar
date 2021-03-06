import time
import io
import threading
import picamera
import cv2
import numpy as np

class Camera(object):
    thread = None  # background thread that reads frames from camera
    frame = None  # current frame is stored here by background thread
    last_access = 0  # time of last client access to the camera

    def initialize(self):
        if Camera.thread is None:
            # start background frame thread
            Camera.thread = threading.Thread(target=self._thread)
            Camera.thread.start()

            # wait until frames start to be available
            while self.frame is None:
                time.sleep(0)

    def get_frame(self):
        Camera.last_access = time.time()
        self.initialize()
        return self.frame

    @classmethod
    def _thread(cls):
        with picamera.PiCamera() as camera:
            # camera setup
            ## camera.resolution = (320, 240)
            ## camera.resolution = (640, 480)
            camera.resolution = (800, 600)
            camera.hflip = False
            camera.vflip = False
            camera.framerate=100
            
            # let camera warm up
            camera.start_preview()
            time.sleep(2)

            stream = io.BytesIO()
            for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
                
                # store frame
                stream.seek(0)
                cls.frame = stream.read()

                ## data = np.fromstring(stream.read(), dtype=np.uint8)
                ## image = cv2.imdecode(data, 1)
                ## cv2.rectangle(image,(380,280),(420,320),(255,0,0),2)
                ## cls.frame = cv2.imencode('.jpg', image)[1].tobytes()
                
                # reset stream for next frame
                stream.seek(0)
                stream.truncate()

                # if there hasn't been any clients asking for frames in
                # the last 10 seconds stop the thread
                if time.time() - cls.last_access > 10:
                    break
        cls.thread = None
