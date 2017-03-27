import cv2

face_cascade = cv2.CascadeClassifier('../../share_local/opencv-3.2.0/data/haarcascades/haarcascade_frontalface_alt.xml')
body_cascade = cv2.CascadeClassifier('../../share_local/opencv-3.2.0/data/haarcascades/haarcascade_fullbody.xml')
eye_cascade = cv2.CascadeClassifier('../../share_local/opencv-3.2.0/data/haarcascades/haarcascade_eye.xml')

class VideoCamera(object):
    def __init__(self, classify=False,fmrate=60):
        self.video = cv2.VideoCapture(0)
        self.classify = classify
        self.video.set(cv2.cv.CV_CAP_PROP_FPS, fmrate)

    def __del__(self):
        self.video.release()

    def get_frame(self):
        success, frame = self.video.read()

        if self.classify:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ### bodies = body_cascade.detectMultiScale(gray, 1.3, 5)
            ### for (x,y,w,h) in bodies:
            ###     cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
                
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x,y,w,h) in faces:
                
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
                ### roi_gray = gray[y:y+h, x:x+w]
                ### roi_color = frame[y:y+h, x:x+w]
                ### eyes = eye_cascade.detectMultiScale(roi_gray)
                ### for (ex,ey,ew,eh) in eyes:
                ###     cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
                    
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()
