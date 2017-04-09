#!/usr/bin/env python

from flask import Flask, render_template, Response
import os
app = Flask(__name__)

from picar import PiCar

car = PiCar()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/forward')
def forward():
    car.forward()
    return ""

@app.route('/reverse')
def reverse():
    car.reverse()
    return ""

@app.route('/left')
def left():
    car.left()
    return ""

@app.route('/right')
def right():
    car.right()
    return ""

@app.route('/stop')
def stop():
    car.stop()
    return ""
        
def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    if car.camera:
        return Response(gen(car.camera),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return ""

# ------------------------------------------------------------------------------------------        
if __name__ == "__main__":
    import os
    pid=os.getpid()
    with open('pid','w+') as pidfile:
        pidfile.write('%s\n' % pid)
        pidfile.close()
    car()
    from threading import Thread
    ## thread = Thread(target=lambda: app.run(threaded=True,host='0.0.0.0',port=8000))
    try:
        ## thread.start()
        ## thread.join()
        app.run(threaded=True,host='0.0.0.0',port=8000)
    finally:
        car.quit()
        del car
