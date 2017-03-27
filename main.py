#!/usr/bin/env python3

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

@app.route('/reverse')
def reverse():
    car.reverse()

@app.route('/left')
def left():
    car.left()

@app.route('/right')
def right():
    car.right()

@app.route('/stop')
def stop():
    car.stop()
        
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
    car()
    try:
        app.run(threaded=True)
    finally:
        del car
