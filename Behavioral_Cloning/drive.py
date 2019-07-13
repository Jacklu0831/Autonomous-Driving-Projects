# set up connection between model and simulator (udacity)
# conda virtual environment
# initialize python web application flask (python microframework for building web apps)
# install socket.io

import socketio
import eventlet
import numpy as np
from flask import Flask
from keras.models import load_model
import base64
from io import BytesIO
from PIL import Image
import cv2

# web sockets perform real time communication between client - server
# in our case it allows bidirectional communication with the simulator
# need middleware to bridge the communication
sio = socketio.Server()

app = Flask(__name__) # similar to "__main__"
speed_limit = 10

# matplotlib.image => mpimg
def img_preprocess(img):
    img = img[60:135, :, :]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV) # yuv is better for neural
    img = cv2.GaussianBlur(img, (3,3), 0)
    img = cv2.resize(img, (200, 66)) # less computationally expensive and matches the neural cfg
    img = img/255.0
    return img


# communicate data
@sio.on('telemetry')
def telemetry(sid, data):
	speed = float(data['speed'])
	# preprocess data from server
	image = Image.open(BytesIO(base64.b64decode(data['image'])))
	image = np.asarray(image)
	image = img_preprocess(image)
	image = np.array([image])
	steering_angle = float(model.predict(image))
	# speed control
	throttle = 1.0 - speed/speed_limit
	print('{} {} {}'.format(steering_angle, throttle, speed_limit))
	send_control(steering_angle, throttle)


# event handler
@sio.on('connect') # message, disconnect
def connect(sid, environ):
	print('Connected')
	send_control(0,0)


def send_control(steering_angle, throttle):
	sio.emit('steer', data = {
		'steering_angle': steering_angle.__str__(),
		'throttle': throttle.__str__()	
	})

# if script executed then run app
# localhost:3000/home
if __name__ == '__main__':
	# with no augmentation
	model = load_model('model/behavioral_cloning_model.h5')
	# with augmentation
	# model = load_model('model/behavioral_cloning_model_imgaug.h5')
	app = socketio.Middleware(sio, app)
	# wsgi allows client send data to server
	eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
