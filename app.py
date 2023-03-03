import math
import time

import numpy as np
from flask import Flask, render_template, request
from flask_socketio import SocketIO
from threading import Lock
from flask import Flask, render_template, session
from flask_socketio import SocketIO, emit
import requests
import cv2
import pyrealsense2 as rs
from matplotlib import use

use('agg')
import matplotlib.pyplot as plt

import movement
from controllarm import XArmControl

try:

    jetson = True
except:
    jetson = False

app = Flask(__name__)
socketio = SocketIO(app)
thread = None
thread_lock = Lock()
x = 0
y = 0
from rplidar import RPLidar

BAUDRATE: int = 115200
TIMEOUT: int = 1
DMAX: int = 4000
IMIN: int = 0
IMAX: int = 50


# flask server to send robot postion to index db


def pick_up():
    arm = XArmControl('USB4996587C3033')
    arm1 = XArmControl('USB4991477B3033', [-7, 35, 18.5])
    arm.go_to_location(10, 20, 20)
    arm1.go_to_location(10, 25, 20)
    arm.arm.closeGripper(1000)
    arm1.arm.closeGripper(1000)
    # print("take it forward")
    time.sleep(1)
    arm.arm.openGripper(0)
    arm1.arm.openGripper(0)
    time.sleep(2)
    arm.go_to_location(10, 17, 10)
    arm1.go_to_location(10, 28, 10)
    time.sleep(2)

    arm.go_to_location(10, 17, 5)
    arm1.go_to_location(10, 28, 5)
    time.sleep(2)
    arm.arm.closeGripper(1000)
    arm1.arm.closeGripper(1000)
    time.sleep(2)

    arm.go_to_location(10, 17, 7)
    arm1.go_to_location(10, 28, 7)
    time.sleep(2)

    arm.go_to_location(3, 17, 7)
    arm1.go_to_location(3, 28, 7)
    time.sleep(2)

    arm.go_to_location(3, 17, 10)
    arm1.go_to_location(3, 28, 10)
    time.sleep(2)

    arm.go_to_location(20, 17, 10)
    arm1.go_to_location(20, 28, 10)
    time.sleep(2)

    arm.go_to_location(20, 17, 5)
    arm1.go_to_location(20, 28, 5)
    time.sleep(2)

    arm.arm.openGripper(0)
    arm1.arm.openGripper(0)
    time.sleep(2)

    arm.go_to_location(20, 17, 15)
    arm1.go_to_location(20, 28, 15)
    time.sleep(2)


@app.route('/')
def index():
    return render_template('index.html')


mode = 0
theta = 0
velocity = 0.01
omega = 0.05
LIDAR_DEVICE = '/dev/ttyUSB0'


@app.route('/move/', methods=['GET', 'POST'])
def move():
    global mode
    data = request.json
    if data['direction'] == 'forward':
        mode = 1
        if jetson:
            pass
        movement.forward_async()

    if data['direction'] == 'left':
        movement.turn_left_async()
        mode = 3
    if data['direction'] == 'right':
        movement.turn_right_async()
        mode = 4
    if data['direction'] == 'backward':
        movement.reverse_async()
        mode = 2
    if data['direction'] == 'stop':
        movement.stop()
        mode = 0
    if data['direction'] == 'pickup':
        mode = 0
        pick_up()

    return '{"success":1}'


def background_thread():
    """Example of how to send server generated events to clients."""
    count = 0
    global mode, theta, x, y, omega, velocity

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    print(pipeline)

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    print('Found device : {0}'.format(LIDAR_DEVICE))
    print('To stop - close the plot window')



    x1 = []
    y1 = []
    fig = plt.figure()
    cntr = 0
    while True:

        socketio.sleep(.1)
        count += 1
        if mode == 0:
            pass
        if mode == 1:
            x += velocity * math.cos(theta)
            y += velocity * math.sin(theta)
        if mode == 2:
            x -= velocity * math.cos(theta)
            y -= velocity * math.sin(theta)
        if mode == 3:
            theta -= omega
        if mode == 4:
            theta += omega
        socketio.emit('robot_position',
                      {"x": x, "y": y})

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        depth_colormap = cv2.flip(depth_colormap, 1)
        color_image = cv2.flip(color_image, 1)
        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        images = cv2.flip(images, 0)
        ret, buffer = cv2.imencode('.jpg', images)
        frame = buffer.tobytes()

        print('frame')

        socketio.emit('robot_image', {'frames': frame})

        # lidar

        # title = 'RPLIDAR'
        # fig.set_label(title)
        fig.canvas.draw()
        #
        # ax = plt.subplot(111, projection='polar')
        # line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
        # ax.set_title('360° scan result')
        # ax.set_rmax(DMAX)
        # ax.grid(True)
        #
        try:
            lidar = RPLidar(port=LIDAR_DEVICE, baudrate=BAUDRATE, timeout=TIMEOUT)
            iterator = lidar.iter_scans()
            if cntr%20 ==0:
                x1 =[]
                y1 = []
                plt.clf()
            # ani = animation.FuncAnimation(fig, update_line, fargs=(iterator, line), interval=50)
            items = [item for item in next(iterator)]
            print(items)

            for point in items:
                if point[2] <= 2000:
                    x1.append(point[2] * np.sin(point[1]* np.pi/180))
                    y1.append(point[2] * np.cos(point[1]* np.pi/180))
            # plt.show()
            plt.scatter(x1, y1)
            # plt.savefig('test.jpg')
            data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            ret, buffer = cv2.imencode('.jpg', data)
            frame = buffer.tobytes()

            socketio.emit('location', {'frames': frame})
            lidar.stop()
            lidar.disconnect()
            cntr +=1
        except:
            pass


@socketio.event
def connect():
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_thread)
    emit('robot_position', {'data': 'Connected', 'count': 0})


if __name__ == '__main__':
    socketio.run(app, host="0.0.0.0", port=7950, debug=True)
