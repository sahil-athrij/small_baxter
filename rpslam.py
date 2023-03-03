#!/usr/bin/env python3

'''
rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar

Copyright (C) 2018 Simon D. Levy
This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''
import io

import numpy as np
from PIL import Image

MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10
LIDAR_DEVICE = '/dev/ttyUSB0'
BAUDRATE: int = 115200
TIMEOUT: int = 1

# Ideally we could use all 250 or so samples that the RPLidar delivers in one
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES = 200

import cv2
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from roboviz import MapVisualizer
total = 0
if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(port=LIDAR_DEVICE, baudrate=BAUDRATE, timeout=TIMEOUT)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    # viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles = None

    # First scan is crap, so ignore it
    next(iterator)

    while True:
        try:
        # Extract (quality, angle, distance) triples from current scan
            items = [item for item in next(iterator)]

            # Extract distances and angles from triples
            distances = [item[2] for item in items]
            angles = [item[1] for item in items]

            # Update SLAM with current Lidar scan and scan angles if adequate
            if len(distances) > MIN_SAMPLES:
                slam.update(distances, scan_angles_degrees=angles)
                previous_distances = distances.copy()
                previous_angles = angles.copy()

            # If not adequate, use previous
            elif previous_distances is not None:
                slam.update(previous_distances, scan_angles_degrees=previous_angles)

            # Get current robot position
            x, y, theta = slam.getpos()

            # Get current map bytes as grayscale
            slam.getmap(mapbytes)
            print(len(mapbytes))
            a = np.array(mapbytes, dtype=np.uint8)
            a = a.reshape((MAP_SIZE_PIXELS,MAP_SIZE_PIXELS))
            a = cv2.cvtColor(a,cv2.COLOR_GRAY2RGB)
            image = Image.fromarray(a,"RGB")
            image.save('hello.png')
            # Display map and robot pose, exiting gracefully if user closes it
            # if not viz.display(x / 1000., y / 1000., theta, mapbytes):
            #     exit(0)

            total+=1
            if total==100:
                break
        except:
            pass

        # Shut down the lidar connection
        lidar.stop()
        lidar.disconnect()
