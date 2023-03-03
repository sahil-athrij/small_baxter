import argparse
from os import path
from platform import system

import matplotlib.animation as animation
import numpy as np
from matplotlib import use
from rplidar import RPLidar

BAUDRATE: int = 115200
TIMEOUT: int = 1
DMAX: int = 4000
IMIN: int = 0
IMAX: int = 50


def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intents = np.array([meas[0] for meas in scan])
    line.set_array(intents)
    return line


def run():
    if system() == 'Darwin':
        use('MacOSX')

    description = 'rplidar device plot'
    epilog = 'The author assumes no liability for any damage caused by use.'
    parser = argparse.ArgumentParser(prog='./device_plot.py', description=description, epilog=epilog)
    parser.add_argument('device', help="device path", type=str)
    args = parser.parse_args()
    dev_path = args.device

    if path.exists(dev_path):
        use('Agg')
        import matplotlib.pyplot as plt

        print('Found device : {0}'.format(dev_path))
        print('To stop - close the plot window')

        lidar = RPLidar(port=dev_path, baudrate=BAUDRATE, timeout=TIMEOUT)
        x = []
        y = []
        fig = plt.figure()
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
        iterator = lidar.iter_scans()
        # ani = animation.FuncAnimation(fig, update_line, fargs=(iterator, line), interval=50)
        items = [item for item in next(iterator)]
        print(items)
        for point in items:
            if point[0] <= 15:
                x.append(point[2] * np.sin(point[1]))
                y.append(point[2] * np.cos(point[1]))
        # plt.show()

        plt.scatter(x, y)
        data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        lidar.stop()
        lidar.disconnect()
    else:
        print('[Error] Could not found device: {0}'.format(dev_path))


if __name__ == '__main__':
    run()
