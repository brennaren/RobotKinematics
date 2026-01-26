'''
A program that plots a semicircle using matplotlib. Each semicircle
consists of arc segments defined by angle steps (similar to plotCircle.py),
and the drawing speed is controlled by steps per second.

@author Brenna Ren
@version January 26, 2026
'''

import matplotlib.pyplot as plt
import numpy as np


def plot_semicircle(start, end, line, angle_step, steps_per_second, direction):
    center = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2]
    distance = np.hypot(end[0] - start[0], end[1] - start[1])
    radius = distance / 2

    num_points = int(180 / angle_step) + 1

    # Determine starting angle using arctan function
    initial_angle = np.arctan2(start[1] - center[1], start[0] - center[0])

    # Generate theta values based on direction
    if direction == 'CW':
        theta = np.linspace(initial_angle, initial_angle-np.pi, num_points)
    else:
        theta = np.linspace(initial_angle, initial_angle+np.pi, num_points)

    for i in range(len(theta)):
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        line.set_data(np.append(line.get_xdata(), x),
                      np.append(line.get_ydata(), y))
        plt.show()
        plt.pause(1 / steps_per_second)
    plt.show()


if __name__ == '__main__':
    plot_semicircle((0, 0), (3, 2), 10, 5, 'CW')   # Modify parameters here
