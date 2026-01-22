'''
A program that plots a circle using matplotlib. Each circle consists
of arc segments defined by angle steps, and the drawing speed is
controlled by steps per second.

@author Brenna Ren
@version January 22, 2026
'''

import matplotlib.pyplot as plt
import numpy as np


def plot_circle(radius, center, angle_step, steps_per_second):
    num_points = int(360 / angle_step) + 1
    theta = np.linspace(0, 2 * np.pi, num_points)
    
    # Plot settings
    plt.ion()
    fig, ax = plt.subplots(figsize=(5, 5))  # Modify figure size here
    ax.set_aspect('equal')
    ax.set_xlim(center[0] - radius - 1, center[0] + radius + 1)
    ax.set_ylim(center[1] - radius - 1, center[1] + radius + 1)
    ax.grid(True)
    line, = ax.plot([], [], '-o')   # Modify line style here

    for i in range(len(theta)):
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        line.set_data(np.append(line.get_xdata(), x),
                      np.append(line.get_ydata(), y))
        plt.show()
        plt.pause(1 / steps_per_second)
    plt.ioff()
    plt.show()


plot_circle(5, (0, 0), 10, 1)   # Modify parameters here
