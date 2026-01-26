'''
A program that plots a quadrilateral using matplotlib. Each quadrilateral
consists of line segments created using the plot_line method, and the
drawing speed is controlled by steps per second.

@author Brenna Ren
@version January 26, 2026
'''

import matplotlib.pyplot as plt
import numpy as np


def plot_line(start, end, line, steps_per_second):
    num_points = int(np.hypot(end[0] - start[0], end[1] - start[1])
                     * steps_per_second)
    x_values = np.linspace(start[0], end[0], num_points)
    y_values = np.linspace(start[1], end[1], num_points)

    for x, y in zip(x_values, y_values):
        line.set_data(np.append(line.get_xdata(), x),
                      np.append(line.get_ydata(), y))
        plt.pause(1 / steps_per_second)


def plot_quadrilateral(vertices, steps_per_second):
    # Coordinate arrays with point 0 appended at the end to close the shape
    x_coords = [v[0] for v in vertices] + [vertices[0][0]]
    y_coords = [v[1] for v in vertices] + [vertices[0][1]]

    # Plot settings
    plt.ion()
    fig, ax = plt.subplots(figsize=(5, 5))  # Modify figure size here
    ax.set_aspect('equal')
    ax.set_xlim(min(x_coords) - 1, max(x_coords) + 1)
    ax.set_ylim(min(y_coords) - 1, max(y_coords) + 1)
    ax.grid(True)
    line, = ax.plot([], [], '-')   # Modify line style here

    for i in range(4):
        plot_line((x_coords[i], y_coords[i]), (x_coords[i+1], y_coords[i+1]),
                  line, steps_per_second)

    plt.ioff()
    plt.show()


if __name__ == '__main__':
    # Modify parameters here
    plot_quadrilateral([(0, 0), (4, 1), (5, 3), (2, 2)], 15)
