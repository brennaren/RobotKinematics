'''
A program that plots a circle, line, quadrilateral, semicircle, and
curved quadrilateral using matplotlib. Each shape consists of smaller
line segments and the drawing speed is controlled by steps per second.
Each shape's plotting function is defined within this single file.

@author Brenna Ren
@version January 28, 2026
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
    line, = ax.plot([], [], '-')   # Modify line style here

    for i in range(len(theta)):
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        line.set_data(np.append(line.get_xdata(), x),
                      np.append(line.get_ydata(), y))
        plt.show()
        plt.pause(1 / steps_per_second)
    plt.show()


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


def plot_curved_quadrilateral(vertices, steps_per_second):
    # Coordinate arrays
    x_coords = [v[0] for v in vertices]
    y_coords = [v[1] for v in vertices]

    # Determine direction for semicircles (CW or CCW)
    direction = 'CCW'

    # Plot settings
    plt.ion()
    fig, ax = plt.subplots(figsize=(5, 5))  # Modify figure size here
    ax.set_aspect('equal')
    ax.set_xlim(min(x_coords) - 2, max(x_coords) + 2)
    ax.set_ylim(min(y_coords) - 2, max(y_coords) + 2)
    ax.grid(True)
    line, = ax.plot([], [], '-')   # Modify line style here

    plot_line((x_coords[0], y_coords[0]), (x_coords[1], y_coords[1]),
              line, steps_per_second)
    plot_semicircle((x_coords[1], y_coords[1]), (x_coords[2], y_coords[2]),
                    line, 10, steps_per_second, direction)
    plot_line((x_coords[2], y_coords[2]), (x_coords[3], y_coords[3]),
              line, steps_per_second)
    plot_semicircle((x_coords[3], y_coords[3]), (x_coords[0], y_coords[0]),
                    line, 10, steps_per_second, direction)
    plt.ioff()
    plt.show()


if __name__ == '__main__':
    # plot_circle(5, (0, 0), 10, 1)   # Modify parameters here
    # plot_quadrilateral([(0, 0), (4, 1), (5, 3), (2, 2)], 15)
    # plot_semicircle((0, 0), (3, 2), 10, 5, 'CW')   # Modify parameters here
    plot_curved_quadrilateral([(0, 0), (2, -2), (5, 3), (2, 2)], 15)
