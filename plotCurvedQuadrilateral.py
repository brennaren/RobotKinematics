'''
A program that plots a curved quadrilateral using matplotlib. Each
curved quadrilateral consists of line segments created using the
plot_line method and semicircles created using the plot_semicircle method,
and the drawing speed is controlled by steps per second.

@author Brenna Ren
@version January 26, 2026
'''

import matplotlib.pyplot as plt

from plotQuadrilateral import plot_line
from plotSemicircle import plot_semicircle


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
    # Modify parameters here
    plot_curved_quadrilateral([(0, 0), (2, -2), (5, 3), (2, 2)], 15)
