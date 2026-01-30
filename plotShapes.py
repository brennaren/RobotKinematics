import matplotlib.pyplot as plt
import numpy as np

line = None             # Global line variable to be used in plotting functions
ax = None               # Global axis variable to be used in plotting functions

def plot_segment(x, y, dx, dy):
    line.set_data(np.append(line.get_xdata(), x),
                  np.append(line.get_ydata(), y))
    ax.quiver(x, y, dx, dy, color='r',
              pivot = 'mid', angles='xy', scale_units='xy')


def plot_circle(radius, center, step, steps_per_second):
    num_points = int(2 * np.pi * radius / step)
    theta = np.linspace(0, 2 * np.pi, num_points)
    dx = -radius * np.sin(theta)
    dy = radius * np.cos(theta)

    for i in range(len(theta)):
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        plot_segment(x, y, dx[i], dy[i])
        plt.pause(1 / steps_per_second)


def plot_line(start, end, step, steps_per_second):
    num_points = int(np.hypot(end[0] - start[0], end[1] - start[1])
                     / step)
    x_values = np.linspace(start[0], end[0], num_points)
    y_values = np.linspace(start[1], end[1], num_points)

    dx = end[0] - start[0]
    dy = end[1] - start[1]

    for x, y in zip(x_values, y_values):
        plot_segment(x, y, dx, dy)
        plt.pause(1 / steps_per_second)


def plot_quadrilateral(vertices, step, steps_per_second):
    # Coordinate arrays with point 0 appended at the end to close the shape
    x_coords = [v[0] for v in vertices] + [vertices[0][0]]
    y_coords = [v[1] for v in vertices] + [vertices[0][1]]

    for i in range(4):
        plot_line((x_coords[i], y_coords[i]), (x_coords[i+1], y_coords[i+1]),
                  step, steps_per_second)


def plot_semicircle(start, end, step, steps_per_second, direction):
    center = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2]
    distance = np.hypot(end[0] - start[0], end[1] - start[1])
    radius = distance / 2

    num_points = int(np.pi * radius / step)

    # Determine starting angle using arctan function
    initial_angle = np.arctan2(start[1] - center[1], start[0] - center[0])

    # Generate theta values based on direction
    if direction == 'CW':
        theta = np.linspace(initial_angle, initial_angle-np.pi, num_points)
    else:
        theta = np.linspace(initial_angle, initial_angle+np.pi, num_points)
    
    dx = -radius * np.sin(theta)
    dy = radius * np.cos(theta)

    for i in range(len(theta)):
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        plot_segment(x, y, dx[i], dy[i])
        plt.pause(1 / steps_per_second)


def plot_curved_quadrilateral(vertices, step, steps_per_second):
    # Coordinate arrays
    x_coords = [v[0] for v in vertices]
    y_coords = [v[1] for v in vertices]

    # Determine direction for semicircles (CW or CCW)
    direction = 'CCW'

    plot_line((x_coords[0], y_coords[0]), (x_coords[1], y_coords[1]),
              step, steps_per_second)
    plot_semicircle((x_coords[1], y_coords[1]), (x_coords[2], y_coords[2]),
                    step, steps_per_second, direction)
    plot_line((x_coords[2], y_coords[2]), (x_coords[3], y_coords[3]),
              step, steps_per_second)
    plot_semicircle((x_coords[3], y_coords[3]), (x_coords[0], y_coords[0]),
                    step, steps_per_second, direction)


def plot_tank_turn(point, start_angle, end_angle, angle_step, steps_per_second):
    global cur_angle
    num_steps = int(abs(end_angle - start_angle) / angle_step)
    angle_values = np.linspace(start_angle, end_angle, num_steps)

    for angle in angle_values:
        plot_segment(point[0], point[1], np.cos(np.radians(angle)), np.sin(np.radians(angle)))
        plt.pause(1 / steps_per_second)


def create_plot():
    global line
    global ax
    # Plot settings
    plt.ion()
    fig, ax = plt.subplots(figsize=(5, 5))  # Modify figure size here
    ax.set_aspect('equal')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.grid(True)
    line, = ax.plot([], [], '-')   # Modify line style here


def end_plot():
    plt.ioff()
    py = input("Press Enter to close the plot...")
    plt.close()
    print("Plot closed")


def clear_plot():
    line.set_data([], [])
    plt.draw()


if __name__ == '__main__':
    create_plot()
    # Add test calls here if needed
    end_plot()
