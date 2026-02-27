import math
import matplotlib.pyplot as plt
import numpy as np

line = None          # trajectory line on main axes
ax = None          # main axes
axd = None          # dict of all subplot axes (mosaic)
fig = None          # figure handle

step = 0.1           # default spatial step size (m)

ROBOT_WIDTH = 0.21   # m, left–right wheel separation
ROBOT_LENGTH = 0.095  # m, front–rear wheel separation

# Wheel-speed arrow scale (visual only, not power %)
_ARROW_SCALE = 1.0
_ARROW_MAX_MPS = 1.0   # m/s that maps to full-length arrow


def valmap(value, istart, istop, ostart, ostop):
    """Equivalent of Arduino map()."""
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def _wheel_speeds(vx, vy, omega):
    """
    Compute per-wheel velocities (m/s) from body-frame vx, vy, omega.
    Returns dict with keys 'FL', 'FR', 'RL', 'RR'.
    Convention matches car.py moveAll().
    """
    half = (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    return {
        'FL': vy + vx - omega * half,
        'FR': vy - vx + omega * half,
        'RL': vy - vx - omega * half,
        'RR': vy + vx + omega * half,
    }


def _update_wheel_display(vx, vy, omega):
    """Redraw all four wheel-speed arrow subplots."""
    if axd is None:
        return
    speeds = _wheel_speeds(vx, vy, omega)
    wheel_keys = {'FL': 'upper left',
                  'FR': 'upper right',
                  'RL': 'lower left',
                  'RR': 'lower right'}
    for name, key in wheel_keys.items():
        if key not in axd:
            continue
        a = axd[key]
        a.cla()
        a.set_xlim(-1.5, 1.5)
        a.set_ylim(-1.5, 1.5)
        a.set_xticks([])
        a.set_yticks([])
        a.set_title(name, fontsize=8, pad=2)

        spd = speeds[name]
        # Arrow points "up" for forward, "down" for backward;
        # map speed to normalised length
        norm = max(_ARROW_MAX_MPS, 0.001)
        length = max(min(abs(spd) / norm, 1.0), 0.05)
        direction = 1 if spd >= 0 else -1
        a.quiver(0, 0, 0, direction * length,
                 color='royalblue' if spd >= 0 else 'tomato',
                 width=0.04, pivot='mid',
                 angles='uv', scale_units='height', scale=_ARROW_SCALE)
        a.text(0, -1.3, f'{spd:+.3f} m/s', ha='center',
               va='bottom', fontsize=7)


def create_plot(xMin, xMax, yMin, yMax):
    """
    Create the composite figure:
      left half  – main trajectory axes
      right half – 2×2 grid of wheel-speed subplots (FL, FR, RL, RR)
    """
    global line, ax, axd, fig
    plt.ion()

    fig, axd = plt.subplot_mosaic(
        [['main', 'upper left',  'upper right'],
         ['main', 'lower left',  'lower right']],
        figsize=(10, 5),
        layout='constrained'
    )
    fig.suptitle('Robot Trajectory + Wheel Speeds', fontsize=10)

    ax = axd['main']
    ax.set_aspect('equal')
    ax.set_xlim(xMin, xMax)
    ax.set_ylim(yMin, yMax)
    ax.grid(True)
    ax.set_title('Trajectory', fontsize=9)
    line, = ax.plot([], [], '-', color='steelblue')

    _update_wheel_display(0, 0, 0)


def end_plot():
    plt.ioff()
    input("Press Enter to close the plot...")
    plt.close()
    print("Plot closed")


def clear_plot():
    if line is not None:
        line.set_data([], [])
        plt.draw()


def plot_segment(x, y, dx, dy):
    line.set_data(np.append(line.get_xdata(), x),
                  np.append(line.get_ydata(), y))
    ax.quiver(x, y, dx, dy, color='r',
              pivot='mid', angles='xy', scale_units='xy')


def plot_line(start, end, speed, step=step):
    dist = np.hypot(end[0] - start[0], end[1] - start[1])
    num_points = max(int(dist / step), 2)
    steps_per_second = speed / step

    x_values = np.linspace(start[0], end[0], num_points)
    y_values = np.linspace(start[1], end[1], num_points)

    dx = end[0] - start[0]
    dy = end[1] - start[1]

    angle = math.atan2(dy, dx)
    vx = speed * math.cos(angle)
    vy = speed * math.sin(angle)

    for x, y in zip(x_values, y_values):
        _update_wheel_display(vx, vy, 0)
        plot_segment(x, y, dx, dy)
        plt.pause(1 / steps_per_second)


def plot_circle(radius, center, speed):
    steps_per_second = speed / step
    num_points = int(2 * np.pi * radius / step)
    theta = np.linspace(0, 2 * np.pi, num_points)

    omega = speed / radius  # rad/s for a circle (CCW positive)

    for i in range(len(theta)):
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        dx = -radius * np.sin(theta[i])
        dy = radius * np.cos(theta[i])
        # Body-frame: moving tangentially = pure vy, with omega turning
        _update_wheel_display(0, speed, omega)
        plot_segment(x, y, dx, dy)
        plt.pause(1 / steps_per_second)


def plot_quadrilateral(vertices, speed, step=step):
    x_coords = [v[0] for v in vertices] + [vertices[0][0]]
    y_coords = [v[1] for v in vertices] + [vertices[0][1]]

    for i in range(4):
        plot_line((x_coords[i], y_coords[i]),
                  (x_coords[i+1], y_coords[i+1]),
                  speed, step)


def plot_semicircle(start, end, speed, direction):
    center = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2]
    distance = np.hypot(end[0] - start[0], end[1] - start[1])
    radius = distance / 2

    num_points = max(int(np.pi * radius / step), 2)
    steps_per_second = speed / step
    initial_angle = np.arctan2(start[1] - center[1], start[0] - center[0])

    if direction == 'CW':
        theta = np.linspace(initial_angle, initial_angle - np.pi, num_points)
        omega = -speed / radius
        tangent_sign = -1   # CW tangent is opposite to CCW
    else:
        theta = np.linspace(initial_angle, initial_angle + np.pi, num_points)
        omega = speed / radius
        tangent_sign = 1

    for i in range(len(theta)):
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        dx = tangent_sign * (-radius * np.sin(theta[i]))
        dy = tangent_sign * (radius * np.cos(theta[i]))
        _update_wheel_display(0, speed, omega)
        plot_segment(x, y, dx, dy)
        plt.pause(1 / steps_per_second)


def plot_curved_quadrilateral(vertices, speed, step=step):
    x = [v[0] for v in vertices]
    y = [v[1] for v in vertices]

    plot_line((x[0], y[0]), (x[1], y[1]), speed, step)
    plot_semicircle((x[1], y[1]), (x[2], y[2]), speed, step, 'CCW')
    plot_line((x[2], y[2]), (x[3], y[3]), speed, step)
    plot_semicircle((x[3], y[3]), (x[0], y[0]), speed, step, 'CCW')


def plot_tank_turn(point, start_angle, end_angle, angle_step,
                   turn_speed):
    num_steps = max(int(abs(end_angle - start_angle) / angle_step), 1)
    angle_values = np.linspace(start_angle, end_angle, num_steps)

    # omega in rad/s: positive = CCW, negative = CW
    delta_deg = end_angle - start_angle
    omega_sign = 1 if delta_deg > 0 else -1
    omega = omega_sign * (turn_speed / (ROBOT_WIDTH / 2))
    arc_per_step = math.radians(angle_step) * (ROBOT_WIDTH / 2)
    pause = arc_per_step / turn_speed if turn_speed > 0 else 0.1

    for angle in angle_values:
        _update_wheel_display(0, 0, omega)
        plot_segment(point[0], point[1],
                     np.cos(np.radians(angle)),
                     np.sin(np.radians(angle)))
        plt.pause(pause)


def plot_hexagon(center, side_length, speed, step=step,
                 direction='CW', tank_turn=True):
    num_sides = 6
    angle_increment = 60 if direction == 'CCW' else -60

    vertices = []
    for i in range(num_sides):
        a = np.radians(i * angle_increment)
        vertices.append((center[0] + side_length * np.cos(a),
                         center[1] + side_length * np.sin(a)))

    current_heading = math.degrees(
        math.atan2(vertices[1][1] - vertices[0][1],
                   vertices[1][0] - vertices[0][0])
    )

    for i in range(num_sides):
        sv = vertices[i]
        ev = vertices[(i + 1) % num_sides]
        plot_line(sv, ev, speed, step)
        if tank_turn:
            next_heading = current_heading + angle_increment
            plot_tank_turn(ev, current_heading, next_heading, 10, speed)
            current_heading = next_heading
