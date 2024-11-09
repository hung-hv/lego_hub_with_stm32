import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from sensor_array import SensorArray

# Define the curve function with the provided polynomial equation
def curve_func(x):
    return (-0.00 * x**5) + (-0.00 * x**4) + (-0.00 * x**3) + (0.01 * x**2) + (0.93 * x) - 33.35

# Define the main figure with two subplots: one for the robot and one for the sensor array
fig, (ax_robot, ax_sensors) = plt.subplots(1, 2, figsize=(12, 8))
plt.subplots_adjust(bottom=0.25)

# Define the robot body in mm for the first subplot
robot_body = plt.Rectangle((-10, -10), 20, 20, color='gray')
ax_robot.add_patch(robot_body)

# Create x values for the curve in mm
x_mm = np.linspace(-500, 500, 100)
y_mm = curve_func(x_mm)

# Draw the curve in mm on the robot plot
line_mm, = ax_robot.plot(x_mm, y_mm, color='black', linewidth=3)

# Create four SensorArray objects for each side of the robot
number_of_sensors = 8
top_sensors = SensorArray('top', number_of_sensors, ax=ax_robot)
bottom_sensors = SensorArray('bottom', number_of_sensors, ax=ax_robot)
left_sensors = SensorArray('left', number_of_sensors, ax=ax_robot)
right_sensors = SensorArray('right', number_of_sensors, ax=ax_robot)

# Initialize robot position in mm
robot_x_mm = 0
robot_y_mm = 0

# Function to update the robot position based on x and y inputs
def update_position_by_coordinates(x, y):
    global robot_x_mm, robot_y_mm
    robot_x_mm = x
    robot_y_mm = y

    robot_body.set_xy((robot_x_mm - 10, robot_y_mm - 10))

    for sensor_array in [top_sensors, bottom_sensors, left_sensors, right_sensors]:
        sensor_array.update_positions(robot_x_mm, robot_y_mm)
        sensor_array.update_colors(curve_func)

# Mouse event handler to update robot position on mouse motion
def on_mouse_move(event):
    if event.inaxes == ax_robot:
        mouse_x, mouse_y = event.xdata, event.ydata
        update_position_by_coordinates(mouse_x, mouse_y)

fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)

# Initialize positions for sensor readings
edge_values = [0, 0, 0, 0]
bars = ax_sensors.bar(['Top', 'Right', 'Bottom', 'Left'], edge_values, color='blue')

# Update function for animation
def update_mm(frame):
    edge_values = [
        top_sensors.active_sensor_count(curve_func),
        right_sensors.active_sensor_count(curve_func),
        bottom_sensors.active_sensor_count(curve_func),
        left_sensors.active_sensor_count(curve_func)
    ]
    
    for i, bar in enumerate(bars):
        bar.set_height(edge_values[i])
        bar.set_color('red' if edge_values[i] > 0 else 'blue')
    
    return robot_body, *[sensor.circle for sensor in top_sensors.sensors + bottom_sensors.sensors + left_sensors.sensors + right_sensors.sensors], *bars

# Set up the plot limits and labels for the robot view in mm
ax_robot.set_xlim(-600, 600)
ax_robot.set_ylim(-600, 600)
ax_robot.axhline(0, color='gray', linewidth=0.5, linestyle='--')
ax_robot.axvline(0, color='gray', linewidth=0.5, linestyle='--')
ax_robot.set_aspect('equal', adjustable='box')
ax_robot.grid(True)
ax_robot.set_title("Omni-Directional Robot with Sensors (Polynomial Curve)")
ax_robot.set_xlabel("X position (mm)")
ax_robot.set_ylabel("Y position (mm)")
ax_robot.legend([line_mm], ['Polynomial Curve'])

# Configure the sensor array plot
ax_sensors.set_ylim(0, 4)
ax_sensors.set_title("Sensor Readings on Each Edge of Array")
ax_sensors.set_ylabel("Active Sensors")

# Create a slider for scaling the coordinate view
ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03])
scale_slider = Slider(ax_slider, 'Scale', 1, 10, valinit=1)

# Function to update axes limits based on slider value
def update_scale(val):
    scale = scale_slider.val
    ax_robot.set_xlim(-600 * scale, 600 * scale)
    ax_robot.set_ylim(-600 * scale, 600 * scale)
    fig.canvas.draw_idle()

scale_slider.on_changed(update_scale)

# Create animation
ani_mm = animation.FuncAnimation(fig, update_mm, frames=100, interval=100)

plt.show()
