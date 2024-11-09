import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button

# Define the main figure with two subplots: one for the robot and one for the sensor array
fig, (ax_robot, ax_sensors) = plt.subplots(1, 2, figsize=(12, 8))
plt.subplots_adjust(bottom=0.25)  # Make space for the slider and buttons

# Define the robot body in mm for the first subplot
robot_body = plt.Rectangle((-10, -10), 20, 20, color='gray')  # Size in mm
ax_robot.add_patch(robot_body)

# Define a curved line function and convert to mm
def curve_func(x, shape=1):
    if shape == 1:
        return 200 * np.sin(2 * np.pi * (x / 1000 + 0.5))  # Sine curve scaled to mm
    elif shape == 2:
        valid_x = np.clip(x, -200, 200)  # Limit x between -200 and 200
        return 200 * np.sqrt(40000 - valid_x**2)  # Circle curve scaled to mm
    else:
        raise ValueError("Invalid shape parameter. Shape must be 1 or 2.")

# Create x values for the curve in mm
x_mm = np.linspace(-500, 500, 100)
y_mm = curve_func(x_mm, 2)  # Default to circle shape

# Draw the curve in mm on the robot plot
line_mm, = ax_robot.plot(x_mm, y_mm, color='black', linewidth=3)

# Sensor Class
class Sensor:
    def __init__(self, offset_x, offset_y, radius=25):
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.circle = plt.Circle((0, 0), radius, color='blue')
        ax_robot.add_patch(self.circle)
    
    def update_position(self, robot_x, robot_y):
        # Update sensor's position based on robot's position
        self.circle.center = (robot_x + self.offset_x, robot_y + self.offset_y)
        
    def update_color(self, curve_func):
        # Check proximity to the curve and update color
        sensor_x, sensor_y = self.circle.center
        if abs(sensor_y - curve_func(sensor_x)) < 100:
            self.circle.set_facecolor('red')
        else:
            self.circle.set_facecolor('blue')
            
    def is_active(self, curve_func):
        # Returns True if the sensor is "active" (detecting the line)
        sensor_x, sensor_y = self.circle.center
        return abs(sensor_y - curve_func(sensor_x)) < 100

# Sensor Array Class
class SensorArray:
    def __init__(self, side, num_sensors, side_length=300):
        self.sensors = []
        offset_positions = np.linspace(-side_length / 2, side_length / 2, num_sensors)
        
        if side == 'top':
            for pos in offset_positions:
                self.sensors.append(Sensor(pos, side_length / 2))
        elif side == 'bottom':
            for pos in offset_positions:
                self.sensors.append(Sensor(pos, -side_length / 2))
        elif side == 'left':
            for pos in offset_positions:
                self.sensors.append(Sensor(-side_length / 2, pos))
        elif side == 'right':
            for pos in offset_positions:
                self.sensors.append(Sensor(side_length / 2, pos))

    def update_positions(self, robot_x, robot_y):
        for sensor in self.sensors:
            sensor.update_position(robot_x, robot_y)
            
    def update_colors(self, curve_func):
        for sensor in self.sensors:
            sensor.update_color(curve_func)
    
    def active_sensor_count(self, curve_func):
        return sum(sensor.is_active(curve_func) for sensor in self.sensors)

# Create four SensorArray objects
top_sensors = SensorArray('top', 4)
bottom_sensors = SensorArray('bottom', 4)
left_sensors = SensorArray('left', 4)
right_sensors = SensorArray('right', 4)

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
ax_robot.set_title("Omni-Directional Robot with Circular Sensors in a Square Array (mm)")
ax_robot.set_xlabel("X position (mm)")
ax_robot.set_ylabel("Y position (mm)")
ax_robot.legend([line_mm], ['Curved Line'])

# Configure the sensor array plot
ax_sensors.set_ylim(0, 4)
ax_sensors.set_title("Sensor Readings on Each Edge of Array")
ax_sensors.set_ylabel("Active Sensors")

# Create a slider for scaling the coordinate view
ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03])  # Position [left, bottom, width, height]
scale_slider = Slider(ax_slider, 'Scale', 1, 10, valinit=1)  # Scale from 1x to 10x

# Function to update axes limits based on slider value
def update_scale(val):
    scale = scale_slider.val
    ax_robot.set_xlim(-600 * scale, 600 * scale)
    ax_robot.set_ylim(-600 * scale, 600 * scale)
    fig.canvas.draw_idle()

# Connect the slider to the update function
scale_slider.on_changed(update_scale)

# Create buttons for shape selection
ax_sine_button = plt.axes([0.1, 0.15, 0.1, 0.05])  # Position for sine button
ax_circle_button = plt.axes([0.22, 0.15, 0.1, 0.05])  # Position for circle button

sine_button = Button(ax_sine_button, 'Sine')
circle_button = Button(ax_circle_button, 'Circle')

# Function to update the curve based on selected shape
def update_shape_sine(event):
    global y_mm
    y_mm = curve_func(x_mm, shape=1)
    line_mm.set_ydata(y_mm)
    ax_robot.relim()  # Recalculate limits
    ax_robot.autoscale_view()  # Autoscale the view
    fig.canvas.draw_idle()

def update_shape_circle(event):
    global y_mm
    y_mm = curve_func(x_mm, shape=2)
    line_mm.set_ydata(y_mm)
    ax_robot.relim()  # Recalculate limits
    ax_robot.autoscale_view()  # Autoscale the view
    fig.canvas.draw_idle()

# Connect the buttons to their respective update functions
sine_button.on_clicked(update_shape_sine)
circle_button.on_clicked(update_shape_circle)

# Create animation in mm
ani_mm = animation.FuncAnimation(fig, update_mm, frames=100, interval=100)

plt.show()
