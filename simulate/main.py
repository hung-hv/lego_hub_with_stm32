import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons
from threading import Timer

# Define the curve function for half a period of a sine wave
def sine_wave(x, shift=0, frequency=1, amplitude=1):
    return amplitude * np.sin(frequency * (x + shift))

# Define the curve function for a full circle using sine and cosine
def circle_points(radius=1, num_points=400):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return x, y

# Define the Sensor class
class Sensor:
    def __init__(self, ax, x, y, color):
        self.point, = ax.plot(x, y, color)
        self.value = 0

    def set_position(self, x, y):
        self.point.set_xdata([x])
        self.point.set_ydata([y])

    def set_color(self, color):
        self.point.set_color(color)

    def set_value(self, value):
        self.value = value
    
    def get_value(self):
        return self.value

# Create the figure and the initial points
fig, ax = plt.subplots(figsize=(12, 10))  # Increase the figure size
x = np.linspace(0, 2 * np.pi, 400)  # Full circle range
y = sine_wave(x)
curve_line, = ax.plot(x, y, label='Curve')  # Plot the curve

# Initialize two arrays of 8 sensors each
sensors_vertical = [Sensor(ax, 0, 0, 'ro') for _ in range(8)]  # Initial points at (0, 0)
sensors_horizontal = [Sensor(ax, 2, 0, 'bo') for _ in range(8)]  # Initial points at (2, 0)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.legend()

# Adjust the main plot to make room for the sliders and selection box
fig.subplots_adjust(left=0.1, bottom=0.3)

# Make a horizontal slider to control the shift of the curve
axpos_shift = fig.add_axes([0.1, 0.25, 0.8, 0.03])
shift_slider = Slider(
    ax=axpos_shift,
    label='Curve Shift',
    valmin=-np.pi,
    valmax=np.pi,
    valinit=0,
)

# Make a horizontal slider to control the frequency of the curve
axpos_freq = fig.add_axes([0.1, 0.2, 0.8, 0.03])
freq_slider = Slider(
    ax=axpos_freq,
    label='Frequency',
    valmin=0.1,
    valmax=5,
    valinit=1,
)

# Make a horizontal slider to control the amplitude of the curve
axpos_amp = fig.add_axes([0.1, 0.15, 0.8, 0.03])
amp_slider = Slider(
    ax=axpos_amp,
    label='Amplitude',
    valmin=0.1,
    valmax=2,
    valinit=1,
)

# Make a horizontal slider to control the scale of the graph
axpos_scale = fig.add_axes([0.1, 0.1, 0.8, 0.03])
scale_slider = Slider(
    ax=axpos_scale,
    label='Scale',
    valmin=1,
    valmax=10,
    valinit=1,
)

# Make a horizontal slider to control the radius of the circle
axpos_radius = fig.add_axes([0.1, 0.05, 0.8, 0.03])
radius_slider = Slider(
    ax=axpos_radius,
    label='Radius',
    valmin=0.1,
    valmax=5,
    valinit=1,
)

# Make a selection box to choose between sine wave and circle
axpos_select = fig.add_axes([0.025, 0.5, 0.15, 0.15])
select_box = RadioButtons(axpos_select, ('Sine Wave', 'Circle'))

# Timer variable to debounce the update function
update_timer = None

def update(val):
    global update_timer
    if update_timer is not None:
        update_timer.cancel()
    
    # Set a delay for the update logic
    update_timer = Timer(0.1, perform_update)
    update_timer.start()

def perform_update():
    shift = shift_slider.val
    frequency = freq_slider.val
    amplitude = amp_slider.val
    scale = scale_slider.val
    radius = radius_slider.val
    curve_type = select_box.value_selected
    if curve_type == 'Sine Wave':
        y = sine_wave(x, shift, frequency, amplitude)
        curve_line.set_xdata(x)
        curve_line.set_ydata(y)
    else:
        x_circle, y_circle = circle_points(radius=radius)
        curve_line.set_xdata(x_circle)
        curve_line.set_ydata(y_circle)
    
    # # Update vertical sensors
    # for i, sensor in enumerate(sensors_vertical):
    #     sensor.set_position(x_val, y_val - i * 0.1)  # Offset each sensor vertically, starting from the top
    #     if curve_type == 'Sine Wave':
    #         distance = abs(y_val - i * 0.1 - sine_wave(x_val, shift_slider.val, freq_slider.val, amp_slider.val))
    #     else:
    #         distance = abs(np.sqrt(x_val**2 + (y_val - i * 0.1)**2) - radius_slider.val)
        
    #     if distance <= 0.1:
    #         sensor.set_color('black')
    #         sensor.set_value(70 - (distance / 0.1) * 70)
    #     else:
    #         sensor.set_color('red')
    #         sensor.set_value(0)

    # # Update horizontal sensors
    # for i, sensor in enumerate(sensors_horizontal):
    #     sensor.set_position(x_val - i * 0.1, y_val)  # Offset each sensor horizontally, starting from the right
    #     if curve_type == 'Sine Wave':
    #         distance = abs(y_val - sine_wave(x_val - i * 0.1, shift_slider.val, freq_slider.val, amp_slider.val))
    #     else:
    #         distance = abs(np.sqrt((x_val - i * 0.1)**2 + y_val**2) - radius_slider.val)
        
    #     if distance <= 0.1:
    #         sensor.set_color('black')
    #         sensor.set_value(70 - (distance / 0.1) * 70)
    #     else:
    #         sensor.set_color('blue')
    #         sensor.set_value(0)
    
    # Calculate weighted averages
    vertical_sensor_values = [sensor.get_value() for sensor in sensors_vertical]
    horizontal_sensor_values = [sensor.get_value() for sensor in sensors_horizontal]

    vertical_weighted_avg = calculate_weighted_average(vertical_sensor_values)
    horizontal_weighted_avg = calculate_weighted_average(horizontal_sensor_values)

    # Clear previous text annotations
    for txt in ax.texts:
        txt.set_visible(False)

    # Add text annotations for weighted averages with bounding boxes
    bbox_props = dict(boxstyle="round,pad=0.3", edgecolor="black", facecolor="white", alpha=0.8)
    ax.text(0.05, 0.95, f'Vertical Weighted Avg: {vertical_weighted_avg:.2f}', transform=ax.transAxes, fontsize=12, verticalalignment='top', bbox=bbox_props)
    ax.text(0.05, 0.90, f'Horizontal Weighted Avg: {horizontal_weighted_avg:.2f}', transform=ax.transAxes, fontsize=12, verticalalignment='top', bbox=bbox_props)

    
    ax.set_xlim(-2 * scale, 2 * scale)
    ax.set_ylim(-2 * scale, 2 * scale)
    fig.canvas.draw_idle()

working_horizon_sensor = 0
working_vertical_sensor = 0
working_sensors = 0
last_weighted_sum = 0
MAXIMUM_SUM = 310
def calculate_weighted_average(sensor_values):
    """
    Calculate the weighted average of the sensor values, scaled to range from 1 to 101.
    
    Args:
    sensor_values (list of float): The values of the sensors.
    
    Returns:
    float: The weighted average of the sensor values, scaled to range from 1 to 101.
    """
    global working_sensors, last_weighted_sum
    working_sensors = 0
    for i in range(8):
        if sensor_values[i] != 0:
            working_sensors += 1
    weight_1 = 1
    weight_2 = 2
    weight_3 = 3
    vertical_sensor_values = [sensor.get_value() for sensor in sensors_vertical]
    horizontal_sensor_values = [sensor.get_value() for sensor in sensors_horizontal]
    print("Vertical Sensor Values:", vertical_sensor_values)
    print("Horizontal Sensor Values:", horizontal_sensor_values)
    # sensor_sum_values = sum(sensor_values)
    if working_sensors > 0:
        if working_sensors == 1:
            #in boundry
            if sensor_values[0] != 0:
                weighted_sum = -MAXIMUM_SUM
            if sensor_values[7] != 0:
                weighted_sum = MAXIMUM_SUM
        else:
            #in other case
            weighted_sum = (weight_3*(sensor_values[7] - sensor_values[0]) +
                            weight_2*(sensor_values[6] - sensor_values[1]) +
                            weight_1*(sensor_values[5] - sensor_values[2]))
            if weighted_sum > MAXIMUM_SUM:
                weighted_sum = MAXIMUM_SUM
            if weighted_sum < -MAXIMUM_SUM:
                weighted_sum = -MAXIMUM_SUM
        last_weighted_sum = weighted_sum
    else:
        weighted_sum = last_weighted_sum
    return weighted_sum
        

def on_mouse_move(event):
    global x_val, y_val
    x_val, y_val = event.xdata, event.ydata
    curve_type = select_box.value_selected
    if x_val is not None and y_val is not None:
        # Update vertical sensors
        for i, sensor in enumerate(sensors_vertical):
            sensor.set_position(x_val, y_val - i * 0.1)  # Offset each sensor vertically, starting from the top
            if curve_type == 'Sine Wave':
                distance = abs(y_val - i * 0.1 - sine_wave(x_val, shift_slider.val, freq_slider.val, amp_slider.val))
            else:
                distance = abs(np.sqrt(x_val**2 + (y_val - i * 0.1)**2) - radius_slider.val)
            
            if distance <= 0.1:
                sensor.set_color('green')
                sensor.set_value(70 - (distance / 0.1) * 70)
            else:
                sensor.set_color('red')
                sensor.set_value(0)

        # Update horizontal sensors
        for i, sensor in enumerate(sensors_horizontal):
            sensor.set_position(x_val - i * 0.1, y_val)  # Offset each sensor horizontally, starting from the right
            if curve_type == 'Sine Wave':
                distance = abs(y_val - sine_wave(x_val - i * 0.1, shift_slider.val, freq_slider.val, amp_slider.val))
            else:
                distance = abs(np.sqrt((x_val - i * 0.1)**2 + y_val**2) - radius_slider.val)
            
            if distance <= 0.1:
                sensor.set_color('green')
                sensor.set_value(70 - (distance / 0.1) * 70)
            else:
                sensor.set_color('blue')
                sensor.set_value(0)
        # Print the values of the sensors
        
        # Call perform_update to refresh the plot
        perform_update()

# Connect the sliders and selection box to the update function
shift_slider.on_changed(update)
freq_slider.on_changed(update)
amp_slider.on_changed(update)
scale_slider.on_changed(update)
radius_slider.on_changed(update)
select_box.on_clicked(update)

# Connect the mouse move event to the handler
fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)

plt.show()