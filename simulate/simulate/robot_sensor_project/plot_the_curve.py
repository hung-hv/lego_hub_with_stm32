import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.widgets import Slider

# Define the curve function with the provided polynomial equation
def curve_func(x):
    return (-0.00 * x**5) + (-0.00 * x**4) + (-0.00 * x**3) + (0.01 * x**2) + (0.93 * x) - 33.35

# Define the main figure with one subplot for the robot
fig, ax_robot = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(bottom=0.25)

# Create x values for the curve in mm
x_mm = np.linspace(-500, 500, 1000)  # Increased resolution for smoother curve
y_mm = curve_func(x_mm)

# Draw the curve in mm on the robot plot
line_mm, = ax_robot.plot(x_mm, y_mm, color='black', linewidth=3)

# Set up the plot limits and labels
ax_robot.set_xlim(-600, 600)
ax_robot.set_ylim(-600, 600)  # Adjusted y-limits to focus on the curve
ax_robot.axhline(0, color='gray', linewidth=0.5, linestyle='--')
ax_robot.axvline(0, color='gray', linewidth=0.5, linestyle='--')
ax_robot.set_aspect('equal', adjustable='box')
ax_robot.grid(True)
ax_robot.set_title("Polynomial Curve Plot")
ax_robot.set_xlabel("X position (mm)")
ax_robot.set_ylabel("Y position (mm)")
ax_robot.legend([line_mm], ['Polynomial Curve'])

# Create a slider for scaling the coordinate view
ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03])  # Position [left, bottom, width, height]
scale_slider = Slider(ax_slider, 'Scale', 1, 10, valinit=1)  # Scale from 1x to 10x

# Function to update axes limits based on slider value
def update_scale(val):
    scale = scale_slider.val
    ax_robot.set_xlim(-600 * scale, 600 * scale)
    ax_robot.set_ylim(-100 * scale, 100 * scale)  # Adjusted y-limits based on scale
    fig.canvas.draw_idle()

# Connect the slider to the update function
scale_slider.on_changed(update_scale)

plt.show()