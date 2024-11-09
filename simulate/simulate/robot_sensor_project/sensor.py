# sensor.py
import matplotlib.pyplot as plt

# Sensor Class
class Sensor:
    def __init__(self, offset_x, offset_y, radius=25, ax=None):
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.circle = plt.Circle((0, 0), radius, color='blue')
        if ax is not None:
            ax.add_patch(self.circle)
    
    def update_position(self, robot_x, robot_y):
        # Update sensor's position based on robot's position
        self.circle.center = (robot_x + self.offset_x, robot_y + self.offset_y)
        
    def update_color(self, curve_func):
        # Check proximity to the sine curve and update color
        sensor_x, sensor_y = self.circle.center
        target_y = curve_func(sensor_x)  # Sine curve y value for the current x
        if abs(sensor_y - target_y) < 100:
            self.circle.set_facecolor('red')
        else:
            self.circle.set_facecolor('blue')
            
    def is_active(self, curve_func):
        # Returns True if the sensor is "active" (detecting the line)
        sensor_x, sensor_y = self.circle.center
        target_y = curve_func(sensor_x)
        return abs(sensor_y - target_y) < 100
