# sensors/sensor.py

import matplotlib.pyplot as plt
import numpy as np

class Sensor:
    def __init__(self, ax=None, x=0, y=0, color='blue', offset_x=0, offset_y=0, radius=0.05):
        self.value = 0
        self.offset_x = offset_x
        self.offset_y = offset_y
        if ax and isinstance(ax, plt.Axes):
            self.point, = ax.plot(x, y, color + 'o')
        else:
            self.circle = plt.Circle((0, 0), radius, color=color)
            plt.gca().add_patch(self.circle)

    def set_position(self, x, y):
        if hasattr(self, 'point'):
            self.point.set_xdata([x])
            self.point.set_ydata([y])
        else:
            self.circle.center = (x + self.offset_x, y + self.offset_y)

    def set_color(self, color):
        if hasattr(self, 'point'):
            self.point.set_color(color)
        else:
            self.circle.set_facecolor(color)

    def set_value(self, value):
        self.value = value
    
    def get_value(self):
        return self.value

    def update_position(self, robot_x, robot_y):
        if hasattr(self, 'circle'):
            self.circle.center = (robot_x + self.offset_x, robot_y + self.offset_y)

    def update_color(self, curve_func):
        if hasattr(self, 'circle'):
            sensor_x, sensor_y = self.circle.center
            if abs(sensor_y - curve_func(sensor_x)) < 100:
                self.circle.set_facecolor('red')
            else:
                self.circle.set_facecolor('blue')

    def is_active(self, curve_func):
        if hasattr(self, 'circle'):
            sensor_x, sensor_y = self.circle.center
            return abs(sensor_y - curve_func(sensor_x)) < 100
        return False