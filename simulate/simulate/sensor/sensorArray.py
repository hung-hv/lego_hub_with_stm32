import matplotlib.pyplot as plt
from sensor.sensor import Sensor
import numpy as np


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