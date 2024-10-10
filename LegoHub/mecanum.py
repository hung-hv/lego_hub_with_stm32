import math
from mindstorms import MSHub, Motor
import utime
import machine
from hub import motion



class Mecanum:
    def __init__(self, wheel_radius, robot_width, robot_length):
        self.wheel_radius = wheel_radius
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.wheel_speeds = [0, 0, 0, 0]
        self.wheel_encoder = [0, 0, 0, 0]
        self.wheel_angles = [0, 0, 0, 0]
        self.robot_speeds = [0, 0, 0]
        self.robot_angles = [0, 0, 0]
        self.robot_speed = 0
        self.robot_angle = 0
        self.robot_angular_speed = 0
        self.robot_angular_acceleration = 0
        self.robot_acceleration = 0
        self.robot_acceleration_x = 0

        # Initialize motors
        # self.motors = [Motor(port) for port in motor_ports]
        
        self.motor_FR = Motor('A') #front right
        # self.motor_FR.start_at_power(50)
        self.motor_FL = Motor('E') #front left
        # self.motor_FL.start_at_power(50)
        self.motor_RR = Motor('D') #rear right
        # self.motor_RR.start_at_power(50)
        self.motor_RL = Motor('F') #rear left
        # self.motor_RL.start_at_power(50)

    def setWheelSpeed(self, speeds):
        """
        Sets the speeds of the four mecanum wheels.
        
        Args:
            speeds (list): A list of four speeds for the wheels.
        """
        limit = 50
        upper_limit = limit
        lower_limit = -limit
        for i in range(len(speeds)):
            if speeds[i] < lower_limit:
                speeds[i] = lower_limit
            elif speeds[i] > upper_limit:
                speeds[i] = upper_limit
        self.motor_FR.start_at_power(int(speeds[0]))
        print("speeds[0]: " + str(speeds[0]))
        self.motor_FL.start_at_power(-int(speeds[1]))
        print("speeds[1]: " + str(speeds[1]))
        self.motor_RR.start_at_power(int(speeds[2]))
        print("speeds[2]: " + str(speeds[2]))
        self.motor_RL.start_at_power(-int(speeds[3]))
        print("speeds[3]: " + str(speeds[3]))
        # self.wheel_speeds = speeds
        # for motor, speed in zip(self.motors, speeds):
        #     motor.start_at_power(speed)

    def setEachWheelSpeed(self, speeds):
        pass

    def getAllSpeeds(self):
        self.wheel_encoder[0] = self.motor_FR.get_speed()
        self.wheel_encoder[1] = self.motor_FL.get_speed()
        self.wheel_encoder[2] = self.motor_RR.get_speed()
        self.wheel_encoder[3] = self.motor_RL.get_speed()
        # print("--------------------\n")
        # print("[" + str(self.wheel_encoder[1]) + "]" + " ---- " + "[" + str(self.wheel_encoder[0]) + "]" + "\n")
        # print("[" + str(self.wheel_encoder[3]) + "]" + " ---- " + "[" + str(self.wheel_encoder[2]) + "]" + "\n")
        # print("--------------------\n")
        return self.wheel_encoder

    def stopMotor(self):
        """
        Stops all motors.
        """
        for motor in self.motors:
            motor.brake()

    def driveRobot(self, vx, vy, omega):
        """
        Drives the robot with given velocities.
        
        Args:
            vx (float): Velocity in the x direction.
            vy (float): Velocity in the y direction.
            omega (float): Angular velocity.
        """
        L = self.robot_length
        W = self.robot_width
        R = self.wheel_radius

        # Calculate wheel speeds
        self.wheel_speeds[0] = (1/R) * (vx + vy + (L + W) * omega)  # Front right
        self.wheel_speeds[1] = (1/R) * (vx - vy - (L + W) * omega)  # Front left
        self.wheel_speeds[2] = (1/R) * (vx - vy + (L + W) * omega)  # Rear right
        self.wheel_speeds[3] = (1/R) * (vx + vy - (L + W) * omega)  # Rear left
        

        # Set motor speeds
        self.setWheelSpeed(self.wheel_speeds)
    
# Timer callback function
def timer_callback(timer):
    hub.status_light.on('green')
    # print("[Timer] callback function called.")
    # lego_hub.write("s") #start receive value from STM32
    # val = lego_hub.read()
    # print("-> Received value:", val)
    # # if val is not None:
    # LineValue = GetUartData(val)
    # print("[Data]:", LineValue)
        # pico.write("hello there")
    # else:
        # print("No valid data received from pico sensor.")
        # timer.deinit()



# Timer2 callback function
# rtc = machine.RTC()
timer_us = 0

#reset the yaw pitch roll
motion.yaw_pitch_roll(0)
#global variable
delta_e = 0
kp = 0.8

ki = 0.008
# ki = 0.004
# kd = 32
kd = 40
PID_control = 0
sampling_time = 1
invert_samling_time = 1/sampling_time
I_term = 0
# prev_I_term = 0

# Timer2 callback function for calculate PID loop
def timer_callback2(timer2):
    # print("[Timer2] callback function called.")
    global timer_us
    global PID_control
    global delta_e
    global invert_samling_time
    global sampling_time
    global I_term

    prev_delta_e = delta_e

    start_time = utime.ticks_us()

    #get delta_e
    delta_e = 0 - motion.yaw_pitch_roll()[0]
    print("[delta_e]: \n" + str(delta_e))
    # if (delta_e > 0):
    #     print("                                                                ----\n")
    # if (delta_e < 0):
    #     print("                                 ++++                               \n")
    # if (delta_e == 0):
    #     print("                                                 ----               \n")
    # hub.light_matrix.write(str(delta_e))
    #calculate PID controller
    P_term = kp * delta_e
    D_term = kd * (delta_e - prev_delta_e)/invert_samling_time
    I_term = I_term + (ki * delta_e * sampling_time)
    if (I_term > 15):
        I_term = 15
    if (I_term < -15):
        I_term = -15
    if (delta_e == 0):
        I_term = 0
    print("P_term: " + str(P_term) + " D_term: " + str(D_term) + " I_term: " + str(I_term))
    PID_control = - (P_term + D_term + I_term)
    if (delta_e > 0 and PID_control > 10):
        PID_control = 0
    if (delta_e < 0 and PID_control < 10):
        PID_control = 0
    print("PID_control: " + str(PID_control))

    # print("yaw: " + str(rotations[0]) + " pitch: " + str(rotations[1]) + " roll: " + str(rotations[2]))

    # print("Time elapsed (us):", start_time - timer_us)
    timer_us = start_time

# Create a timer object
timer = machine.Timer(-1)
timer2 = machine.Timer(-1)
# Initialize the timer to call the callback function every 0.1ms
timer.init(period=500, mode=machine.Timer.PERIODIC, callback=timer_callback)
timer2.init(period=sampling_time, mode=machine.Timer.PERIODIC, callback=timer_callback2)

# Example usage
wheel_radius = 1  # 5 cm
robot_width = 1    # 20 cm
robot_length = 1   # 30 cm
# motor_ports = ['C', 'D', 'E', 'F']  # Motor ports
hub = MSHub()
print("init hub\n")

# Initialize the Mecanum robot
mecanum_robot = Mecanum(wheel_radius, robot_width, robot_length)

while 1:
    print("test!!!!\n")
    # Drive the robot forward
    # mecanum_robot.driveRobot(50, 0, 0)  # vx = 0.5 m/s, vy = 0 m/s, omega = 0 rad/s
    # mecanum_robot.getAllSpeeds()
    # utime.sleep(1)
    # mecanum_robot.driveRobot(-50, 0, 0)  # vx = 0.5 m/s, vy = 0 m/s, omega = 0 rad/s
    # mecanum_robot.getAllSpeeds()
    # utime.sleep(1)
    # mecanum_robot.driveRobot(0, 50, 0)  # vx = 0.5 m/s, vy = 0 m/s, omega = 0 rad/s
    # mecanum_robot.getAllSpeeds()
    # utime.sleep(1)
    # mecanum_robot.driveRobot(0, -50, 0)  # vx = 0.5 m/s, vy = 0 m/s, omega = 0 rad/s
    # mecanum_robot.getAllSpeeds()
    # utime.sleep(1)

    # #apply PID_control
    print("PID_control: " + str(PID_control))
    mecanum_robot.driveRobot(0, 0, PID_control)  # vx = 0.5 m/s, vy = 0 m/s, omega = 0 rad/s
    mecanum_robot.getAllSpeeds()
    # utime.sleep(0)

# Stop the robot
mecanum_robot.stop_motors()
