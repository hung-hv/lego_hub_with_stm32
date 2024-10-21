import math
from mindstorms import MSHub, Motor
import utime
import machine
from hub import motion
from hub import port

LineValue = 0
portB = port.B
"""
MODE_DEFAULT = 0
MODE_FULL_DUPLEX = 1
MODE_HALF_DUPLEX = 2
MODE_GPIO = 3
"""
portB.mode(1)
utime.sleep_ms(500)
portB.baud(9600)


class Uart:
    # initialize
    def __init__(self, port, timeOut: int, id: str):
        self.port = port
        self.message_started = False
        self.timeOut = timeOut
        self.id = id

    #read data from UART
    def read(self):
        # self.write("<"+self.id+">")
        start = utime.time()
        message = []
        while 1:
            # print("wating TX data...")
            byte_read = self.port.read(1)  # Read one byte over UART lines
            # print("-> byte read: ", byte_read)
            
            if byte_read:
                if byte_read == b"\0":
                    # End of message. Convert the message to a string and return it.
                    return str("".join(message))
                    # return str(message)
                else:
                    # Accumulate message byte.
                    try:
                        message.append(chr(byte_read[0]))
                    except:
                        pass
                if self.timeOut is not 0:
                    if utime.time() - start >= self.timeOut:
                        # raise Exception("Timeout exceded. Is pi pico connected?")
                        print("[Timeout] UART timeout exceded")
                        break
        
            else: 
                return None  # Return None if no valid data is received

    # write string to UART
    def write(self, str):
        self.port.write(str)

def GetUartData(message):
    """
    Retrieves data from UART based on the given message.
    Args:
        message (str): The message received from UART.
    Returns:
        int: [1:101] The data retrieved from UART.
            0: has less than 2 characters.
            200: first character is not 's'.
    """
    if message is not None and len(message) >= 4:
        if message[0] == 's':
            #todo: check if 's' not in [0] position
            direction = message[1]
            horizon_data = ord(message[2])
            vertical_data = ord(message[3])
            return direction, horizon_data, vertical_data
        else:
            return 0, 51, 51
    else:
        return 0, 51, 51    

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
        
        self.motor_FR = Motor('F') #front right
        # self.motor_FR.start_at_power(50)
        self.motor_FL = Motor('D') #front left
        # self.motor_FL.start_at_power(50)
        self.motor_RR = Motor('E') #rear right
        # self.motor_RR.start_at_power(50)
        self.motor_RL = Motor('A') #rear left
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
        # print("speeds[0]: " + str(speeds[0]))
        self.motor_FL.start_at_power(-int(speeds[1]))
        # print("speeds[1]: " + str(speeds[1]))
        self.motor_RR.start_at_power(int(speeds[2]))
        # print("speeds[2]: " + str(speeds[2]))
        self.motor_RL.start_at_power(-int(speeds[3]))
        # print("speeds[3]: " + str(speeds[3]))
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
        # L_and_R = 1
        self.wheel_speeds[0] = (1/R) * (vx + vy + (L + W) * omega)  # Front right
        self.wheel_speeds[1] = (1/R) * (vx - vy - (L + W) * omega)  # Front left
        self.wheel_speeds[2] = (1/R) * (vx - vy + (L + W) * omega)  # Rear right
        self.wheel_speeds[3] = (1/R) * (vx + vy - (L + W) * omega)  # Rear left
        

        # Set motor speeds
        self.setWheelSpeed(self.wheel_speeds)

# Initialize the hub
lego_hub = Uart(portB, 0, "a")
hub = MSHub()
print("init hub\n")
direction = ''
horizon_data = 0
vertical_data = 0
# Timer callback function
def timer_callback(timer):
    global direction, horizon_data, vertical_data
    hub.status_light.on('green')
    # print("[Timer] callback function called.")
    # lego_hub.write("r") #start receive value from STM32
    # val = lego_hub.read()
    # print("-> Received value:", val)
    
    # Send request to send 'r'
    lego_hub.write('r')
    # Receive ACK from STM32
    ack = lego_hub.port.read(1)
    if ack and ack == b'a':
        val = lego_hub.read()
        # print("-> Received value:", val)
        # Process the received value
        direction, horizon_data, vertical_data = GetUartData(val)
        # if direction is not None:
        # print("direction: " + str(direction) + " horizon_data: " + str(horizon_data) + " vertical_data: " + str(vertical_data))
        #     print(f"Received direction: {direction}")
        #     print(f"horizontal: {horizon_data}")
        #     print(f"vertical: {vertical_data}")
    # print("-> Received value:", val)
    # if val is not None:
    # LineValue = GetUartData(val)
    # print("[Data]:", LineValue)
    # pid_line_calculate(LineValue)
        # pico.write("hello there")
    # else:
        # print("No valid data received from pico sensor.")
        # timer.deinit()

kp_line = 1.3
ki_line = 0.005
kd_line = 0.6
PID_line_control = 0
sampling_time_line = 10
invert_samling_time_line = 1/sampling_time_line
I_term_line = 0
delta_e_line = 0
prev_delta_e_line = 0

def pid_line_calculate(sensor_value):
    global kp_line, ki_line, kd_line, PID_line_control, invert_samling_time_line, sampling_time_line, I_term_line, delta_e_line, prev_delta_e_line

    prev_delta_e_line = delta_e_line

    delta_e_line = 51 - sensor_value
    print("[delta_e_line]:", delta_e_line)
    P_term_line = kp_line * delta_e_line
    I_term_line = I_term_line + (ki_line * delta_e_line * sampling_time_line)
    D_term_line = kd_line * (delta_e_line - prev_delta_e_line)/invert_samling_time_line
    PID_line_control = P_term_line + I_term_line + D_term_line
    print("[PID_line_control]:", PID_line_control)

kp_vertical = 0.6
ki_vertical = 0
kd_vertical = 0.2
PID_vertical_control = 0
# sampling_time_line = 10
# invert_samling_time_line = 1/sampling_time_line
I_term_vertical = 0
delta_e_vertical = 0
prev_delta_e_vertical = 0

def pid_vertical_calculate(sensor_value):
    # global kp_line, ki_line, kd_line, PID_line_control, invert_samling_time_line, sampling_time_line, I_term_line, delta_e_line, prev_delta_e_line
    global kp_vertical, ki_vertical, kd_vertical, PID_vertical_control, I_term_vertical, delta_e_vertical, prev_delta_e_vertical, invert_samling_time_line, sampling_time_line

    prev_delta_e_vertical = delta_e_vertical

    delta_e_vertical = 51 - sensor_value
    print("[delta_e_vertical]:", delta_e_vertical)
    P_term_line = kp_vertical * delta_e_vertical
    I_term_vertical = I_term_vertical + (ki_vertical * delta_e_vertical * sampling_time_line)
    D_term_line = kd_vertical * (delta_e_vertical - prev_delta_e_vertical)/invert_samling_time_line
    PID_vertical_control = P_term_line + I_term_vertical + D_term_line
    print("[PID_vertical_control]:", PID_vertical_control)

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
kd = 20
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
    # print("[delta_e]: \n" + str(delta_e))
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
    # print("P_term: " + str(P_term) + " D_term: " + str(D_term) + " I_term: " + str(I_term))
    PID_control = - (P_term + D_term + I_term)
    if (delta_e > 0 and PID_control > 10):
        PID_control = 0
    if (delta_e < 0 and PID_control < 10):
        PID_control = 0
    # print("PID_control: " + str(PID_control))

    # print("yaw: " + str(rotations[0]) + " pitch: " + str(rotations[1]) + " roll: " + str(rotations[2]))

    # print("Time elapsed (us):", start_time - timer_us)
    timer_us = start_time

# Create a timer object
timer = machine.Timer(-1)
timer2 = machine.Timer(-1)
# Initialize the timer to call the callback function every 0.1ms
timer.init(period=sampling_time_line, mode=machine.Timer.PERIODIC, callback=timer_callback)
timer2.init(period=sampling_time, mode=machine.Timer.PERIODIC, callback=timer_callback2)

# Example usage
wheel_radius = 1  # 5 cm
robot_width = 0.5    # 20 cm
robot_length = 0.5   # 30 cm
# motor_ports = ['C', 'D', 'E', 'F']  # Motor ports



# Initialize the Mecanum robot
mecanum_robot = Mecanum(wheel_radius, robot_width, robot_length)
direction_flag = 1
while 1:
    # print("test!!!!\n")
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
    # print("PID_control: " + str(PID_control))
    speed = 40
    
    if (direction == 'h'):
        direction_flag = 1
        # pid_line_calculate(horizon_data)
        # vertical_control = 0
        # if (vertical_data > 51):
        #     vertical_control = speed
        # else:
        #     vertical_control = -speed
        # mecanum_robot.driveRobot(vertical_control, -PID_line_control, 0)
    elif (direction == 'v'):
        direction_flag = 0
    #     pid_vertical_calculate(vertical_data)
    #     horizon_control =0
    #     if (horizon_data > 51):
    #         horizon_control = speed
    #     else:
    #         horizon_control = -speed
    #     mecanum_robot.driveRobot(PID_vertical_control, horizon_control, 0)
    # else:
    #     mecanum_robot.driveRobot(0, 0, 0)

    if (direction_flag == 1):
        pid_line_calculate(horizon_data)
        vertical_control = 0
        if (vertical_data > 51):
            vertical_control = speed
        else:
            vertical_control = -speed
        mecanum_robot.driveRobot(vertical_control, -PID_line_control, 0)
    if (direction_flag == 0):
        pid_vertical_calculate(vertical_data)
        horizon_control =0
        if (horizon_data > 51):
            horizon_control = speed
        else:
            horizon_control = -speed
        mecanum_robot.driveRobot(PID_vertical_control, horizon_control, 0)
    #     pass
    # if vertical_data == None:
    #     vertical_data = 51
    #     pid_vertical_calculate(vertical_data)
    # else:
    #     pid_vertical_calculate(vertical_data)
    
    # pid_vertical_calculate(vertical_data)
    # print("vertical_data: " + str(vertical_data))   
    # mecanum_robot.driveRobot(PID_vertical_control, 0, 0)

    # mecanum_robot.driveRobot(30, 0, 0)
    # mecanum_robot.driveRobot(30, 0, 20)  # vx = 0.5 m/s, vy = 0 m/s, omega = 0 rad/s
    # print("PID_control: " + str(PID_line_control))
    mecanum_robot.getAllSpeeds()
    # utime.sleep(0)

# Stop the robot
mecanum_robot.stop_motors()
