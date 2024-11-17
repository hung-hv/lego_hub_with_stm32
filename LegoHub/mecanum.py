import math
from mindstorms import MSHub, Motor
import utime
import machine
from hub import motion
from hub import port

portB = port.B
portB.mode(1)   #MODE_DEFAULT = 0|MODE_FULL_DUPLEX = 1|MODE_HALF_DUPLEX = 2|MODE_GPIO = 3
utime.sleep_ms(500)
portB.baud(115200)

# # Initialize UART on port A
# portB = port.B.device
# portB.mode(1)  # UART mode
# portB.baud(9600)
class Uart:
    # initialize
    def __init__(self, port, timeOut: int, id: str):
        self.port = port
        self.message_started = False
        self.timeOut = timeOut
        self.id = id

    # Read data from UART
    def read(self, total_bits: int):
        start = utime.time()
        message = []

        while True:
            byte_read = self.port.read(1)  # Read one byte from UART
            print("-> byte read: ", byte_read)

            if byte_read:  # If a byte is received
                if byte_read == b"\0":  # Check for end-of-message character
                    # If the message is longer than total_bits, take only the last total_bits characters
                    if len(message) >= total_bits:
                        return "".join(message[-total_bits:])
                    else:
                        print("[Error] Message shorter than expected total_bits.")
                        return None  # Or raise an exception

                else:  # Accumulate valid bytes in the message list
                    message.append(chr(byte_read[0]))

            # Handle timeout
            if self.timeOut != 0 and utime.time() - start >= self.timeOut:
                print("[Timeout] UART timeout exceeded.")
                return None


    # write string to UART
    def write(self, str):
        self.port.write(str)

def GetUartData(message):
    """
    Retrieves data from UART based on the given message.
    Args:
        message (str): The message received from UART.
    Returns:
        int: [1:101] The data retrieved from UART. 111 is not valid value
            0: has less than 2 characters.
            200: first character is not 's'.
    """
    
    if message is not None:
        return ord(message)
    else:
        return 444


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
        # limit = 50
        upper_limit = 100
        lower_limit = -50
        for i in range(len(speeds)):
            if speeds[i] <= lower_limit:
                speeds[i] = lower_limit
            elif speeds[i] >= upper_limit:
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

    def driveRobotSimple(self, vx, PID):
        """
        Drives the robot with given velocities.
        Args:
            vx (float): Velocity in the x direction.
            omega (float): Angular velocity.
        """
        min_limit = -20
        right_speed = vx  +PID
        if right_speed <= min_limit:
            right_speed = min_limit
        left_speed = vx -PID
        if left_speed <= min_limit:
            left_speed = min_limit
        self.wheel_speeds[0] = right_speed  # Front right
        self.wheel_speeds[2] = right_speed # Rear right
        self.wheel_speeds[1] = left_speed # Front left
        self.wheel_speeds[3] = left_speed # Rear left
        # Set motor speeds
        self.setWheelSpeed(self.wheel_speeds)

kp_line = 0.7
ki_line = 0.003 #0.005
kd_line = 5 #0.6
sampling_time_line = 1
invert_samling_time_line = 1/sampling_time_line
I_term_line = 0
delta_e_line = 0
prev_delta_e_line = 0
#reset the yaw pitch roll
motion.yaw_pitch_roll(0)
yaw_value = 0
prev_yaw_value = 0

def pid_line_calculate(sensor_value):
    global kp_line, ki_line, kd_line, invert_samling_time_line, sampling_time_line, I_term_line, delta_e_line, prev_delta_e_line, yaw_value, prev_yaw_value

    prev_delta_e_line = delta_e_line

    delta_e_line = 51 - sensor_value #middle value
    print("[delta_e_line]:", delta_e_line)
    prev_yaw_value = yaw_value
    yaw_value = motion.yaw_pitch_roll()[0]
    if delta_e_line <= -10 or delta_e_line >= 10:
    # if 1:
        P_term_line = kp_line * delta_e_line
        I_term_line = I_term_line + (ki_line * delta_e_line * sampling_time_line)
        # D_term_line = kd_line * (delta_e_line - prev_delta_e_line)/invert_samling_time_line
        D_term_line = kd_line * (yaw_value - prev_yaw_value)/invert_samling_time_line
        print(D_term_line)
        PID_line_control = P_term_line + I_term_line - D_term_line
    else:
        PID_line_control = 0
    # print("[PID_line_control]:", PID_line_control)
    return PID_line_control

# Initialize the hub and uart for portB
lego_hub = Uart(portB, 0, "a")
hub = MSHub()
print("init hub\n")
FLAG_UART_ACTIVE = 0
# Timer callback function
timer_counter = 0
cl1_timer_counter = 0
FLAG_OBJ_3 = 0
def timer_callback(timer):
    global FLAG_UART_ACTIVE, timer_counter, FLAG_OBJ_3
    # global FLAG_UART_ACTIVE
    # hub.status_light.on('green')
    FLAG_UART_ACTIVE = 1 #set flag to receive and transmit uart data
    if FLAG_OBJ_3 == 1:
        timer_counter = timer_counter + 1
    else: 
        timer_counter = 0
    if timer_counter >=100000:
        timer_counter = 0

# Create a timer object
timer = machine.Timer(-1)
# Initialize the timer to call the callback function every 0.1ms
timer.init(period=sampling_time_line, mode=machine.Timer.PERIODIC, callback=timer_callback)

# Initialize the Mecanum robot
mecanum_robot = Mecanum(1, 1, 1)

def visualize_horizon_data(horizon_data):
    # Define the maximum and minimum values for horizon_data
    min_value = 1
    max_value = 101
    bar_length = 20  # Length of the visual bar
    # Handle error value (111) and invalid data
    if horizon_data == 111:
        print("Error: Invalid horizon data")
        print("X" + "-" * (bar_length - 1))
    elif horizon_data <= min_value or horizon_data >= max_value:
        print("Error: horizon_data out of range")
        print("X" + "-" * (bar_length - 1))
    else:
        # Map horizon_data (1 to 101) to the range of 0 to bar_length-1
        position = int((horizon_data - min_value) / (max_value - min_value) * (bar_length - 1))
        # Create the visualization string
        bar = "-" * position + "X" + "-" * (bar_length - 1 - position)
        # Print the visual bar
        print(bar)

PID_value = 0

FLAG_ROBOT_RUN = 0 # 0-stop robot, 1-robot run
FLAG_DIRECTIONAL = 0 # 2-left|3-mid|4-right
FLAG_RUN_MODE = 1 #1-normal run by pid| 3-run by challenge 3
while 1:
    
        
    #     mecanum_robot.driveRobot(0, 0, 0)
    if FLAG_UART_ACTIVE == 1:
        horizon_data = 333 # a invalid value
        lego_hub.write("rq")
        rx_message = lego_hub.read(4)
        if rx_message is not None:
            if rx_message.startswith("ok"):
                print("ACKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK")
                if rx_message[3] is not None:
                    horizon_data  = ord(rx_message[3])
                if rx_message[2] is not None:
                    FLAG_DIRECTIONAL = ord(rx_message[2])
                FLAG_UART_ACTIVE = 0 #deactivate transmition, wait for next transmit
                FLAG_ROBOT_RUN = 1
            if rx_message.startswith("no"):
                print("NOT ACK-----------------------------------------")
                FLAG_UART_ACTIVE = 1 #send request again till get uart signal
                FLAG_ROBOT_RUN = 0
        
        if horizon_data < 111:
            # FLAG_ROBOT_RUN = 1 #run 
            visualize_horizon_data(horizon_data)
            PID_value = pid_line_calculate(horizon_data)
            hub.status_light.on('green')
            if FLAG_DIRECTIONAL == 3:
                #run by mode 3
                FLAG_RUN_MODE = 3
            # lego_hub.write("ef")
        else:
            # lego_hub.write("rq")
            # val = lego_hub.read()
            # FLAG_ROBOT_RUN = 0 #stop the robot
            hub.status_light.on('red')
        print(horizon_data)

    if FLAG_DIRECTIONAL == 3:
        #run by mode 3
        FLAG_RUN_MODE = 3
    

    if FLAG_ROBOT_RUN == 1:
        # pass
        if FLAG_RUN_MODE == 3:
            FLAG_OBJ_3 = 1 #activate timer
            if timer_counter < 2000:
                mecanum_robot.driveRobot(0, 45, 0)
            if timer_counter >= 2000 and timer_counter <= 5000:
                mecanum_robot.driveRobot(40, 0, -5)
            if timer_counter > 5000:
                FLAG_RUN_MODE = 1
                FLAG_OBJ_3 = 0 #deactivate timer
            
        if FLAG_RUN_MODE == 1: 
            mecanum_robot.driveRobotSimple(30, -PID_value)  # vx = 0.5 m/s, vy = 0 m/s, omega = 0 rad/s
        mecanum_robot.getAllSpeeds()
    else:
        pass
    # # #wrong value of could not receive value from uart
    # mecanum_robot.driveRobotSimple(150, 0)


# Stop the robot
mecanum_robot.stop_motors()
