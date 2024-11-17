
import math
from mindstorms import MSHub, Motor
import utime
import machine
from hub import motion
from hub import port

# UART Configuration
portB = port.B
portB.mode(1)   # MODE_FULL_DUPLEX = 1
utime.sleep_ms(500)
portB.baud(115200)

class Uart:
    # Initialize UART
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
                if byte_read == b"\0":  # End-of-message character
                    if len(message) >= total_bits:
                        return "".join(message[-total_bits:])
                    else:
                        print("[Error] Message shorter than expected total_bits.")
                        return None
                else:
                    message.append(chr(byte_read[0]))

            # Handle timeout
            if self.timeOut != 0 and (utime.time() - start) >= self.timeOut:
                print("[Timeout] UART timeout exceeded.")
                return None

    # Write string to UART
    def write(self, str_data):
        self.port.write(str_data)

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
        self.motor_FR = Motor('F')  # Front right
        self.motor_FL = Motor('D')  # Front left
        self.motor_RR = Motor('E')  # Rear right
        self.motor_RL = Motor('A')  # Rear left

    def setWheelSpeed(self, speeds):
        """
        Sets the speeds of the four mecanum wheels.
        Args:
            speeds (list): A list of four speeds for the wheels.
        """
        upper_limit = 100
        lower_limit = -50
        for i in range(len(speeds)):
            if speeds[i] <= lower_limit:
                speeds[i] = lower_limit
            elif speeds[i] >= upper_limit:
                speeds[i] = upper_limit
        self.motor_FR.start_at_power(int(speeds[0]))
        self.motor_FL.start_at_power(-int(speeds[1]))
        self.motor_RR.start_at_power(int(speeds[2]))
        self.motor_RL.start_at_power(-int(speeds[3]))

    def getAllSpeeds(self):
        self.wheel_encoder[0] = self.motor_FR.get_speed()
        self.wheel_encoder[1] = self.motor_FL.get_speed()
        self.wheel_encoder[2] = self.motor_RR.get_speed()
        self.wheel_encoder[3] = self.motor_RL.get_speed()
        return self.wheel_encoder

    def stopMotor(self):
        """
        Stops all motors.
        """
        self.motor_FR.brake()
        self.motor_FL.brake()
        self.motor_RR.brake()
        self.motor_RL.brake()

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

    def driveRobotSimple(self, vx, PID):
        """
        Drives the robot with given velocities.
        Args:
            vx (float): Velocity in the x direction.
            PID (float): PID control value for steering.
        """
        min_limit = -20
        right_speed = vx + PID
        left_speed = vx - PID

        # Apply limits
        right_speed = max(min(right_speed, 100), min_limit)
        left_speed = max(min(left_speed, 100), min_limit)

        self.wheel_speeds[0] = right_speed  # Front right
        self.wheel_speeds[2] = right_speed  # Rear right
        self.wheel_speeds[1] = left_speed   # Front left
        self.wheel_speeds[3] = left_speed   # Rear left
        # Set motor speeds
        self.setWheelSpeed(self.wheel_speeds)

# PID Configuration
kp_line = 0.7
ki_line = 0.003
kd_line = 5
sampling_time_line = 1
invert_sampling_time_line = 1 / sampling_time_line
I_term_line = 0
delta_e_line = 0
prev_delta_e_line = 0
yaw_value = 0
prev_yaw_value = 0

# Reset yaw pitch roll
motion.yaw_pitch_roll(0)

def pid_line_calculate(sensor_value):
    global kp_line, ki_line, kd_line, invert_sampling_time_line, sampling_time_line
    global I_term_line, delta_e_line, prev_delta_e_line, yaw_value, prev_yaw_value

    prev_delta_e_line = delta_e_line
    delta_e_line = 51 - sensor_value  # Middle value
    print("[delta_e_line]:", delta_e_line)
    prev_yaw_value = yaw_value
    yaw_value = motion.yaw_pitch_roll()[0]

    if delta_e_line <= -10 or delta_e_line >= 10:
        P_term_line = kp_line * delta_e_line
        I_term_line += ki_line * delta_e_line * sampling_time_line
        # Using yaw for D term
        D_term_line = kd_line * (yaw_value - prev_yaw_value) / invert_sampling_time_line
        print("[D_term_line]:", D_term_line)
        PID_line_control = P_term_line + I_term_line - D_term_line
    else:
        PID_line_control = 0
    return PID_line_control

# Initialize UART and MSHub
lego_hub = Uart(portB, 0, "a")
hub = MSHub()
print("Hub initialized.\n")

# Flags and Variables
FLAG_UART_ACTIVE = 0
FLAG_ROBOT_RUN = 0  # 0-stop robot, 1-robot run
FLAG_DIRECTIONAL = 0  # 1-left | 2-mid | 3-right
FLAG_RUN_MODE = 1  # 1-normal run by PID | 3-run by challenge 3
FLAG_CHALLENGE_3_DONE = False  # Ensure Challenge 3 runs once after 5 seconds
challenge_3_start_time = 0  # To track the start time of Challenge 3

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

# Timer Callback Function
timer_counter = 0
def timer_callback(timer):
    global FLAG_UART_ACTIVE, timer_counter
    FLAG_UART_ACTIVE = 1  # Set flag to receive and transmit UART data
    # Increment timer_counter if needed for other purposes
    timer_counter += 1
    if timer_counter >= 100000:
        timer_counter = 0

# Create and initialize the timer
timer = machine.Timer(-1)
# Initialize the timer to call the callback function every 1000 ms (1 second)
timer.init(period=1000, mode=machine.Timer.PERIODIC, callback=timer_callback)

# Define States
IDLE = 0
CHALLENGE_1 = 1
CHALLENGE_2 = 2
CHALLENGE_3 = 3

# Initialize state
current_state = IDLE

# Function to handle state transitions based on challenge completion
def handle_state(state, challenge_complete):
    if state == IDLE:
        if challenge_complete == CHALLENGE_1:
            return CHALLENGE_1
        elif challenge_complete == CHALLENGE_2:
            return CHALLENGE_2
        elif challenge_complete == CHALLENGE_3:
            return CHALLENGE_3
    elif state == CHALLENGE_1:
        if challenge_complete:
            return CHALLENGE_2
    elif state == CHALLENGE_2:
        if challenge_complete:
            return CHALLENGE_3
    elif state == CHALLENGE_3:
        if challenge_complete:
            return IDLE
    return state

# Main Loop
while True:
    # Read UART input when active
    if FLAG_UART_ACTIVE == 1:
        lego_hub.write("rq")
        rx_message = lego_hub.read(4)  # Assume 4 bytes of data are received

        if rx_message and rx_message.startswith("ok"):
            print("ACK Received")
            FLAG_UART_ACTIVE = 0
            FLAG_ROBOT_RUN = 1

            # Parse data
            if len(rx_message) > 3:
                FLAG_DIRECTIONAL = ord(rx_message[2])  # Directional info
                horizon_data = ord(rx_message[3])      # Horizon data

            # Determine event based on received data
            # Not needed as transitions are based on challenge completion
            # This can be adjusted based on specific logic
            # For now, we assume transitions are sequential

        elif rx_message and rx_message.startswith("no"):
            print("Not Acknowledged")
            FLAG_UART_ACTIVE = 1  # Retry

    # Execute actions based on the current state
    if current_state == CHALLENGE_1:
        print("Executing Challenge 1")
        PID_value = pid_line_calculate(horizon_data)
        mecanum_robot.driveRobotSimple(30, -PID_value)
        # Example condition to end Challenge 1
        if some_condition_to_end_challenge_1():
            FLAG_ROBOT_RUN = 0
            current_state = handle_state(current_state, True)

    elif current_state == CHALLENGE_2:
        print("Executing Challenge 2")
        mecanum_robot.driveRobot(40, 0, 0)
        # Example condition to end Challenge 2
        if some_condition_to_end_challenge_2():
            FLAG_ROBOT_RUN = 0
            current_state = handle_state(current_state, True)

    elif current_state == CHALLENGE_3:
        print("Executing Challenge 3")
        # Check if 5 seconds have passed since Challenge 3 started
        if challenge_3_start_time == 0:
            challenge_3_start_time = utime.time()
            print("Challenge 3 started at:", challenge_3_start_time)

        elapsed_time = utime.time() - challenge_3_start_time
        print(f"Challenge 3 running for {elapsed_time:.2f} seconds")

        if elapsed_time >= 5 and not FLAG_CHALLENGE_3_DONE:
            # Execute timer-controlled movement once
            print("Starting timer-controlled movement for Challenge 3")
            FLAG_CHALLENGE_3_DONE = True  # Ensure this block runs only once
            # Initialize timer-controlled variables
            challenge_3_timer_start = utime.time()
            # Execute the timer-controlled movement
            while True:
                current_time = utime.time() - challenge_3_timer_start
                if current_time < 2:
                    mecanum_robot.driveRobot(0, 45, 0)  # Move diagonally
                elif 2 <= current_time <= 5:
                    mecanum_robot.driveRobot(40, 0, -5)  # Move forward with rotation
                else:
                    mecanum_robot.stopMotor()
                    print("Timer-controlled movement for Challenge 3 completed.")
                    current_state = handle_state(current_state, True)
                    challenge_3_start_time = 0  # Reset for potential future use
                    break  # Exit the movement loop

        else:
            # Continue with PID-based control during the first 5 seconds
            PID_value = pid_line_calculate(horizon_data)
            mecanum_robot.driveRobotSimple(30, -PID_value)

    elif current_state == IDLE:
        print("Robot is idle")
        mecanum_robot.stopMotor()
        # Potentially wait for a new UART command to start a challenge

    # Transition to the next challenge if in IDLE and FLAG_ROBOT_RUN is set
    if current_state == IDLE and FLAG_ROBOT_RUN == 0 and FLAG_UART_ACTIVE == 1:
        # Example logic: Start challenges sequentially
        # Adjust based on your specific event triggering
        if FLAG_DIRECTIONAL == 1:
            current_state = handle_state(current_state, CHALLENGE_1)
        elif FLAG_DIRECTIONAL == 2:
            current_state = handle_state(current_state, CHALLENGE_2)
        elif FLAG_DIRECTIONAL == 3:
            current_state = handle_state(current_state, CHALLENGE_3)
        else:
            print("Unknown FLAG_DIRECTIONAL value. Staying in IDLE.")

    # Small delay to prevent CPU overload
    utime.sleep_ms(100)

# Define the condition functions (placeholders)
def some_condition_to_end_challenge_1():
    # Implement your condition to end Challenge 1
    # Example: Based on sensor data or time
    return False  # Replace with actual condition

def some_condition_to_end_challenge_2():
    # Implement your condition to end Challenge 2
    # Example: After driving forward a certain distance or time
    return False  # Replace with actual condition
