from mindstorms import MSHub
from mindstorms.control import wait_for_seconds
# from mindstorms.operator import Color
import utime
import hub
from hub import port
import machine

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
    if message is not None and len(message) >= 2:
        if message[0] == 's':
            #todo: check if 's' not in [0] position
            return ord(message[1])
        else:
            return 200
    else:
        return 0    
    

# Timer callback function
def timer_callback(timer):
    hub.status_light.on('green')
    print("[Timer] callback function called.")
    lego_hub.write("s") #start receive value from STM32
    val = lego_hub.read()
    print("-> Received value:", val)
    # if val is not None:
    LineValue = GetUartData(val)
    print("[Data]:", LineValue)
        # pico.write("hello there")
    # else:
        # print("No valid data received from pico sensor.")
        # timer.deinit()
# Create a timer object
timer = machine.Timer(-1)

# Initialize the timer to call the callback function every 0.1ms
timer.init(period=500, mode=machine.Timer.PERIODIC, callback=timer_callback)


# Initialize the hub
lego_hub = Uart(portB, 0, "a")
hub = MSHub()

# Example usage
while True:
    hub.status_light.on('blue')
    wait_for_seconds(1)
    hub.status_light.on('red')
    wait_for_seconds(1)

