from mindstorms import MSHub
from mindstorms.control import wait_for_seconds
# from mindstorms.operator import Color
import utime
import hub
from hub import port
import machine
from Util.uart import Uart




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


class picoSensor:
    # funkcija, ki skrbi za inicializacijo
    def __init__(self, port, timeOut: int, id: str):
        self.port = port
        self.message_started = False
        self.timeOut = timeOut
        self.id = id

    """
    funkcija za branje podatkov, ki jih posreduje pico
    najprej pico pošlje id senzorja nato pa čaka na sporočilo
    sporočilo je oblike <podatek>, kjer znak < označuje začetek podatka, znak > pa konec
    funkcija vrne podatek kot int
    """

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


    # s to funkcijo lahko pošjemo poljubno sporočilo pico
    def write(self, str):
        self.port.write(str)


def GetUartData(message: str):
    """
    Retrieves data from UART based on the given message.

    Args:
        message (str): The message received from UART.

    Returns:
        int: [1:101] The data retrieved from UART.
            0: has less than 2 characters.
            200: first character is not 's'.

    Raises:
        None

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
    print("[Timer] callback function called.")
    pico.write("s") #start receive value from STM32
    val = pico.read()
    print("-> Received value:", val)
    if val is not None:
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


# definicija senzorjev
pico = picoSensor(portB, 0, "a")
pico1 = picoSensor(portB, 0, "b")
hub = MSHub()

# Example usage
while True:
    hub.status_light.on('blue')

    
    # val1 = 0
    # val = pico.read()
    # if val is not None:
    #     if val % 2 == 0:
    #         val1 = pico1.read()
    #         if val1 is None:
    #             val1 = 0
    #     print("this is val: ", val + val1, " ", type(val))
    #     hub.display.show(str(val))
    #     break
    # else:
    #     print("No valid data received from pico sensor.")
    
    wait_for_seconds(2)
    	
    hub.status_light.on('red')

