# uart.py

import utime
from hub import port

class Uart:
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
        self.write("<"+self.id+">")
        start = utime.time()
        message = []
        while 1:
            print("waiting for TX data...")
            byte_read = self.port.read(1)  # Read one byte over UART lines
            if byte_read:
                if byte_read == b"\n":
                    # End of message. Convert the message to a string and return it.
                    return str("".join(message))
                else:
                    # Accumulate message byte.
                    try:
                        message.append(chr(byte_read[0]))
                    except:
                        pass
                if self.timeOut != 0:
                    if utime.time() - start >= self.timeOut:
                        print("[Timeout] UART timeout exceeded")
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