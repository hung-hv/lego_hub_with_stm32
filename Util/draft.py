# def configure_port(port_name: str, baudrate: int):
#     """
#     Configures the specified port with the given baud rate.
    
#     Args:
#         port_name (str): The name of the port (e.g., 'A', 'B', 'C', 'D', 'E', 'F').
#         baudrate (int): The baud rate to set for the port.
#     """
#     port_obj = port.A
#     if port_name not in ['A', 'B', 'C', 'D', 'E', 'F']:
#         raise ValueError("Invalid port name. Port name must be 'A', 'B', 'C', 'D', 'E', or 'F'.")
#     elif port_name == 'A':
#         port_obj = port.A
#     elif port_name == 'B':
#         port_obj = port.B
#     elif port_name == 'C':
#         port_obj = port.C
#     elif port_name == 'D':
#         port_obj = port.D
#     elif port_name == 'E':
#         port_obj = port.E
#     elif port_name == 'F':
#         port_obj = port.F
#     # port_obj = getattr(port, port_name)
#     """
#     MODE_DEFAULT = 0
#     MODE_FULL_DUPLEX = 1
#     MODE_HALF_DUPLEX = 2
#     MODE_GPIO = 3
#     """
#     port_obj.mode(1)  # Set port to UART mode
#     utime.sleep_ms(500)
#     port_obj.baud(baudrate)
#     print(f"Initializing port {port_name} as UART port with baudrate {baudrate}")

# PORTNAME = 'B'

# configure_port(PORTNAME, BAUDRATE)