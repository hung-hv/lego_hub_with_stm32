from mindstorms import MSHub, Motor
import utime

hub = MSHub()
motor_a = Motor('D')

speed = 50
motor_a.start_at_power(50)
# Press on the Hub's left button when you want the motor to stop.
# hub.left_button.wait_until_pressed()
# motor_a.stop()

while 1:
    motor_a.start_at_power(speed)
    print('Speed =', motor_a.get_speed())
    utime.sleep(1)
    # speed = -speed