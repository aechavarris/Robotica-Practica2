import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder C

BP.set_motor_power(BP.PORT_B, 40)
BP.set_motor_power(BP.PORT_C, -40)

time.sleep(10.00)

BP.set_motor_power(BP.PORT_B + BP.PORT_C, 0)