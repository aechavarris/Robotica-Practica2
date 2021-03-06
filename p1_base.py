#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import math as m
import time
from Robot import Robot

#Trayectoria 1: Linea recta 1.2m (3 baldosas) en 3s
def trayectoria1(robot):
    robot.setSpeed(0.4,0)
    time.sleep(3)
    robot.setSpeed(0,0)

#Trayectoria 2: Rectangulo 0.4x0.8 (1s por baldosa)
def trayectoria2(robot):
    robot.setSpeed(0.4,0)
    time.sleep(1)
    robot.setSpeed(0,m.pi/2)
    time.sleep(1)
    robot.setSpeed(0.4,0)
    time.sleep(2)
    robot.setSpeed(0,m.pi/2)
    time.sleep(1)
    robot.setSpeed(0.4,0)
    time.sleep(1)
    robot.setSpeed(0,m.pi/2)
    time.sleep(1)
    robot.setSpeed(0.4,0)
    time.sleep(2)
    robot.setSpeed(0,0)

#Trayectoria 3: Cadenas bici
def trayectoria3(robot):
    a = 1
    r = 2
    alpha = 0.2
    robot.setSpeed(0,m.pi/2)
    time.sleep(1)
    robot.setSpeed(a,-m.pi/2+alpha)
    time.sleep(1)
    robot.setSpeed(2*r/3,0)
    time.sleep(3)
    robot.setSpeed(r/4, (-m.pi-2*alpha)/4)
    time.sleep(4)
    robot.setSpeed(2*r/3,0)
    time.sleep(3)
    robot.setSpeed(a,-m.pi/2+alpha)
    time.sleep(1)


#Trayectoria 4: Circulos tangentes
def trayectoria4(robot):
    d = 0.4
    robot.setSpeed(0,-m.pi/2)
    time.sleep(1)
    robot.setSpeed(2*d/3,-m.pi/3)
    time.sleep(3)
    robot.setSpeed(2*d/3,m.pi/3)
    time.sleep(6)
    robot.setSpeed(2*d/3,-m.pi/3)
    time.sleep(3)
    robot.setSpeed(0,0)


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()
        robot.logger.debug("Logger iniciado ")
        # 2. perform trajectory
        trayectoria1(robot)
        robot.updateOdometry()

        # DUMMY, CODE! delete when you have your own
        # robot.setSpeed(1,1)
        # print("Start : %s" % time.ctime())
        # time.sleep(3)
        # print("X value from main tmp %d" % robot.x.value)
        # time.sleep(3)
        # print("End : %s" % time.ctime())

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # PART 1:
        # robot.setSpeed()
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...



        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



