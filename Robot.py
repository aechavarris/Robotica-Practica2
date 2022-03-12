#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

#import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import numpy as np
import math as m
import logging


# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

#TODO GENERAL: añadir DocId


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        Log_Format = "%(levelname)s %(asctime)s - %(message)s"

        logging.basicConfig(filename = "logfile.log",
                            filemode = "w",
                            format = Log_Format, 
                            level = logging.DEBUG)

        self.logger = logging.getLogger()
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.R = 0.029 #Radio de la rueda
        self.L = 0.103 #Distancia entre ruedas
        #self.MOTOR_D = BP.PORT_B #Puerto motor derecho
        #self.MOTOR_I = BP.PORT_C #Puerto motor izquierdo
        #self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        #self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        #self.BP.offset_motor_encoder(self.BP.PORT_B,
        #    self.BP.get_motor_encoder(self.BP.PORT_B))
        #self.BP.offset_motor_encoder(self.BP.PORT_C,
        #    self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        self.v = Value('d',0)
        self.w = Value('d',0)
        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()

        # odometry update period --> UPDATE value!
        self.P = 1.0



    def setSpeed(self, v,w):
        """ To be filled - These is all dummy sample code """
        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...

        #speedPower = 100
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        self.v.value = v
        self.w.value = w
        
        # Pseudocódigo (Diapositiva 8)
        # wd,wi = (1/R, L/2R) dot (v,w)
        #         (1/R,-L/2R)
        # motor_d = wd
        # motor_i = wi
        speed_matrix = np.array([[1/self.R, self.L/(2*self.R)] ,
                                [1/self.R, -self.L/(2*self.R)]])
        motor_w = np.dot(speed_matrix, np.array([v,w]))
        motor_w[0] = m.degrees(motor_w[0])
        motor_w[1] = m.degrees(motor_w[1])
        self.BP.set_motor_power(MOTOR_D, motor_w[0]) #Motor derecho
        self.BP.set_motor_power(MOTOR_I, motor_w[1]) #Motor izquierdo

        #speedDPS_left = 180
        #speedDPS_right = 180
        #self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        #self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)


    def readSpeed(self):
        """ To be filled"""
        #[f_d, p_d, e_d, rep_d] = self.BP.get_motor_status(MOTOR_D)
        #[f_i, p_i, e_i, rep_i] = self.BP.get_motor_status(MOTOR_I)

        #w_d = m.radians(rep_d)
        #w_i = m.radians(rep_i)
        #w = self.R*(w_d-w_i)/self.L
        #v = self.R*(w_d-w_i)/2

        #return [v,w]
        return [self.v.value,self.w.value]

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.viejoDer = 0
        self.viejoIzq = 0
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)
        
    def norm_pi(theta):
        return theta % 2*m.pi

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        odometryLog = []
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.time()
            odometryLog.append((self.x.value, self.y.value, self.th.value))

            # compute updates

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            #sys.stdout.write("Dummy update of odometry ...., X=  %d, \
                #Y=  %d, th=  %d \n" %(self.x.value, self.y.value, self.th.value) )
            #print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )
            #Pseudocodigo
            #previamente almacenaremos en variables self valores "antiguos"
            #   motoresV = motor derechoV, motor izquierdoV
            #   calcular diferencia de velocidades
            #   calcular cambio de valores x/y/th (Diapositiva 4 y 7)
            #   actualizar valores de odometria

            [motorDer, motorIzq] = [self.BP.get_motor_encoder(MOTOR_D), 
                                   self.BP.get_motor_encoder(MOTOR_I)]
            
            speedDer = motorDer - self.viejoDer
            speedIzq = motorIzq - self.viejoIzq
            speedDer = m.radians(speedDer) * self.R
            speedIzq = m.radians(speedIzq) * self.R
            
            
            s = (speedDer + speedIzq) / 2.0
            aux_th = (speedDer - speedIzq) / self.L
            #aux_x = s * m.cos(th + aux_th/2)
            #aux_y = s * m.sin(th + aux_th/2)

            # to "lock" a whole set of operations, we can use a "mutex"
            self.lock_odometry.acquire()
            th_viejo = self.th.value
            self.x.value += s * m.cos(th_viejo + aux_th/2)
            self.y.value += s * m.sin(th_viejo + aux_th/2)
            self.th.value += aux_th
            self.th.value = self.norm_pi(self.th.value)
            x_log = self.x.value
            y_log = self.y.value
            theta_log = self.th.value
            self.lock_odometry.release()
            
            self.viejoDer = motorDer
            self.viejoIzq = motorIzq

            #try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                #sys.stdout.write("Reading encoder values .... \n")
                #[encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                #    self.BP.get_motor_encoder(self.BP.PORT_C)]
            #except IOError as error:
                #print(error)
                #sys.stdout.write(error)

            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.time()
            time.sleep(self.P - (tEnd-tIni))
            self.logger.debug(str(x_log), ' ', str(y_log), ' ', str(theta_log))

        self.logger.debug("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))
        #sys.stdout.write("Stopping odometry ... X=  %.2f, \
        #        Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        #self.BP.reset_all()

