#!/usr/bin/python
# -*- coding: UTF-8 -*-
#import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import numpy as np
import math as m
import logging


fig, ax = plt.subplots()

def init():
  ax.axis('equal')
  ax.set_xlim(-0.6, 0.6)
  ax.set_ylim(-0.1, 1.7)

def parse_args():
    """Parses arguments."""
    parser = argparse.ArgumentParser(
        description='Obtencion de distintas medidas de evaluacion para un sistema de recuperacion de informacion')
    parser.add_argument('-l', '--log', type=str, required=True)
    parser.add_argument('-c', '--color', choices=['red', 'green'], default='red',
                        help='Color del plot, opciones [red, green]')
    parser.add_argument('-f', '--freq', type=int, default=5,
                        help='Frecuencia de plot de posiciones del log')
    parser.add_argument('-i', '--interval', type=float, default=0.001,
                        help='Tiempo entre cada plot')
    parser.add_argument('-m', '--map', type=str, default=0.1,
                        help='Mapa del recorrido')
    return parser.parse_args()

def dibrobot(loc_eje,c,tamano):
  if tamano=='p':
    largo=0.1
    corto=0.05
    descentre=0.01
  else:
    largo=0.5
    corto=0.25
    descentre=0.05

  trasera_dcha=np.array([-largo,-corto,1])
  trasera_izda=np.array([-largo,corto,1])
  delantera_dcha=np.array([largo,-corto,1])
  delantera_izda=np.array([largo,corto,1])
  frontal_robot=np.array([largo,0,1])
  tita=loc_eje[2]
  Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
             [np.sin(tita), np.cos(tita), loc_eje[1]],
              [0,        0 ,        1]])
  Hec=np.array([[1,0,descentre],
              [0,1,0],
              [0,0,1]])
  extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
  robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
  ax.plot(robot[0,:], robot[1,:], color=c)
  

def main():
  args = parse_args()
  posiciones = np.genfromtxt(args.log, dtype=[float, float, float], delimiter=",")
  plt.show(block = False)
  x_pos = [elem[0] for elem in posiciones]
  y_pos = [elem[1] for elem in posiciones]
  xmin = min(x_pos) - 0.5
  xmax = max(x_pos) + 0.5
  ymin = min(y_pos) - 0.5
  ymax = max(y_pos) + 0.5

  plt.axis('equal')
  #plt.axis([xmin, xmax, ymin, ymax])
  colors = cm.jet(np.linspace(0,1,len(posiciones)))
  for i,loc in enumerate(posiciones):
      if i % args.freq == 0:
          dibrobot(loc, colors[i], 'p')
          plt.draw()
          #plt.pause(args.interval)
  input('Press ENTER to finish')
          


if __name__ == "__main__":
    main()
