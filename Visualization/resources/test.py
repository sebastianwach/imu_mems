import pygame
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import serial

print("Running test")


ser = serial.Serial('COM3', 115200, timeout = 0.1)

while True:
    line = ser.readline().decode('UTF-8').replace('\n', '')
    print(line)
    # yaw = float(line.split('y')[1])
    # print("Yaw", yaw)
    # pitch = float(line.split('p')[1])
    # print("Pitch:", pitch)
    # roll = float(line.split('r')[1])
    # print()s
    # print("Roll:", roll)
