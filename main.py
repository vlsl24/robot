#!/usr/bin/env pybricks-micropython

# Pybricks imports
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# Robot definitions
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Sensor definitions
left_light = ColorSensor(Port.S2)
right_light = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4) 

"""
Important! The code may not function correctly if the first line from the template is not included.
"""

# Your code goes here

# Define line/background reflection
line_reflection = 7
background_reflection = 60

#Creating target value, this will be the value when the line/background both share the same amount of the sensor
target_value = (line_reflection + background_reflection) * 0.5

#Abs amount from line reflection -> target and background reflection -> target
difference = target_value - line_reflection

rotation_speed = 25.0
drive_speed = 25.0

found_line = False

def follow_line():
    global waitedTime
    global found_line

    while(not found_line):
        robot.turn(90)
        robot.stop()
        robot.drive(100, 0)
        waitedTime = 0
        while(waitedTime < 2000):
            wait(10)
        
            if(right_light.reflection() <= line_reflection):
                found_line = True
        
            waitedTime += 10


    #Found line
    while True:
        reflection = right_light.reflection()

        #Because we do 'reflection - target_value' we are following the left side of the line
        offset_to_target = reflection - target_value

        #Convert offset to a value between -1.0 -> 1.0
        rotation_percentage = offset_to_target / difference

        if(abs(rotation_percentage) > 0.5):
            robot.drive(0, rotation_speed * rotation_percentage)
        else:
            robot.drive(drive_speed, rotation_speed * rotation_percentage)  


follow_line()

# If we need to get the reflections of line/background
# while True:
#     print(sensor.reflection())
