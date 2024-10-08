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

FIND_LINE = 0
FOLLOW_LINE = 1
ENTER_PARKING = 2
EXIT_PARKING = 3

def find_line():
    while True:
        found_line = True
        if found_line: #We found the line yippie, or a value close enough to the line!
            return FOLLOW_LINE #Start following the line

    return FIND_LINE #We should never get here but just go back to find_line if we do

def follow_line():
    while True:
        #Follow line here, same loop as before or maybe even better :o
        found_parking_line = False

        if found_parking_line: #We detected a parking-line with our other sensor
            return ENTER_PARKING

    return FIND_LINE #We should never get here but lets just go back to find_line if we do

def enter_parking():
    while True:
        #Check if occupied and if we have checked the whole parking space
        is_occupied = False
        checked_whole_parkingspot = False

        if is_occupied: #We detected another robot in the parking spot
            #Reverse our inputs trying to get in so we're back to the same spot we started at when we entered
            #If this isn't easy to do, maybe just return FIND_LINE
            #Maybe set a timer/distance until we can detect parking-lines again?
            return FOLLOW_LINE
        elif checked_whole_parkingspot: #We have finished checking the whole parkingspot, it should be available
            #enter the parkingspot
            wait(5000) #Wait 5 seconds
            return EXIT_PARKING
        
    return FIND_LINE #We should never get here but lets just go back to find_line if we do

def exit_parking():
    while True:
        #Exit the parkingspot
        exited_parkingspot = False

        if exited_parkingspot: #We have exited the parkingspot and we're back to the same spot we started at when we entered
            #If this isn't easy to do, maybe just return FIND_LINE
            return FOLLOW_LINE
        
    return FIND_LINE #We should never get here but lets just go back to find_line if we do

state = FIND_LINE #We start with finding the line

#Main loop, this is what we run all the time
while True:
    print("Current State:", state)

    if state == FIND_LINE:
        state = find_line()
    elif state == FOLLOW_LINE:
        state = follow_line()
    elif state == ENTER_PARKING:
        state = enter_parking()
    elif state == EXIT_PARKING:
        state = exit_parking()
    else:
        print("Our state value is not defined!")