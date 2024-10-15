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
parking_sensor = ColorSensor(Port.S2) #left
follow_sensor = ColorSensor(Port.S3) #right
distance_sensor = UltrasonicSensor(Port.S4) 

"""
Important! The code may not function correctly if the first line from the template is not included.
"""

# Your code goes here

#Other Variables
line_reflection = 6
background_reflection = 26

#Creating target value, this will be the value when the line/background both share the same amount of the sensor
target_value = (line_reflection + background_reflection) * 0.5

#Abs amount from line reflection -> target and background reflection -> target
difference = target_value - line_reflection

rotation_speed = 25.0
drive_speed = 25.0

#States
FIND_LINE = 0
FOLLOW_LINE = 1
ENTER_PARKING = 2
EXIT_PARKING = 3

#Functions
#Returns true if there is something closer than X mm infront of the robot, else false
def blocked_ahead():
    return distance_sensor.distance() <= 130

#Returns true if the sensor reflection is close enough to the line reflection
def sensor_on_line(sensor:ColorSensor):
    return sensor.reflection() <= (line_reflection + 2)

def get_percentage_to_target_reflection(sensor:ColorSensor):
    reflection = sensor.reflection()

    #Because we do 'reflection - target_value' we are following the left side of the line
    offset_to_target = reflection - target_value

    #Convert offset to a value between -1.0 -> 1.0
    percentage = offset_to_target / difference
    return percentage

def find_line():
    distance_to_drive = 100
    rotation_index = 0
    while True:
        robot.turn(90)
        robot.stop()
        robot.drive(drive_speed, 0)
        driven_amount = 0

        while(driven_amount < distance_to_drive):
            wait(10)
        
            if(sensor_on_line(follow_sensor)): # + 2 because its close enough to the line. Probably found the line
                return FOLLOW_LINE #Start following the line :)

            driven_amount += 1 #We're driving 100mm/s -> 100mm/1000ms. We drive for 10 ms = 1mm
        
        rotation_index += 1
        if(not rotation_index & 1): #Increases the distance to drive each time we do a 180 spint (2 rotations) and still haven't found the line
            distance_to_drive += 50 #This is to expand the 'square' the robot is driving in. This value should probably be something related to the robot size.

    return FIND_LINE #We should never get here but just go back to find_line if we do

def follow_line():
    while True:
        if blocked_ahead(): #If something is blocking us, we shouldn't do anything
            robot.stop()
            continue

        rotation_percentage = get_percentage_to_target_reflection(follow_sensor)

        if(abs(rotation_percentage) > 0.5): #Large rotation, stop moving forward until we get closer to the line again
            robot.drive(0, rotation_speed * rotation_percentage)
        else:
            robot.drive(drive_speed, rotation_speed * rotation_percentage) 

        if(sensor_on_line(parking_sensor)): #We detected a parking-line with our other sensor
            return ENTER_PARKING

    return FIND_LINE #We should never get here but lets just go back to find_line if we do

def enter_parking():
    robot.stop() #else it would just keep driving with last settings. Just temp
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
    robot.drive(-drive_speed, 0) #Start backing out of the parking

    while True:
        percentage = get_percentage_to_target_reflection(follow_sensor)

        if(percentage < 0.7): #We should be just reaching the line again since 1.0 == only background
            robot.stop()
            return FOLLOW_LINE #Because we are close to the line but not on it, we should immediately rotate right whilst not moving

    return FIND_LINE #We should never get here but lets just go back to find_line if we do
    

state = EXIT_PARKING #We start with finding the line

# while True:
#     print(follow_sensor.reflection())

#Main loop, this is what we run all the time
# while True:
#     print("Current State:", state)

#     if state == FIND_LINE:
#         state = find_line()
#     elif state == FOLLOW_LINE:
#         state = follow_line()
#     elif state == ENTER_PARKING:
#         state = enter_parking()
#     elif state == EXIT_PARKING:
#         state = exit_parking()
#     else:
#         print("Our state value is not defined!")
