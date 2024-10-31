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
line_reflections = [5, 6]
background_reflections = [23, 26]

#Creating target value, this will be the value when the line/background both share the same amount of the sensor
target_values = [(line_reflections[0] + background_reflections[0]) * 0.5, (line_reflections[1] + background_reflections[1]) * 0.5] 

#Abs amount from line reflection -> target and background reflection -> target
differences = [target_values[0] - line_reflections[0], target_values[1] - line_reflections[1]]

rotation_speed = 20.0
drive_speed = 65.0

#States
FIND_LINE = 0
FOLLOW_LINE = 1
ENTER_PARKING = 2
EXIT_PARKING = 3
EXIT_PROGRAM = 4

#Functions
def cruise_controll():
    maxDist =200
    minDist =150
    diff = maxDist - minDist
    
    if (distance_sensor.distance() < maxDist):
        if (distance_sensor.distance() <= minDist):
            return 0.0
        percentage = (distance_sensor.distance() - minDist) / diff
        percentage *= percentage
        
        return float(percentage)

    return 1.0
    
#Returns true if there is something closer than distance mm infront of the robot, else false
def blocked_ahead(distance):
    return distance_sensor.distance() <= distance

#Returns true if the sensor reflection is close enough to the line reflection
def sensor_on_line(sensor:ColorSensor):
    return get_percentage_to_target_reflection(sensor) <= -1.0

def get_percentage_to_target_reflection(sensor:ColorSensor):
    reflection = sensor.reflection()

    index = 0
    if sensor == parking_sensor:
        index = 1

    #Because we do 'reflection - target_value' we are following the left side of the line
    offset_to_target = reflection - target_values[index]

    #Convert offset to a value between -1.0 -> 1.0
    percentage = offset_to_target / differences[index]

    if percentage > 1.0:
        percentage = 1.0
    elif percentage < -1.0:
        percentage = -1.0
    return percentage

def find_line():
    distance_to_drive = 100
    rotation_index = 0
    while True:
        robot.turn(90)
        robot.stop()
        robot.reset()
        robot.drive(drive_speed, 0)
        
        while robot.distance() < distance_to_drive:
            cruise_percentage = cruise_controll()
            robot.drive(drive_speed * cruise_percentage, 0)
            if sensor_on_line(follow_sensor):
                print("Found line")
                robot.stop()
                left_motor.run(-100)
                while get_percentage_to_target_reflection(follow_sensor) < 0: #Try to rotate left until we come back to the background.
                    #This should land us on the correct side of the line
                    pass
                return FOLLOW_LINE

        rotation_index += 1
        if(not rotation_index & 1): #Increases the distance to drive each time we do a 180 spin (2 rotations) and still haven't found the line
            distance_to_drive += 50 #This is to expand the 'square' the robot is driving in. This value should probably be something related to the robot size.

    return FIND_LINE #We should never get here but just go back to find_line if we do

def follow_line():
    #Find-parking states
    FIND_FIRST_LINE = 0
    FIND_SECOND_LINE = 1
    FIND_BACKGROUND = 2

    find_parking_state = FIND_FIRST_LINE
    while True:
        cruise_control_percentage = cruise_controll()

        rotation_percentage = get_percentage_to_target_reflection(follow_sensor)

        abs_rotation = abs(rotation_percentage)
        if(abs_rotation > 0.9): #Large rotation, stop moving forward until we get closer to the line again
            robot.drive(0, rotation_speed * rotation_percentage * cruise_control_percentage)
        else:
            robot.drive(drive_speed * cruise_control_percentage * (1.0 - abs_rotation), rotation_speed * rotation_percentage) 

        if(find_parking_state == FIND_FIRST_LINE):
            if(sensor_on_line(parking_sensor)): #We detected a full-parking-line with our other sensor
                find_parking_state = FIND_BACKGROUND
                print("Entered FIND_BACKGROUND")
        elif(find_parking_state == FIND_BACKGROUND):
            if(get_percentage_to_target_reflection(parking_sensor) > 0.5):
                find_parking_state = FIND_SECOND_LINE
                print("Entered FIND_SECOND_LINE")
        elif(find_parking_state == FIND_SECOND_LINE):
            if(get_percentage_to_target_reflection(parking_sensor) < -0.5):
                return ENTER_PARKING

    return FIND_LINE #We should never get here but lets just go back to find_line if we do

def enter_parking():
    robot.drive(drive_speed, 0)
    wait((20/drive_speed)*1000) #Drive 20 mm, to line up better with the parking spot
    robot.stop()
    left_motor.run_angle(drive_speed , -360)
    right_motor.run_angle(drive_speed , 0) #Doesnt do anything right now
    
    robot.reset()
    robot.drive(drive_speed , 0)
    park_distance = 275
    while robot.distance() < park_distance: #Bör vara runt 250
        
        #We might want to use both sensors to check if we are too close to the lines
        #Or just correct our rotation if it doesn't line up properly

        if get_percentage_to_target_reflection(follow_sensor) < 0:
            robot.stop()
            right_motor.run_angle(drive_speed, 20)
            robot.drive(drive_speed, 0)

        if get_percentage_to_target_reflection(parking_sensor) < 0:
            robot.stop()
            left_motor.run_angle(drive_speed, 20)
            robot.drive(drive_speed, 0)

        if blocked_ahead(park_distance - robot.distance()): #Kollar alltid samma distance såhär
            return EXIT_PARKING
            
    robot.stop()
    wait(5000)

    return EXIT_PARKING

def exit_parking():
    robot.drive(-drive_speed, 0) #Start reversing out of the parking

    while True:
        percentage = get_percentage_to_target_reflection(follow_sensor)

        if(percentage < -0.5): #We should be just reaching the line again since 1.0 == only background, but we don't want to overshoot so we start early
            robot.stop()
            robot.straight(10)
            robot.stop()
            left_motor.run_angle(100, 45)
            return FOLLOW_LINE #Because we are close to the line but not on it, we should immediately rotate right whilst not moving

    return FIND_LINE #We should never get here but lets just go back to find_line if we do
    
def get_state_name(state:int):
    if state == FIND_LINE:
        return "FIND_LINE"
    elif state == FOLLOW_LINE:
        return "FOLLOW_LINE"
    elif state == ENTER_PARKING:
        return "ENTER_PARKING"
    elif state == EXIT_PARKING:
        return "EXIT_PARKING"
    return "UNKNOWN_STATE"

state = FOLLOW_LINE #We start with finding the line

# while True:
#     print(follow_sensor.reflection(), parking_sensor.reflection())

#Main loop, this is what we run all the time
while state != EXIT_PROGRAM:
    print("Current State:", get_state_name(state))

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
