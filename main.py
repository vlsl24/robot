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
parking_sensor = ColorSensor(Port.S3) #right
follow_sensor = ColorSensor(Port.S2) #left
distance_sensor = UltrasonicSensor(Port.S4) 

"""
Important! The code may not function correctly if the first line from the template is not included.
"""

# Your code goes here

#Parking variables
parking_spots = 3
parking_index = 0
parking_states = list()

PARKING_FREE = 0
PARKING_BUSY = 1
PARKING_NOT_VISISTED = 2

first_lap = True

parking_states = [PARKING_NOT_VISISTED] * parking_spots

#Other Variables
line_reflections = [7, 6]
background_reflections = [95, 87]

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
#Draw parking states
def draw_parking_state(index:int, x:int, y:int, width:int, height:int):
    text = str(index + 1) + ". " + parkingstate_to_name(parking_states[index])
    if index == parking_index:
        text += "<--"
    ev3.screen.draw_text(x, y, text)

def update_screen():
    ev3.screen.clear()

    height_per_entry = round(ev3.screen.height / parking_spots)

    for i in range(0, parking_spots):
        draw_parking_state(i, 3, height_per_entry * i, ev3.screen.width, height_per_entry)

def parkingstate_to_name(parkingstate:int):
    if parkingstate == PARKING_FREE:
        return "[  ]"
    elif parkingstate == PARKING_BUSY:
        return "[X]"
    elif parkingstate == PARKING_NOT_VISISTED:
        return "[?]"

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

    #Get the correct index because the sensors might have different heights or other reason for getting different values from the sensor
    index = 0
    if sensor == parking_sensor:
        index = 1

    #Because we do 'reflection - target_value' we are following the left side of the line
    offset_to_target = reflection - target_values[index]

    #Convert offset to a value between -1.0 -> 1.0
    percentage = offset_to_target / differences[index]

    #Clamp the value just in case
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
                left_motor.run(100)
                while get_percentage_to_target_reflection(follow_sensor) < 0: #Try to rotate left until we come back to the background.
                    #This should land us on the correct side of the line
                    pass
                return FOLLOW_LINE

        rotation_index += 1
        if(not rotation_index & 1): #Increases the distance to drive each time we do a 180 spin (2 rotations) and still haven't found the line
            distance_to_drive += 50 #This is to expand the 'square' the robot is driving in. This value should probably be something related to the robot size.

    return FIND_LINE #We should never get here but just go back to find_line if we do

def follow_line():
    global parking_index

    #Find-parking states
    FIND_PARKING_LINE = 0
    LINE_UP_WITH_PARKING = 1
    FIND_BACKGROUND = 2

    find_parking_state = FIND_PARKING_LINE
    while True:
        cruise_control_percentage = cruise_controll()

        rotation_percentage = -get_percentage_to_target_reflection(follow_sensor)

        if(find_parking_state == FIND_PARKING_LINE):
            if(get_percentage_to_target_reflection(parking_sensor) < -0.5): #We detected a full-parking-line with our other sensor
                if first_lap or parking_states[parking_index] == PARKING_FREE:
                    find_parking_state = LINE_UP_WITH_PARKING
                    robot.reset() #To measure distance when lining up.
                    print("Entered LINE_UP_WITH_PARKING")
                else:
                    # Parking spot busy
                    ev3.speaker.beep(500, 100)
                    parking_index += 1
                    clamp_parking_index()
                    update_screen()

                    find_parking_state = FIND_BACKGROUND
                    print("Parking spot busy :(")
        elif find_parking_state == LINE_UP_WITH_PARKING:
            park_distance = 185
            if(robot.distance() > park_distance): #Should be lined up :)
                return on_found_parking()
            if (robot.distance() > park_distance * 0.8 and rotation_percentage < 0): #Do not rotate after driving half the park space
                rotation_percentage = 0
        elif(find_parking_state == FIND_BACKGROUND):
            if(get_percentage_to_target_reflection(parking_sensor) > 0.5):
                find_parking_state = FIND_PARKING_LINE

        abs_rotation = abs(rotation_percentage)

        if(abs_rotation > 0.8): #Large rotation, stop moving forward until we get closer to the line again
            robot.drive(0, rotation_speed * rotation_percentage * cruise_control_percentage)
        else:
            robot.drive(drive_speed * cruise_control_percentage * (1.0 - abs_rotation), rotation_speed * rotation_percentage) 

    return FIND_LINE #We should never get here but lets just go back to find_line if we do

def clamp_parking_index():
    global parking_index
    global parking_spots

    if parking_index >= parking_spots:
        parking_index = 0

def on_found_parking():
    global parking_index

    current_index = parking_index
    parking_index += 1
    clamp_parking_index()

    park_distance = 275
    if first_lap:
        rotate_to_parking_spot()
        parking_states[current_index] = PARKING_BUSY if blocked_ahead(park_distance) else PARKING_FREE
        update_first_lap_status()
        return EXIT_PARKING
    else:
        if parking_states[current_index] == PARKING_FREE:
            clamp_parking_index()
            rotate_to_parking_spot()
            return ENTER_PARKING
    return FOLLOW_LINE

def rotate_to_parking_spot():
    robot.stop()
    left_motor.run_angle(drive_speed , 0)
    right_motor.run_angle(drive_speed , -310) #Doesnt do anything right now

def update_first_lap_status():
    global first_lap

    # Checks if there are any parking spots yet to be visited.
    # If we have not visited all parking spots, we are still on the first lap.
    for i in range(0, parking_spots):
        if parking_states[i] == PARKING_NOT_VISISTED:
            return
    first_lap = False

def enter_parking():
    robot.reset()
    robot.drive(drive_speed , 0)
    park_distance = 75 #275
    while robot.distance() < park_distance:
        
        #Check if we are too close to the parking lines and adjust
        if get_percentage_to_target_reflection(follow_sensor) < 0:
            robot.stop()
            left_motor.run_angle(drive_speed, 20)
            robot.drive(drive_speed, 0)

        if get_percentage_to_target_reflection(parking_sensor) < 0:
            robot.stop()
            right_motor.run_angle(drive_speed, 20)
            robot.drive(drive_speed, 0)
            
    robot.stop()
    wait(5000) # Wait 5 sec

    return EXIT_PARKING

def exit_parking():
    robot.drive(-drive_speed, 0) #Start reversing out of the parking

    while True:
        percentage = get_percentage_to_target_reflection(follow_sensor)

        if(percentage < -0.5): #We should be just reaching the line again since 1.0 == only background, but we don't want to overshoot so we start early
            robot.stop()

            # Here we drive adjust to robot at bit to make sure the parking sensor is on the "correct" side of the line.
            robot.straight(10)
            robot.stop()
            right_motor.run_angle(100, 45)

            update_screen()
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

update_screen()
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
