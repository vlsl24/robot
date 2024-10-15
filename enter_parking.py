#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
ultrasonic= UltrasonicSensor(Port.S4)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 35

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
# PROPORTIONAL_GAIN = 1.2

# def follow_line():
#     if line_sensor.reflection <= 9:
#         enter_parking()
#     return
# def exit_parking():
    # roboten kör ut från parkering
    # return

def enter_parking():
    robot.drive(DRIVE_SPEED, 0)
    wait((20/DRIVE_SPEED)*1000)
    robot.stop()
    left_motor.run_angle(DRIVE_SPEED , -360)
    right_motor.run_angle(DRIVE_SPEED , 0)
    
    robot.reset()
    robot.drive(DRIVE_SPEED , 0)
    while robot.distance() < 250:

        if ultrasonic.distance()<300:
            print("exit")
            
    robot.stop()
    wait(5000)
    print("exit")
    


    #     else:    
    #         left_motor.run_angle(DriveBase , 0)
    #         right_motor.run_angle(DriveBase , 0)
            

    # distance =1
    # while distance<5:
    #     left_motor.run_angle(15 , 100)
    #     right_motor.run_angle(15 , 100)
    #     distance+=5
    # wait(3000) 
    # exit_parking()
             




# Start following the line endlessly.
# while True:
    # Calculate the deviation from the threshold.
    # deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    # turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    # robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    # wait(10)

if __name__=="__main__":
    enter_parking()    




