def cruise_controll():
    maxDist =250
    minDist =100
    
    while (distance_sensor.distance < maxDist):
        percentage = distance_sensor.distance / minDist
        robot.drive(drive_speed * percentage, rotation_speed)
        
        if (distance_sensor.distance < minDist):
          robot.drive(0,0)
    
    
