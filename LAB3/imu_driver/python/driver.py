#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import math as m
import serial
import utm
import rospy
from imu_driver.msg import Vectornav
import argparse
import sys
# check package.xml aqnd CMakeLists for edits needed to recognize message file

if __name__ == '__main__':
    SENSOR_NAME = "imu"
    rospy.init_node('imu_driver')
        
       
    args = rospy.myargv(argv=sys.argv)
    #print(args)
    
    serial_port = args[1] 
    #print(serial_port)
   
    
    
    
    
    
    
    
    
    
    serial_baud = 115200
 
    port = serial.Serial(serial_port, serial_baud, timeout=3.0)
    
    # specifying output rate for IMU device
    outputratecommand = str("$VNWRG,07,40*XX")
    port.write(outputratecommand.encode())
  
    
    pub = rospy.Publisher('imu/', Vectornav, queue_size = 10) #correct topic name?--need to
    # confirm topic title, need to resolve not recognizing message issue
    
    line = port.readline()
    
        
    msg = Vectornav()
    
    msg.Header.frame_id = "imu1_frame"
    
    counter = 0
    
    while not rospy.is_shutdown(): #True:
        
        line = port.readline().decode("utf-8").strip() # keep getting /x00 in strings. Since Vnav is confiugred, delimiting by ',', and decoding appropriately, its pry because my computer is slow. Advised to reduce sleep time and throw out string and grab next one
        #line = line.encode("utf-8")
        #line = line.decode()
        
        if line.startswith("$VNYMR"): # need to doublecheck that the list of split string is in same order as GPGGA
            foundVNYMR = line.split(',')
            #print(line)
            
            if len(foundVNYMR) == 13:
                print("THIS IS THE LINE")
                print(line)
                #below two lines remove the checksum portion at end of VNYMR string from last element in foundVNYMR
                VNYMRremovedchecksum = foundVNYMR[12].split('*')
                foundVNYMR[12] = VNYMRremovedchecksum[0]
                
                #print("found it!")
                #print(foundVNYMR)
                #print("length: ", len(foundVNYMR))
                # take foundVNYMR, put values into variables, use these and service to convert to quaternion, convert back to quaternion, and store one/both(?) into messag
                
                
                
                thetime = rospy.get_rostime()
                try:
                    # yaw, pitch, roll units, initially in degrees, converted to radians
                    yaw = float(foundVNYMR[1]) * m.pi/180
                    pitch = float(foundVNYMR[2]) * m.pi/180
                    roll = float(foundVNYMR[3]) * m.pi/180
                    
                    # magnetometer x,y,z, initially in Gauss, converted to Teslas (1 Gauss = .0001 Tesla)
                    MagX = float(foundVNYMR[4]) * 0.0001
                    MagY = float(foundVNYMR[5]) * 0.0001
                    MagZ = float(foundVNYMR[6]) * 0.0001
                    
                    # accelerometer x,y,x, in m/s^2
                    AccelX = float(foundVNYMR[7])
                    AccelY = float(foundVNYMR[8])
                    AccelZ = float(foundVNYMR[9])
                    
                    # gyroscope x,y,z initially in rad/s
                    GyroX = float(foundVNYMR[10])
                    GyroY = float(foundVNYMR[11])
                    GyroZ = float(foundVNYMR[12])
                except:
                    continue
                else:
                    # yaw, pitch, roll units, initially in degrees, converted to radians
                    yaw = float(foundVNYMR[1]) * m.pi/180
                    pitch = float(foundVNYMR[2]) * m.pi/180
                    roll = float(foundVNYMR[3]) * m.pi/180
                    
                    # magnetometer x,y,z, initially in Gauss, converted to Teslas (1 Gauss = .0001 Tesla)
                    MagX = float(foundVNYMR[4]) * 0.0001
                    MagY = float(foundVNYMR[5]) * 0.0001
                    MagZ = float(foundVNYMR[6]) * 0.0001
                    
                    # accelerometer x,y,x, in m/s^2
                    AccelX = float(foundVNYMR[7])
                    AccelY = float(foundVNYMR[8])
                    AccelZ = float(foundVNYMR[9])
                    
                    # gyroscope x,y,z initially in rad/s
                    GyroX = float(foundVNYMR[10])
                    GyroY = float(foundVNYMR[11])
                    GyroZ = float(foundVNYMR[12])
                
                    # finding the x,y,z, and w of the quaternion based on the euler angles (roll, pitch, yaw)
                    quatx = (m.sin(roll/2) * m.cos(pitch/2) * m.cos(yaw/2)) - (m.cos(roll/2) * m.sin(pitch/2) * m.sin(yaw/2))
                    quaty = (m.cos(roll/2) * m.sin(pitch/2) * m.cos(yaw/2)) + (m.sin(roll/2) * m.cos(pitch/2) * m.sin(yaw/2))
                    quatz = (m.cos(roll/2) * m.cos(pitch/2) * m.sin(yaw/2)) - (m.sin(roll/2) * m.sin(pitch/2) * m.cos(yaw/2))
                    quatw = (m.cos(roll/2) * m.cos(pitch/2) * m.cos(yaw/2)) + (m.sin(roll/2) * m.sin(pitch/2) * m.sin(yaw/2))

                
                
                
                    msg.Header.stamp.secs = thetime.secs
                    msg.Header.stamp.nsecs = thetime.nsecs
                    msg.Header.seq = counter
                    
                    msg.imu.orientation.x = quatx
                    msg.imu.orientation.y = quaty
                    msg.imu.orientation.z = quatz
                    msg.imu.orientation.w = quatw
                    msg.imu.angular_velocity.x = GyroX
                    msg.imu.angular_velocity.y = GyroY
                    msg.imu.angular_velocity.z = GyroZ
                    msg.imu.linear_acceleration.x = AccelX
                    msg.imu.linear_acceleration.y = AccelY
                    msg.imu.linear_acceleration.z = AccelZ
                    
                    msg.mag_field.magnetic_field.x = MagX
                    msg.mag_field.magnetic_field.y = MagY
                    msg.mag_field.magnetic_field.z = MagZ
                    
                    
                    msg.line = line
                    
                    
                    counter = counter + 1
                    
                    rospy.loginfo(msg)
                    pub.publish(msg)
        #rospy.sleep(1/80) # sleeping for 1/80 s (instead of 1/40 since my computers slow and need to filter bd strings)



