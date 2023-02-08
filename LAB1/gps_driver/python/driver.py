#!/usr/bin/env/ python3
# -*- coding: utf-8 -*-

import math
import serial
import utm
import rospy
from gps_driver.msg import gps_msg
import argparse
import sys


if __name__ == '__main__':
    SENSOR_NAME = "gps"
    rospy.init_node('gps_driver')
    
    
    
    
    
    #gps_parser = argparse.ArgumentParser(description='Get path to GPS puck serial port:')
    #gps_parser.add_argument('port_path', type=str, help='serial port path, Ex. /dev/pts/1')
    #args = gps_parser.parse_args()
   
    #serial_port = rospy.get_param('~port', args.port_path)
    args = rospy.myargv(argv=sys.argv)
    print(args)
    
    serial_port = args[1] #rospy.get_param('~port')    #"/dev/pts/2"
    print(serial_port)
   
    
    
    
    
    #location_of_port = rospy.get_param('/driver/port')
    
    
    
    
    serial_baud = 4800
    #sampling_rate = rospy.get_param('~sampling_rate', 5.0)
    port = serial.Serial(serial_port, serial_baud, timeout=3.0)
    
    #sampling_count = int(round(1/(sampling_rate*0.007913)))
    
    pub = rospy.Publisher('/gps', gps_msg, queue_size = 10)
    
    #rospy.sleep(0.2)
    line = port.readline()
    
    #sleep_time = 1/sampling_rate - 0.025
    
    msg = gps_msg()
    msg.Header.frame_id = "GPS1_Frame"
    
    counter = 0
    
    while not rospy.is_shutdown(): #True:
        
        line = port.readline().decode("utf-8").strip()
        
        # line = "$GPGGA,184102.000,4158.3952,N,08754.0118,W,1,07,1.1,169.2,M,-34.1,M,,0000*63"
        if line.startswith("$GPGGA"):
            foundGPGGA = line.split(',')
            
            universal_time_clock = foundGPGGA[1]
            print("universaltimeclock")
            print(universal_time_clock)
            
            UTC_hh = int(universal_time_clock[0:2])
            UTC_hh = int(universal_time_clock[0:2])
            print("UTC_hh")
            print(UTC_hh)
            UTC_hh_secs = UTC_hh * 3600
            UTC_mm = int(universal_time_clock[2:4])
            print("UTC_mm")
            print(UTC_mm)
            UTC_mm_secs = UTC_mm * 60
            
            
            UTC_ss_secs = int(universal_time_clock[4:6])
            print("UTC_ss")
            print(UTC_ss_secs)
            
            UTM_secs = UTC_hh_secs + UTC_mm_secs + UTC_ss_secs
            
            UTC_decisecs = universal_time_clock[7:9]
            print("UTC_decisecs")
            UTC_decisecs = float(universal_time_clock[7:9])
            print("UTC_decisecs")
            print(UTC_decisecs)
            
            UTM_nanosecs = int(UTC_decisecs * (10**8))
        
        
            print("UTM_secs")
            print(UTM_secs)
            print("UTM_nanosecs")
            print(UTM_nanosecs)
            
            
            
            latitude = foundGPGGA[2]
            print("latitude")
            print(latitude)
            latitudeDD = float(latitude[0:2])
            print("latitude_DD")
            print(latitudeDD)
            latitude_mins = float(latitude[2:])
            print("latitude_mins")
            print(latitude_mins)
            latitude_decimal = latitudeDD + (latitude_mins / 60)
            print("latitude_decimal")
            print(latitude_decimal)
            
            
            latitude_direction = foundGPGGA[3]
            print("latitude direction")
            print(str(latitude_direction))
            if latitude_direction == 'S':
                latitude_decimal = -(latitude_decimal)
            print("latitude decimal")
            print(latitude_decimal)
            
            
            
            
            
            
            longitude = foundGPGGA[4]
            print("longitude")
            print(longitude)
            longitudeDDD = float(longitude[0:3])
            print("longitudeDDD")
            print(longitudeDDD)
            longitude_mins = float(longitude[3:])
            print("longitude_mins")
            print(longitude_mins)
            longitude_decimal = longitudeDDD + (longitude_mins / 60)
            print("longitude_decimal")
            print(longitude_decimal)
            
            
            longitude_direction = foundGPGGA[5]
            print("longitude direction")
            print(longitude_direction)
            if longitude_direction == 'W':
                longitude_decimal = (-longitude_decimal)  #check if this negative sign works
            print("longitude decimal")
            print(longitude_decimal)
            
            
            utm_vals = utm.from_latlon(latitude_decimal, longitude_decimal)
            print("UTM")
            print(utm_vals)
            
            
            
            number_satellites = foundGPGGA[7]
            print("numsats")
            print(number_satellites)
            
            
            
            horizontal_precision_dilution = foundGPGGA[8]
            print("hdop")
            print(horizontal_precision_dilution)
            horizontal_precision_dilution = float(horizontal_precision_dilution)
            
            
            altitude = foundGPGGA[9]
            print("altitude")
            print(altitude)
            altitude = float(altitude)
            
            msg.Header.stamp.secs = UTM_secs
            msg.Header.stamp.nsecs = UTM_nanosecs
            msg.Header.seq = counter
            
            msg.Latitude = latitude_decimal
            msg.Longitude = longitude_decimal
            msg.Altitude = altitude
            msg.HDOP = horizontal_precision_dilution
            msg.UTM_easting = utm_vals[0]
            msg.UTM_northing = utm_vals[1]
            msg.UTC = universal_time_clock
            msg.Zone = utm_vals[2]
            msg.Letter = utm_vals[3]
            msg.GPGGA = line
            
            
            counter = counter + 1
            
            rospy.loginfo(msg)
            pub.publish(msg)
            
            
            
            
            
            '''
            print(foundGPGGA)

            universal_time_clock = foundGPGGA[7:17]
            print(universal_time_clock)
            UTC_hh = int(universal_time_clock[0:2])
            print("UTC_hh")
            print(UTC_hh)
            UTC_hh_secs = UTC_hh * 3600
            UTC_mm = int(universal_time_clock[2:4])
            print("UTC_mm")
            print(UTC_mm)
            UTC_mm_secs = UTC_mm * 60
            
            
            UTC_ss_secs = int(universal_time_clock[4:6])
            print("UTC_ss")
            print(UTC_ss_secs)
            
            UTM_secs = UTC_hh_secs + UTC_mm_secs + UTC_ss_secs
            
            UTC_decisecs = universal_time_clock[7:9]
            print("UTC_decisecs")
            UTC_decisecs = float(universal_time_clock[7:9])
            print("UTC_decisecs")
            print(UTC_decisecs)
            
            UTM_nanosecs = int(UTC_decisecs * (10**8))
            
            print("UTM_secs")
            print(UTM_secs)
            print("UTM_nanosecs")
            print(UTM_nanosecs)
            

            latitude = foundGPGGA[17:27]
            print("latitude")
            print(latitude)
            latitudeDD = float(latitude[0:2])
            print("latitude_DD")
            print(latitudeDD)
            latitude_mins = float(latitude[2:])
            print("latitude_mins")
            print(latitude_mins)
            latitude_decimal = latitudeDD + (latitude_mins / 60)
            print("latitude_decimal")
            print(latitude_decimal)
            latitude_direction = foundGPGGA[28:29]
            print("latitude direction")
            print(str(latitude_direction))
            if latitude_direction == 'S':
                latitude_decimal = latitude_decimal * (0-1)
            
            print("latitude decimal")
            print(latitude_decimal)


            longitude = foundGPGGA[30:41]
            print("longitude")
            print(longitude)
            longitudeDDD = float(longitude[0:3])
            print("longitudeDDD")
            print(longitudeDDD)
            longitude_mins = float(longitude[3:])
            print("longitude_mins")
            print(longitude_mins)
            longitude_decimal = longitudeDDD + (longitude_mins / 60)
            print("longitude_decimal")
            print(longitude_decimal)
            longitude_direction = foundGPGGA[42:43]
            print("longitude direction")
            print(longitude_direction)
            if longitude_direction == 'W':
                longitude_decimal = (-longitude_decimal)  #check if this negative sign works
            print("longitude decimal")
            print(longitude_decimal)

            utm_vals = utm.from_latlon(latitude_decimal, longitude_decimal)
            print("UTM")
            print(utm_vals)


            number_satellites = foundGPGGA[46:48]
            print("numsats")
            print(number_satellites)

            horizontal_precision_dilution = foundGPGGA[49:53]
            print("hdop")
            print(horizontal_precision_dilution)
            
            horizontal_precision_dilution = float(horizontal_precision_dilution)

            altitude = foundGPGGA[52:57]
            print("altitude")
            print(altitude)
            altitude = float(altitude)
            
            msg.Header.stamp.secs = UTM_secs
            msg.Header.stamp.nsecs = UTM_nanosecs
            msg.Header.seq = counter
            
            msg.Latitude = latitude_decimal
            msg.Longitude = longitude_decimal
            msg.Altitude = altitude
            msg.HDOP = horizontal_precision_dilution
            msg.UTM_easting = utm_vals[0]
            msg.UTM_northing = utm_vals[1]
            msg.UTC = universal_time_clock
            msg.Zone = utm_vals[2]
            msg.Letter = utm_vals[3]
            msg.GPGGA = line
            
            
            counter = counter + 1
            
            
            rospy.loginfo(msg)
            pub.publish(msg)
            
            #pub.publish
            '''
            
            
            
            
                        
            
        #rospy.sleep(sleep_time)    






















































'''
import rospy
import serial
#import utm


if __name__ == '__main__':
    SENSOR_NAME = "emulator"
    rospy.init_node('WeCanDoIt')
    serial_port = rospy.get_param('~port','/dev/pts/1')
    serial_baud = 4800
    sampling_rate = 1
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    
    
    
    
 
    while True:
        line = port.readline()
        if line.startswith(b"$GPGGA"):
            foundGPGGA = line
            print(foundGPGGA)
            
            universal_time_clock = foundGPGGA[7:15]
            print(universal_time_clock)
            # need to convert the above to usable time for ROS
            # by using the time library
          
            
            latitude = foundGPGGA[17:25]
            print(latitude)
     
            latitude_direction = foundGPGGA[27]
            
            longitude = foundGPGGA[29:38]
            
            longitude_direction = foundGPGGA[40]
            
            number_satellites = foundGPGGA[44:45]
            
            horizontal_precision_dilution = foundGPGGA[47:49]
            
            altitude = int(foundGPGGA[51:57])
    '''
  
            
            
