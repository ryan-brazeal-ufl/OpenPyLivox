#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

Started on Mon. May 13th 2019

@author: Ryan Brazeal
@email: ryan.brazeal@ufl.edu

Program Name: livox_controller_demo.py
Version: 1.0.2

Description: Python3 demo for controlling a single or multiple Livox Mid-40 sensor(s) using openpylivox


"""

#openpylivox library import
import openpylivox as opl

#only used for this demo
import time


#demo operations for a single Livox Mid-40 Sensor
def singleSensorDemo():
    
    #create an openpylivox object
    sensor = opl.openpylivox(True)   #optional True/False, is used to have sensor messages printed to screen (default = False)      
    
    #the sensor object's showMessages properties can be turned on/off throughout program execution (example further down)
    sensor.showMessages = False
    sensor.showMessages = True
    
    #sensor object's showMessages property can always be reset back to what it was initially set as, at the time of instantiation using
    sensor.resetShowMessages()
          
    #easiest to try to automatically set the openpylivox sensor connection parameters and connect
#    connected = sensor.auto_connect()
    
    #or if your computer has multiple IP address and the correct IP is not being automatically determined
    #you can force the computer IP to a manual address
    connected = sensor.auto_connect("192.168.1.20")

    #or manually define all IP addresses and ports (still need to properly configure your IP, Subnet, etc. on your computer)
                             #  computer IP       sensorIP    data port  command port
#    connected = sensor.connect("192.168.1.20", "192.168.1.42",  60001,     50001)
    
    #make sure a sensor was connected
    if connected:
    
        #the sensor's connection parameters can be returned as a list of strings
        connParams = sensor.connectionParameters()
        
        #the sensor's firmware version can be returned as a string
        firmware = sensor.firmware()
        
        #the sensor's serial number can be returned as a string
        serial = sensor.serialNumber()
        
        sensor.showMessages = False
        
        #set the output coordinate system to Spherical
#        sensor.setSphericalCS()
        
        #set the output coordinate system to Cartesian
        sensor.setCartesianCS()
        
        #read current extrinsic values from the sensor
        sensor.readExtrinsic()
        
        #set all the sensor's extrinsic parameters equal to zero
        #(*** IMPORTANT: does not affect raw point cloud data stream, seems to only be used in Livox-Viewer? ***)
        sensor.setExtrinsicToZero()
                
        x = 12.345      #units of meters
        y = -23.456     #units of meters
        z = 34.567      #units of meters
        roll = -0.05    #units of degrees
        pitch = 8.77    #units of degrees
        yaw = -174.14   #units of degrees
    
        #set the sensor's extrinsic parameters to specific values
        #(*** IMPORTANT: does not affect raw point cloud data stream, seems to only be used in Livox-Viewer? ***)
#        sensor.setExtrinsicTo(x, y, z, roll, pitch, yaw)
        
        #the sensor's extrinsic parameters can be returned as a list of floats
        extParams = sensor.extrinsicParameters()
        
        #turn on (True) or off (False) rain/fog suppression on the sensor
        sensor.setRainFogSuppression(False)
        
        sensor.resetShowMessages()
    
        #make sure the lidar is spinning (ie., in normal mode), safe to call if the lidar is already spinning
        sensor.lidarSpinUp()
    
        ##########################################################################################
        #if you want to set a static IP address on the sensor
        #program is force killed as the sensor needs to be rebooted for IP changes to be set
    
#        sensor.setStaticIP("192.168.1.40")
    
        #if you want to set IP address on the sensor to dynamic (ie., assigned by a DHCP server)
        #program is force killed as the sensor needs to be rebooted for IP changes to be set
    
#        sensor.setDynamicIP() 
       
        ##########################################################################################
        
        #start data stream (inherently this means the point cloud data is first stored in memory before being written to file, resulting in POOR PERFORMANCE)
        #   ***** DEPRECATED as of version 1.0.1 use .dataStart_RT() instead *****
#        sensor.dataStart()  #works ok for short duration datasets
        
        #start data stream (real-time writing of point cloud data to a comma delimited ASCII file, resulting in IMPROVED PERFORMANCE)
#        sensor.dataStart_RT()
        
        #start data stream (real-time writing of point cloud data to a BINARY file, resulting in MUCH IMPROVED PERFORMANCE)
        sensor.dataStart_RT_B()
        
        #the sensor's lidar status codes can be returned as a list of integers
#        status = sensor.lidarStatusCodes()
        
        filePathAndName = "test.bin"  #IMPORTANT: watch for OS specific path behaviour (future update will include a python package to handle this automatically)
        secsToWait = 1.0                #seconds, time delayed data capture start
        duration = 10.0                 #seconds, zero (0) specifies an indefinite duration
        
        #(*** IMPORTANT: this command starts a new thread, so the current program (thread) needs to exist for the 'duration' ***)
        #capture the data stream and save it to a CSV text file
        #   ***** DEPRECATED as of version 1.0.2 use .saveDataToFile instead *****
#        sensor.saveDataToCSV(filePathAndName, secsToWait, duration)
        
        #(*** IMPORTANT: this command starts a new thread, so the current program (thread) needs to exist for the 'duration' ***)
        #capture the data stream and save it to a file
        sensor.saveDataToFile(filePathAndName, secsToWait, duration)
        
        #simulate other operations being performed
        while True:
            # 1 + 1 = 2
            
#            time.sleep(3)   #example of time < (duration + secsToWait), therefore early data capture stop

#            #close the output data file, even if duration has not occurred (ideally used when duration = 0)
#            #   ***** DEPRECATED as of version 1.0.2 use .closeFile() instead *****
#            sensor.closeCSV()
            
#            sensor.closeFile()
#            break
        
            #exit loop when capturing is complete (*** IMPORTANT: ignores (does not check) sensors with duration set to 0)
            if sensor.doneCapturing():
                break
        
        #########################################################################################################
        ##### NOTE: Any one of the following commands with also close the output data file (if still being written) #####
        #########################################################################################################
        
        #stop data stream
        sensor.dataStop()
    
        #if you want to put the lidar in stand-by mode, not sure exactly what this does, lidar still spins?
#        sensor.lidarStandBy()
    
        #if you want to stop the lidar from spinning (ie., lidar to power-save mode) 
        sensor.lidarSpinDown()
            
        #properly disconnect from the sensor
        sensor.disconnect()
        
        #convert BINARY data to CSV file (no harm done if filePathAndName is mistakenly an ASCII CSV file)
        opl.convertBin2CSV(filePathAndName,deleteBin=False)
        
    else:
        print("\n***** Could not connect to a Livox sensor *****\n")



#demo operations for automatically connecting to multiple Livox Mid-40 Sensors
def multipleSensorsDemo():

    #simple list to collect individual sensor objects
    sensors = []

    connected = 0
    #loop through and find and connect to all sensors
    while connected != 1:
              
        #instantiate an openpylivox sensor object
        sensor = opl.openpylivox(True)
        
        #auto find and connect to a sensor
        connected = sensor.auto_connect("192.168.1.20")
        
        if connected:
            #initial commands for each sensor
#            sensor.setCartesianCS()
            sensor.setSphericalCS()
            sensor.setRainFogSuppression(False)
            
            print()     #just for demo readability purposes
            
            #append to sensor objects list
            sensors.append(sensor)
    
    #make sure a sensor was found and connected
    if sensors:
        #spin up all the sensors 
        for i in range(0, len(sensors)):
            sensors[i].lidarSpinUp()
        
        print()     #just for demo readability purposes
            
            
        #start all their data streams
        for i in range(0, len(sensors)):
#            sensors[i].dataStart()
            sensors[i].dataStart_RT()
        
        print()     #just for demo readability purposes
            
        #save data from all the sensors to individual CSV files, using sensor's serial # as filename
        for i in range(0, len(sensors)):
            sensors[i].showMessages = False
            filename = sensors[i].serialNumber() + "_sph.csv"
            sensors[i].resetShowMessages()
            
            #   ***** DEPRECATED as of version 1.0.2 use .saveDataToFile instead *****
#            sensors[i].saveDataToCSV(filename, 0.5, 10.0)
            sensors[i].saveDataToFile(filename, 0.5, 10.0)
        
        print()     #just for demo readability purposes
        
        #simulate other operations being performed
        while True:
            # 1 + 1 = 2
        
            #utility function to exit loop when capturing is complete from ALL sensors (*** IMPORTANT: ignores (does not check) sensors with duration set to 0)         
            if opl.allDoneCapturing(sensors):
                break
        
        #stop data on all the sensors
        for i in range(0, len(sensors)):
            sensors[i].dataStop()
        
        print()     #just for demo readability purposes
            
        #spin down all the sensors
        for i in range(0, len(sensors)):
            sensors[i].lidarSpinDown()
        
        print()     #just for demo readability purposes
            
        #disconnect all the sensors
        for i in range(0, len(sensors)):
            sensors[i].disconnect()



if __name__ == '__main__':
    
    singleSensorDemo()
    
#    multipleSensorsDemo()
    
    
    
    
