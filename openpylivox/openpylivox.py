#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

Started on Mon. May 13th 2019

@author: Ryan Brazeal
@email: ryan.brazeal@ufl.edu

Program Name: openpylivox.py
Version: 1.0.1

Description: Python3 driver for UDP Communications with Lixov Mid-40 Lidar sensor

Livox SDK link: https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol

Change Log:
    - v1.0.0 release Sept. 13th 2019
    - v1.0.1 release May 27th 2020
    

"""


#standard modules
import binascii
import select
import socket
import struct
import threading
import time
import sys

#additional modules
import crcmod
import numpy as np     #not heavily used in v1.0 but future version will require it for sure!


class _heartbeatThread(object):
    
    def __init__(self, interval, transmit_socket, send_to_IP, send_to_port, send_command, showMessages):
        self.interval = interval
        self.IP = send_to_IP
        self.port = send_to_port
        self.t_socket = transmit_socket
        self.t_command = send_command
        self.started = True
        self.work_state = -1
        self.idle_state = 0
        self.showMessages = showMessages
        
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
        self.thread.start()                     


    def run(self):
        while True:
            if self.started:
                self.t_socket.sendto(self.t_command,(self.IP, self.port))
                
                #check for proper response from heartbeat request
                if select.select([self.t_socket],[],[],0.1)[0]:
                    binData, addr = self.t_socket.recvfrom(22)
                    tempObj = openpylivox()
                    _,ack,cmd_set,cmd_id,ret_code_bin = tempObj._parseResp(binData)           
    
                    if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "3":
                        ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                        if ret_code != 0:
                            if self.showMessages: print("   " + self.IP + " --> incorrect heartbeat response")
                        else:
                            self.work_state = int.from_bytes(ret_code_bin[1], byteorder='little')
                            
                            #TODO: read and store the lidar status codes from heartbeat response (right now only being read from data stream)
                            
                            if self.work_state == 4:
                                print("   " + self.IP + " --> *** ERROR: HEARTBEAT ERROR MESSAGE RECEIVED ***")
                                sys.exit(0)
                    elif ack == "MSG (message)" and cmd_set == "General" and cmd_id == "7":
                        #not given an option to hide this message!!
                        print("   " + self.IP + " --> *** ERROR: ABNORMAL STATUS MESSAGE RECEIVED ***")
                        sys.exit(1)
                    else:
                        if self.showMessages: print("   " + self.IP + " --> incorrect heartbeat response")
                
                for i in range(9,-1,-1):
                    self.idle_state = i
                    time.sleep(self.interval / 10.0)
            else:
                break
            
            
    def stop(self):
        self.started = False      
        self.thread.join()
        self.idle_state = 9
        

class _dataCaptureThread(object):
    
    def __init__(self, sensorIP, receive_socket, filePathAndName, fileType, secsToWait, duration, firmwareType, showMessages):
        
        self.startTime = -1
        self.sensorIP = sensorIP
        self.r_socket = receive_socket
        self.filePathAndName = filePathAndName
        #fileType 0 = Stored ASCII, 1 = Real-time ASCII
        self.fileType = fileType
        self.secsToWait = secsToWait
        self.duration = duration
        self.firmwareType = firmwareType
        self.started = True
        self.isCapturing = False
        self.dataType = -1
        self.numPts = 0
        self.nullPts = 0
        self.showMessages = showMessages
        self.system_status = -1
        self.temp_status = -1
        self.volt_status = -1
        self.motor_status = -1
        self.dirty_status = -1
        self.firmware_status = -1
        self.pps_status = -1
        self.device_status = -1
        
        if duration == 0:
            self.duration = 126230400   #4 years of time (so technically not indefinite)

        self.thread = None

        if self.fileType == 1:
            self.thread = threading.Thread(target=self.run_realtime_csv, args=())
        else:
            self.thread = threading.Thread(target=self.run, args=())

        self.thread.daemon = True
        self.thread.start()                     


    def run(self):
        
        #read point cloud data packet to get packet version and datatype
        #keep looping to 'consume' data that we don't want included in the captured point cloud data

        breakByCapture = False
        while True:

            if self.started:
                selectTest = select.select([self.r_socket],[],[],0)
                if selectTest[0]:
                    data_pc, addr = self.r_socket.recvfrom(1500)
                    version = int.from_bytes(data_pc[0:1], byteorder='little')
                    self.dataType = int.from_bytes(data_pc[9:10], byteorder='little')
                    timestamp_type = int.from_bytes(data_pc[8:9], byteorder='little')
                    timestamp1 = self.getTimestamp(data_pc[10:18], timestamp_type)
                    self.updateStatus(data_pc[4:8])
                    if self.isCapturing:
                        self.startTime = timestamp1
                        breakByCapture = True
                        break
            else:
                break

        if breakByCapture:

            #check data packet is as expected (first byte anyways)
            if version == 5:
        
                #lists to capture point cloud data stream info 
                #TODO: should use an object, I know, I know!
                timestamps = []
                timestamp_types = []
                slot_ids = []
                lidar_ids = []
                coord1s = []
                coord2s = []
                coord3s = []
                intensities = []
                returnNums = []
    
                #delayed start to capturing data check (secsToWait parameter)
                timestamp2 = self.startTime
                while True:
                    if self.started:
                        timeSinceStart = timestamp2 - self.startTime
                        if timeSinceStart <= self.secsToWait:
                            #read data from receive buffer and keep 'consuming' it
                            if select.select([self.r_socket],[],[],0)[0]:
                                data_pc, addr = self.r_socket.recvfrom(1500)
                                timestamp_type = int.from_bytes(data_pc[8:9], byteorder='little')
                                timestamp2 = self.getTimestamp(data_pc[10:18], timestamp_type)
                                self.updateStatus(data_pc[4:8])
                        else:
                            self.startTime = timestamp2
                            break
                    else:
                         break   
                
                if self.showMessages: print("   " + self.sensorIP + " --> CAPTURING DATA...")
                
                #duration adjustment (trying to get exactly 100,000 points / sec)
                if self.duration != 126230400:
                    if self.firmwareType == 1:
                        self.duration += (0.001 * (self.duration / 2.0))
                    elif self.firmwareType == 2:
                        self.duration += (0.0005 * (self.duration / 2.0))
                    elif self.firmwareType == 3:
                        self.duration += (0.00055 * (self.duration / 2.0))
                
                timestamp_sec = self.startTime
                #main loop that captures the desired point cloud data 
                while True:
                    if self.started:
                        timeSinceStart = timestamp_sec - self.startTime
                    
                        if timeSinceStart <= self.duration:
                            
                            #read data from receive buffer
                            if select.select([self.r_socket],[],[],0)[0]:
                                data_pc, addr = self.r_socket.recvfrom(1500)
                                
                                version = int.from_bytes(data_pc[0:1], byteorder='little')
                                slot_id = int.from_bytes(data_pc[1:2], byteorder='little')
                                lidar_id = int.from_bytes(data_pc[2:3], byteorder='little')
                                
                                #byte 3 is reserved
                                
                                #update lidar status information
                                self.updateStatus(data_pc[4:8])
                                
                                timestamp_type = int.from_bytes(data_pc[8:9], byteorder='little')   
                                timestamp_sec = self.getTimestamp(data_pc[10:18], timestamp_type)
                                
                                bytePos = 18
                                
                                #single return firmware (most common)
                                if self.firmwareType == 1:
                                    #to account for first point's timestamp being increment in the loop
                                    timestamp_sec -= 0.00001
                                    
                                    #Cartesian Coordinate System
                                    if self.dataType == 0:
                                        for i in range(0,100):
                                            
                                            # X coordinate
                                            coord1 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            # Y coordinate
                                            coord2 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            # Z coordinate
                                            coord3 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4 
                                            
                                            #intensity
                                            intensity = data_pc[bytePos:bytePos+1]
                                            bytePos += 1
                                            
                                            #timestamp
                                            timestamp_sec += 0.00001
                                            
                                            timestamps.append(timestamp_sec)
                                            timestamp_types.append(timestamp_type)
                                            slot_ids.append(slot_id)
                                            lidar_ids.append(lidar_id)
                                            coord1s.append(coord1)
                                            coord2s.append(coord2)
                                            coord3s.append(coord3)
                                            intensities.append(intensity)
                                         
                                    #Spherical Coordinate System
                                    elif self.dataType == 1:
                                        for i in range(0,100):
                                            
                                            #distance
                                            coord1 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            #zenith
                                            coord2 = data_pc[bytePos:bytePos+2]
                                            bytePos += 2
                                            
                                            #azimuth
                                            coord3 = data_pc[bytePos:bytePos+2]
                                            bytePos += 2
                                            
                                            #intensity
                                            intensity = data_pc[bytePos:bytePos+1]
                                            bytePos += 1
                                            
                                            #timestamp
                                            timestamp_sec += 0.00001
                                            
                                            timestamps.append(timestamp_sec)
                                            timestamp_types.append(timestamp_type)
                                            slot_ids.append(slot_id)
                                            lidar_ids.append(lidar_id)
                                            coord1s.append(coord1)
                                            coord2s.append(coord2)
                                            coord3s.append(coord3)
                                            intensities.append(intensity)
                                         
                                #double return firmware
                                elif self.firmwareType == 2:
                                    #to account for first point's timestamp being increment in the loop
                                    timestamp_sec -= 0.00001
                                
                                    #Cartesian Coordinate System
                                    if self.dataType == 0:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                            # X coordinate
                                            coord1 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            # Y coordinate
                                            coord2 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            # Z coordinate
                                            coord3 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            #intensity
                                            intensity = data_pc[bytePos:bytePos+1]
                                            bytePos += 1
                                            
                                            zeroORtwo = i % 2
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORtwo)) * 0.00001
                                            
                                            #return number
                                            returnNum += zeroORtwo * 1
                                            
                                            timestamps.append(timestamp_sec)
                                            timestamp_types.append(timestamp_type)
                                            slot_ids.append(slot_id)
                                            lidar_ids.append(lidar_id)
                                            coord1s.append(coord1)
                                            coord2s.append(coord2)
                                            coord3s.append(coord3)
                                            intensities.append(intensity)
                                            returnNums.append(returnNum)
                                         
                                    #Spherical Coordinate System
                                    elif self.dataType == 1:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                            #distance
                                            coord1 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            #zenith
                                            coord2 = data_pc[bytePos:bytePos+2]
                                            bytePos += 2
                                            
                                            #azimuth
                                            coord3 = data_pc[bytePos:bytePos+2]
                                            bytePos += 2
                                            
                                            #intensity
                                            intensity = data_pc[bytePos:bytePos+1]
                                            bytePos += 1
                                            
                                            zeroORtwo = i % 2
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORtwo)) * 0.00001
                                            
                                            #return number
                                            returnNum += zeroORtwo * 1
                                            
                                            timestamps.append(timestamp_sec)
                                            timestamp_types.append(timestamp_type)
                                            slot_ids.append(slot_id)
                                            lidar_ids.append(lidar_id)
                                            coord1s.append(coord1)
                                            coord2s.append(coord2)
                                            coord3s.append(coord3)
                                            intensities.append(intensity)
                                            returnNums.append(returnNum)
                                
                                #triple return firmware
                                elif self.firmwareType == 3:
                                    #to account for first point's timestamp being increment in the loop
                                    timestamp_sec -= 0.000016666
                                
                                    #Cartesian Coordinate System
                                    if self.dataType == 0:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                            # X coordinate
                                            coord1 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            # Y coordinate
                                            coord2 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            # Z coordinate
                                            coord3 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            #intensity
                                            intensity = data_pc[bytePos:bytePos+1]
                                            bytePos += 1
                                            
                                            zeroORoneORtwo = i % 3
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORoneORtwo)) * 0.000016666
                                            
                                            #return number
                                            returnNum += zeroORoneORtwo * 1
                                            
                                            timestamps.append(timestamp_sec)
                                            timestamp_types.append(timestamp_type)
                                            slot_ids.append(slot_id)
                                            lidar_ids.append(lidar_id)
                                            coord1s.append(coord1)
                                            coord2s.append(coord2)
                                            coord3s.append(coord3)
                                            intensities.append(intensity)
                                            returnNums.append(returnNum)
                                         
                                    #Spherical Coordinate System
                                    elif self.dataType == 1:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                            #distance
                                            coord1 = data_pc[bytePos:bytePos+4]
                                            bytePos += 4
                                            
                                            #zenith
                                            coord2 = data_pc[bytePos:bytePos+2]
                                            bytePos += 2
                                            
                                            #azimuth
                                            coord3 = data_pc[bytePos:bytePos+2]
                                            bytePos += 2
                                            
                                            #intensity
                                            intensity = data_pc[bytePos:bytePos+1]
                                            bytePos += 1
                                            
                                            zeroORoneORtwo = i % 3
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORoneORtwo)) * 0.000016666
                                            
                                            #return number
                                            returnNum += zeroORoneORtwo * 1
                                            
                                            timestamps.append(timestamp_sec)
                                            timestamp_types.append(timestamp_type)
                                            slot_ids.append(slot_id)
                                            lidar_ids.append(lidar_id)
                                            coord1s.append(coord1)
                                            coord2s.append(coord2)
                                            coord3s.append(coord3)
                                            intensities.append(intensity)
                                            returnNums.append(returnNum)
                        
                        #duration check (exit point)
                        else:
                            self.started = False
                            self.isCapturing = False
                            break
                    #thread still running check (exit point)
                    else:
                        break
                
                #make sure some data was captured
                lenData = len(coord1s)
                if lenData > 0:
                
                    if self.showMessages: print("   " + self.sensorIP + " --> writing data to file: " + self.filePathAndName)
                    csvFile = open(self.filePathAndName,"w")
                    
                    numPts = 0
                    nullPts = 0
                    
                    #TODO: apply coordinate transformations to the raw X, Y, Z point cloud data based on the extrinsic parameters
                    #rotation definitions and the sequence they are applied is always a bit of a head scratcher, lots of different definitions
                    #Geospatial/Traditional Photogrammetry/Computer Vision/North America/Europe all use different approaches
                    
                    #single return fimware
                    if self.firmwareType == 1:
                        
                        #Cartesian
                        if self.dataType == 0:
                            csvFile.write("//X,Y,Z,Inten-sity,Time\n")
                            for i in range(0,lenData):
                                coord1 = round(float(struct.unpack('<i',coord1s[i])[0])/1000.0,3)
                                coord2 = round(float(struct.unpack('<i',coord2s[i])[0])/1000.0,3)
                                coord3 = round(float(struct.unpack('<i',coord3s[i])[0])/1000.0,3)
                                if coord1 or coord2 or coord3:
                                    numPts += 1
                                    csvFile.write("{0:.3f}".format(coord1) \
                                                  + "," + "{0:.3f}".format(coord2) \
                                                  + "," + "{0:.3f}".format(coord3) \
                                                  + "," + str(int.from_bytes(intensities[i], byteorder='little')) \
                                                  + "," + "{0:.6f}".format(timestamps[i]) + "\n")
                                else:
                                    nullPts += 1
                                
                        #Spherical
                        elif self.dataType == 1:
                            csvFile.write("//Distance,Zenith,Azimuth,Inten-sity,Time\n")
                            for i in range(0,lenData):
                                coord1 = round(float(struct.unpack('<I',coord1s[i])[0])/1000.0,3)
                                coord2 = round(float(struct.unpack('<H',coord2s[i])[0])/100.0,2)
                                coord3 = round(float(struct.unpack('<H',coord3s[i])[0])/100.0,2)
                                if coord1:
                                    numPts += 1
                                    csvFile.write("{0:.3f}".format(coord1) \
                                                  + "," + "{0:.2f}".format(coord2) \
                                                  + "," + "{0:.2f}".format(coord3) \
                                                  + "," + str(int.from_bytes(intensities[i], byteorder='little')) \
                                                  + "," + "{0:.6f}".format(timestamps[i]) + "\n")
                                else:
                                    nullPts += 1
                            
                    #multiple returns firmware
                    elif self.firmwareType == 2 or self.firmwareType == 3:
                        
                        #Cartesian
                        if self.dataType == 0:
                            csvFile.write("//X,Y,Z,Inten-sity,Time,ReturnNum\n")
                            for i in range(0,lenData):
                                coord1 = round(float(struct.unpack('<i',coord1s[i])[0])/1000.0,3)
                                coord2 = round(float(struct.unpack('<i',coord2s[i])[0])/1000.0,3)
                                coord3 = round(float(struct.unpack('<i',coord3s[i])[0])/1000.0,3)
                                if coord1 or coord2 or coord3:
                                    numPts += 1
                                    csvFile.write("{0:.3f}".format(coord1) \
                                                  + "," + "{0:.3f}".format(coord2) \
                                                  + "," + "{0:.3f}".format(coord3) \
                                                  + "," + str(int.from_bytes(intensities[i], byteorder='little')) \
                                                  + "," + "{0:.6f}".format(timestamps[i]) \
                                                  + "," + str(returnNums[i]) + "\n")
                                else:
                                    nullPts += 1
                                
                        #Spherical
                        elif self.dataType == 1:
                            csvFile.write("//Distance,Zenith,Azimuth,Inten-sity,Time,ReturnNum\n")
                            for i in range(0,lenData):
                                coord1 = round(float(struct.unpack('<I',coord1s[i])[0])/1000.0,3)
                                coord2 = round(float(struct.unpack('<H',coord2s[i])[0])/100.0,2)
                                coord3 = round(float(struct.unpack('<H',coord3s[i])[0])/100.0,2)
                                if coord1:
                                    numPts += 1
                                    csvFile.write("{0:.3f}".format(coord1) \
                                                  + "," + "{0:.2f}".format(coord2) \
                                                  + "," + "{0:.2f}".format(coord3) \
                                                  + "," + str(int.from_bytes(intensities[i], byteorder='little')) \
                                                  + "," + "{0:.6f}".format(timestamps[i]) \
                                                  + "," + str(returnNums[i]) + "\n")
                                else:
                                    nullPts += 1
    
                    self.numPts = numPts
                    self.nullPts = nullPts
                    
                    if self.showMessages: 
                        print("   " + self.sensorIP + " --> closed CSV file: " + self.filePathAndName)
                        print("                    (points: " + str(numPts) + " good, " + str(nullPts) + " null, " + str(numPts + nullPts) + " total)")
                    csvFile.close()
                
                else:
                    if self.showMessages: print("   " + self.sensorIP + " --> WARNING: no point cloud data was captured")
                
            else:
                if self.showMessages: print("   " + self.sensorIP + " --> Incorrect lidar packet version")               

   
    def run_realtime_csv(self):
        
        #read point cloud data packet to get packet version and datatype
        #keep looping to 'consume' data that we don't want included in the captured point cloud data

        breakByCapture = False
        while True:

            if self.started:
                selectTest = select.select([self.r_socket],[],[],0)
                if selectTest[0]:
                    data_pc, addr = self.r_socket.recvfrom(1500)
                    version = int.from_bytes(data_pc[0:1], byteorder='little')
                    self.dataType = int.from_bytes(data_pc[9:10], byteorder='little')
                    timestamp_type = int.from_bytes(data_pc[8:9], byteorder='little')
                    timestamp1 = self.getTimestamp(data_pc[10:18], timestamp_type)
                    self.updateStatus(data_pc[4:8])
                    if self.isCapturing:
                        self.startTime = timestamp1
                        breakByCapture = True
                        break
            else:
                break

        if breakByCapture:

            #check data packet is as expected (first byte anyways)
            if version == 5:
    
                #delayed start to capturing data check (secsToWait parameter)
                timestamp2 = self.startTime
                while True:
                    if self.started:
                        timeSinceStart = timestamp2 - self.startTime
                        if timeSinceStart <= self.secsToWait:
                            #read data from receive buffer and keep 'consuming' it
                            if select.select([self.r_socket],[],[],0)[0]:
                                data_pc, addr = self.r_socket.recvfrom(1500)
                                timestamp_type = int.from_bytes(data_pc[8:9], byteorder='little')
                                timestamp2 = self.getTimestamp(data_pc[10:18], timestamp_type)
                                self.updateStatus(data_pc[4:8])
                        else:
                            self.startTime = timestamp2
                            break
                    else:
                         break   
                
                if self.showMessages: print("   " + self.sensorIP + " --> CAPTURING DATA...")
                
                #duration adjustment (trying to get exactly 100,000 points / sec)
                if self.duration != 126230400:
                    if self.firmwareType == 1:
                        self.duration += (0.001 * (self.duration / 2.0))
                    elif self.firmwareType == 2:
                        self.duration += (0.0005 * (self.duration / 2.0))
                    elif self.firmwareType == 3:
                        self.duration += (0.00055 * (self.duration / 2.0))
                
                timestamp_sec = self.startTime
                
                if self.showMessages: print("   " + self.sensorIP + " --> writing real-time data to file: " + self.filePathAndName)
                csvFile = open(self.filePathAndName,"w",1)
                    
                numPts = 0
                nullPts = 0
                
                #write header info
                if self.firmwareType == 1:  #single return firmware
                    if self.dataType == 0: #Cartesian
                        csvFile.write("//X,Y,Z,Inten-sity,Time\n")
                    elif self.dataType == 1: #Spherical
                        csvFile.write("//Distance,Zenith,Azimuth,Inten-sity,Time\n")
                elif self.firmwareType == 2 or self.firmwareType == 3:  #double or triple return firmware
                    if self.dataType == 0: #Cartesian
                        csvFile.write("//X,Y,Z,Inten-sity,Time,ReturnNum\n")
                    elif self.dataType == 1: #Spherical
                        csvFile.write("//Distance,Zenith,Azimuth,Inten-sity,Time,ReturnNum\n")
                
                #main loop that captures the desired point cloud data 
                while True:
                    if self.started:
                        timeSinceStart = timestamp_sec - self.startTime
                    
                        if timeSinceStart <= self.duration:
                            
                            #read data from receive buffer
                            if select.select([self.r_socket],[],[],0)[0]:
                                data_pc, addr = self.r_socket.recvfrom(1500)
                                
#                                version = int.from_bytes(data_pc[0:1], byteorder='little')
#                                slot_id = int.from_bytes(data_pc[1:2], byteorder='little')
#                                lidar_id = int.from_bytes(data_pc[2:3], byteorder='little')
                                
                                #byte 3 is reserved
                                
                                #update lidar status information
                                self.updateStatus(data_pc[4:8])
                                
                                timestamp_type = int.from_bytes(data_pc[8:9], byteorder='little')   
                                timestamp_sec = self.getTimestamp(data_pc[10:18], timestamp_type)
                                
                                bytePos = 18
                                
                                #single return firmware (most common)
                                if self.firmwareType == 1:
                                    #to account for first point's timestamp being increment in the loop
                                    timestamp_sec -= 0.00001
                                    
                                    #Cartesian Coordinate System
                                    if self.dataType == 0:
                                        for i in range(0,100):
                                            
                                            # X coordinate
                                            coord1 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Y coordinate
                                            coord2 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Z coordinate
                                            coord3 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4 
                                            
                                            #intensity
                                            intensity = str(int.from_bytes(data_pc[bytePos:bytePos+1], byteorder='little'))
                                            bytePos += 1
                                            
                                            #timestamp
                                            timestamp_sec += 0.00001
                                            
                                            if coord2:
                                                numPts += 1
                                                csvFile.write(coord1 + "," + coord2 + "," + coord3 + "," + intensity + "," + "{0:.6f}".format(timestamp_sec) + "\n")
                                            else:
                                                nullPts += 1
                                    
                                    #Spherical Coordinate System
                                    elif self.dataType == 1:
                                        for i in range(0,100):
                                            
                                            # Distance coordinate
                                            coord1 = "{0:.3f}".format(float(struct.unpack('<I',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Zenith coordinate
                                            coord2 = "{0:.2f}".format(float(struct.unpack('<H',data_pc[bytePos:bytePos+2])[0])/100.0)
                                            bytePos += 2
                                            
                                            # Azimuth coordinate
                                            coord3 = "{0:.2f}".format(float(struct.unpack('<H',data_pc[bytePos:bytePos+2])[0])/100.0)
                                            bytePos += 2 
                                            
                                            #intensity
                                            intensity = str(int.from_bytes(data_pc[bytePos:bytePos+1], byteorder='little'))
                                            bytePos += 1
                                            
                                            #timestamp
                                            timestamp_sec += 0.00001
                                            
                                            if coord1:
                                                numPts += 1
                                                csvFile.write(coord1 + "," + coord2 + "," + coord3 + "," + intensity + "," + "{0:.6f}".format(timestamp_sec) + "\n")
                                            else:
                                                nullPts += 1
                                         
                                #double return firmware
                                elif self.firmwareType == 2:
                                    #to account for first point's timestamp being increment in the loop
                                    timestamp_sec -= 0.00001
                                
                                    #Cartesian Coordinate System
                                    if self.dataType == 0:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                            # X coordinate
                                            coord1 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Y coordinate
                                            coord2 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Z coordinate
                                            coord3 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4 
                                            
                                            #intensity
                                            intensity = str(int.from_bytes(data_pc[bytePos:bytePos+1], byteorder='little'))
                                            bytePos += 1
                                            
                                            zeroORtwo = i % 2
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORtwo)) * 0.00001
                                            
                                            #return number
                                            returnNum += zeroORtwo * 1
                                            
                                            if coord2:
                                                numPts += 1
                                                csvFile.write(coord1 + "," + coord2 + "," + coord3 + "," + intensity + "," + "{0:.6f}".format(timestamp_sec) + "," + str(returnNum) + "\n")
                                            else:
                                                nullPts += 1
                        
                                         
                                    #Spherical Coordinate System
                                    elif self.dataType == 1:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                             # Distance coordinate
                                            coord1 = "{0:.3f}".format(float(struct.unpack('<I',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Zenith coordinate
                                            coord2 = "{0:.2f}".format(float(struct.unpack('<H',data_pc[bytePos:bytePos+2])[0])/100.0)
                                            bytePos += 2
                                            
                                            # Azimuth coordinate
                                            coord3 = "{0:.2f}".format(float(struct.unpack('<H',data_pc[bytePos:bytePos+2])[0])/100.0)
                                            bytePos += 2 
                                            
                                            #intensity
                                            intensity = str(int.from_bytes(data_pc[bytePos:bytePos+1], byteorder='little'))
                                            bytePos += 1
                                            
                                            zeroORtwo = i % 2
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORtwo)) * 0.00001
                                            
                                            #return number
                                            returnNum += zeroORtwo * 1
                                            
                                            if coord1:
                                                numPts += 1
                                                csvFile.write(coord1 + "," + coord2 + "," + coord3 + "," + intensity + "," + "{0:.6f}".format(timestamp_sec) + "," + str(returnNum) + "\n")
                                            else:
                                                nullPts += 1
                                            
                                
                                #triple return firmware
                                elif self.firmwareType == 3:
                                    #to account for first point's timestamp being increment in the loop
                                    timestamp_sec -= 0.000016666
                                
                                    #Cartesian Coordinate System
                                    if self.dataType == 0:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                            # X coordinate
                                            coord1 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Y coordinate
                                            coord2 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Z coordinate
                                            coord3 = "{0:.3f}".format(float(struct.unpack('<i',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4 
                                            
                                            #intensity
                                            intensity = str(int.from_bytes(data_pc[bytePos:bytePos+1], byteorder='little'))
                                            bytePos += 1
                                            
                                            zeroORoneORtwo = i % 3
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORoneORtwo)) * 0.000016666
                                            
                                            #return number
                                            returnNum += zeroORoneORtwo * 1
                                            
                                            if coord2:
                                                numPts += 1
                                                csvFile.write(coord1 + "," + coord2 + "," + coord3 + "," + intensity + "," + "{0:.6f}".format(timestamp_sec) + "," + str(returnNum) + "\n")
                                            else:
                                                nullPts += 1
                                        
                                         
                                    #Spherical Coordinate System
                                    elif self.dataType == 1:
                                        for i in range(0,100):
                                            returnNum = 1
                                            
                                             # Distance coordinate
                                            coord1 = "{0:.3f}".format(float(struct.unpack('<I',data_pc[bytePos:bytePos+4])[0])/1000.0)
                                            bytePos += 4
                                            
                                            # Zenith coordinate
                                            coord2 = "{0:.2f}".format(float(struct.unpack('<H',data_pc[bytePos:bytePos+2])[0])/100.0)
                                            bytePos += 2
                                            
                                            # Azimuth coordinate
                                            coord3 = "{0:.2f}".format(float(struct.unpack('<H',data_pc[bytePos:bytePos+2])[0])/100.0)
                                            bytePos += 2 
                                            
                                            #intensity
                                            intensity = str(int.from_bytes(data_pc[bytePos:bytePos+1], byteorder='little'))
                                            bytePos += 1
                                            
                                            zeroORoneORtwo = i % 3
                                            
                                            #timestamp
                                            timestamp_sec += float(not(zeroORoneORtwo)) * 0.000016666
                                            
                                            #return number
                                            returnNum += zeroORoneORtwo * 1
                                            
                                            if coord1:
                                                numPts += 1
                                                csvFile.write(coord1 + "," + coord2 + "," + coord3 + "," + intensity + "," + "{0:.6f}".format(timestamp_sec) + "," + str(returnNum) + "\n")
                                            else:
                                                nullPts += 1
                    
                        
                        #duration check (exit point)
                        else:
                            self.started = False
                            self.isCapturing = False
                            break
                    #thread still running check (exit point)
                    else:
                        break
                

    
                self.numPts = numPts
                self.nullPts = nullPts
                    
                if self.showMessages: 
                    print("   " + self.sensorIP + " --> closed CSV file: " + self.filePathAndName)
                    print("                    (points: " + str(numPts) + " good, " + str(nullPts) + " null, " + str(numPts + nullPts) + " total)")
                
                csvFile.close()
                
            else:
                if self.showMessages: print("   " + self.sensorIP + " --> Incorrect lidar packet version") 
    
    
    def getTimestamp(self, data_pc, timestamp_type):
        
        #nanosecond timestamp
        if timestamp_type == 0 or timestamp_type == 1 or timestamp_type == 4:    
            timestamp_sec = round(float(struct.unpack('<Q',data_pc[0:8])[0])/1000000000.0,6) #convert to seconds
        
        #UTC timestamp, microseconds past the hour
        elif timestamp_type == 3:
            timestamp_year = int.from_bytes(data_pc[0:1], byteorder='little')
            timestamp_month = int.from_bytes(data_pc[1:2], byteorder='little')
            timestamp_day = int.from_bytes(data_pc[2:3], byteorder='little')
            timestamp_hour = int.from_bytes(data_pc[3:4], byteorder='little')
            timestamp_sec = round(float(struct.unpack('<L',data_pc[4:8])[0])/1000000.0,6) #convert to seconds
        
            timestamp_sec += timestamp_hour * 3600. #seconds into the day
        
            #TODO: check and adjust for hour, day, month and year crossovers
            
        return timestamp_sec
    
    
    #parse lidar status codes and update object properties, can provide real-time warning/error message display
    def updateStatus(self, data_pc):
        
        status_bits = str(bin(int.from_bytes(data_pc[0:1], byteorder='little')))[2:].zfill(8)
        status_bits += str(bin(int.from_bytes(data_pc[1:2], byteorder='little')))[2:].zfill(8)
        status_bits += str(bin(int.from_bytes(data_pc[2:3], byteorder='little')))[2:].zfill(8)
        status_bits += str(bin(int.from_bytes(data_pc[3:4], byteorder='little')))[2:].zfill(8)
        
        self.temp_status = int(status_bits[0:2],2)
        self.volt_status = int(status_bits[2:4],2)
        self.motor_status = int(status_bits[4:6],2)
        self.dirty_status = int(status_bits[6:8],2)
        self.firmware_status = int(status_bits[8:9],2)
        self.pps_status = int(status_bits[9:10],2)
        self.device_status = int(status_bits[10:11],2)
        self.system_status = int(status_bits[30:],2)
        
        #check if the system status in NOT normal
        if self.system_status:
            if self.showMessages:
                if self.system_status == 1:
                    if self.temp_status == 1:
                        print("   " + self.sensorIP + " --> * WARNING: temperature *")
                    if self.volt_status == 1:
                        print("   " + self.sensorIP + " --> * WARNING: voltage *")
                    if self.motor_status == 1:
                        print("   " + self.sensorIP + " --> * WARNING: motor *")
                    if self.dirty_status == 1:
                        print("   " + self.sensorIP + " --> * WARNING: dirty or blocked *")
                    if self.device_status == 1:
                        print("   " + self.sensorIP + " --> * WARNING: approaching end of service life *")
                elif self.system_status == 2: 
                    if self.temp_status == 1:
                        print("   " + self.sensorIP + " --> *** ERROR: TEMPERATURE ***")
                    if self.volt_status == 1:
                        print("   " + self.sensorIP + " --> *** ERROR: VOLTAGE ***")
                    if self.motor_status == 1:
                        print("   " + self.sensorIP + " --> *** ERROR: MOTOR ***")
                    if self.firmware_status == 1:
                        print("   " + self.sensorIP + " --> *** ERROR: ABNORMAL FIRMWARE ***")
            
    
    #returns latest status Codes from within the point cloud data packet
    def statusCodes(self):
        
        return [self.system_status,self.temp_status,self.volt_status,self.motor_status,self.dirty_status,self.firmware_status,self.pps_status,self.device_status]
    
        
    def stop(self):
        
        self.started = False
        self.thread.join()


class openpylivox(object):
    
    _CMD_QUERY = bytes.fromhex((b'AA010F0000000004D70002AE8A8A7B').decode('ascii'))
    _CMD_HEARTBEAT = bytes.fromhex((b'AA010F0000000004D7000338BA8D0C').decode('ascii'))
    _CMD_DISCONNECT = bytes.fromhex((b'AA010F0000000004D70006B74EE77C').decode('ascii'))
    _CMD_LIDAR_START = bytes.fromhex((b'AA011000000000B8090100011122FD62').decode('ascii'))
    _CMD_LIDAR_POWERSAVE = bytes.fromhex((b'AA011000000000B809010002AB73F4FB').decode('ascii'))
    _CMD_LIDAR_STANDBY = bytes.fromhex((b'AA011000000000B8090100033D43F38C').decode('ascii'))
    _CMD_DATA_STOP = bytes.fromhex((b'AA011000000000B809000400B4BD5470').decode('ascii'))
    _CMD_DATA_START = bytes.fromhex((b'AA011000000000B809000401228D5307').decode('ascii'))
    _CMD_CARTESIAN_CS = bytes.fromhex((b'AA011000000000B809000500F58C4F69').decode('ascii'))
    _CMD_SPHERICAL_CS = bytes.fromhex((b'AA011000000000B80900050163BC481E').decode('ascii'))
    _CMD_DYNAMIC_IP = bytes.fromhex((b'AA011400000000A8240008000000000068F8DD50').decode('ascii'))
    _CMD_WRITE_ZERO_EXTRINSIC = bytes.fromhex((b'AA012700000000B5ED01010000000000000000000000000000000000000000000000004CDEA4E7').decode('ascii'))
    _CMD_READ_EXTRINSIC = bytes.fromhex((b'AA010F0000000004D70102EFBB9162').decode('ascii'))
    _CMD_RAIN_FOG_ON = bytes.fromhex((b'AA011000000000B809010301D271D049').decode('ascii'))
    _CMD_RAIN_FOG_OFF = bytes.fromhex((b'AA011000000000B8090103004441D73E').decode('ascii'))

    _SPECIAL_FIRMWARE_TYPE_DICT = {"03.03.0001": 2,
                                   "03.03.0002": 3,
                                   "03.03.0006": 2,
                                   "03.03.0007": 3}
    
    def __init__(self, showMessages = False):
        
        self._isConnected = False
        self._isData = False
        self._isWriting = False
        self._dataSocket = ""
        self._cmdSocket = ""
        self._heartbeat = None
        self._firmware = "UNKNOWN"
        self._coordSystem = -1
        self._x = None
        self._y = None
        self._z = None
        self._roll = None
        self._pitch = None
        self._yaw = None
        self._captureStream = None
        self._serial = "UNKNOWN"
        self._ipRangeCode = 0
        self._computerIP = ""
        self._sensorIP = ""
        self._dataPort = -1
        self._cmdPort = -1
        self._init_showMessages = showMessages
        self.showMessages = showMessages
        
    
    def _reinit(self):
        
        self._dataSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmdSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dataSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._cmdSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        foundIPs, foundSerials, ipRangeCodes = self._searchForMid40s(True)
        
        foundMatchIP = False
        for i in range(0,len(foundIPs)):
            if foundIPs[i] == self._sensorIP:
                foundMatchIP = True
                self._serial = foundSerials[i]
                self._ipRangeCode = ipRangeCodes[i]
                break
        
        if foundMatchIP == False:
            print("\n* ERROR: specified sensor IP:Command Port cannot connect to a Livox sensor *")
            print("* common causes are a wrong IP or the command port is being used already   *\n")
            time.sleep(0.1)
            sys.exit(2)
            
        return len(foundIPs)
        
    
    def _checkIP(self, inputIP):
        
        IPclean = ""
        if inputIP:
            IPparts = inputIP.split(".")
            if len(IPparts) == 4:
                i = 0
                for i in range(0,4):
                    try:
                        IPint = int(IPparts[i])
                        if IPint >= 0 and IPint <= 254:
                            IPclean += str(IPint)
                            if i < 3:
                                IPclean += "."
                        else:
                            IPclean = ""
                            break
                    except:
                        IPclean = ""
                        break
        
        return IPclean
    
    
    def _checkPort(self, inputPort):

        try:
            portNum = int(inputPort)
            if portNum >= 0 and portNum <= 65535:
                pass
            else:
                portNum = -1
        except:
            portNum = -1
            
        return portNum
        
    
    def _searchForMid40s(self, opt = False):
    
        serverSock_INIT = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        serverSock_INIT.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serverSock_INIT.bind(("0.0.0.0", 55000))
    
        foundDevice = select.select([serverSock_INIT],[],[],1)[0]
        
        IPs = []
        Serials = []
        ipRangeCodes = []
        
        if foundDevice:
            readData = True
            while readData:
                
                binData, addr = serverSock_INIT.recvfrom(34)
                if len(addr) == 2:
                    if addr[1] == 65000:
                        if len(IPs) == 0:
                            goodData, cmdMessage, dataMessage, device_serial, typeMessage, ipRangeCode = self._info(binData)
                            
                            if typeMessage == "Mid-40":
                                IPs.append(self._checkIP(addr[0]))
                                Serials.append(device_serial)
                                ipRangeCodes.append(ipRangeCode)
                        else:
                            existsAlready = False
                            for i in range(0,len(IPs)):
                                if addr[0] == IPs[i]:
                                    existsAlready = True
                            if existsAlready:
                                readData = False
                                break
                            else:
                                goodData, cmdMessage, dataMessage, device_serial, typeMessage, ipRangeCode = self._info(binData)
                            
                                if typeMessage == "Mid-40":
                                    IPs.append(self._checkIP(addr[0]))
                                    Serials.append(device_serial)
                                    ipRangeCodes.append(ipRangeCode)
                else:
                    readData = False
                
        serverSock_INIT.close()
        time.sleep(0.2)
        
        if self.showMessages and opt:
            for i in range(0,len(IPs)):
                print("   Found Mid-40 w. serial #" + Serials[i] + " at IP: " + IPs[i])
            print()
        
        return IPs, Serials, ipRangeCodes

    
    def _auto_computerIP(self):
        
        try:        
            hostname = socket.gethostname()
            self._computerIP = socket.gethostbyname(hostname)

        except:
            self.computerIP = ""
    
    
    def _bindPorts(self):
        
        try:
            self._dataSocket.bind((self._computerIP, self._dataPort))
            self._cmdSocket.bind((self._computerIP, self._cmdPort))
            assignedDataPort = self._dataSocket.getsockname()[1]
            assignedCmdPort = self._cmdSocket.getsockname()[1]

            time.sleep(0.1)
            
            return assignedDataPort, assignedCmdPort
            
        except socket.error as err:
            print(" *** ERROR: cannot bind to specified IP:Port(s), " + err)
            sys.exit(3)
    
    
    def _waitForIdle(self):
        
        while self._heartbeat.idle_state != 9:
            time.sleep(0.1)
            
    
    def _disconnectSensor(self):
        
        self._waitForIdle()
        self._cmdSocket.sendto(self._CMD_DISCONNECT,(self._sensorIP, 65000))
        
        #check for proper response from disconnect request
        if select.select([self._cmdSocket],[],[],0.1)[0]:
            binData, addr = self._cmdSocket.recvfrom(16)
            _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

            if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "6":
                ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                if ret_code == 1:
                    if self.showMessages: print("   " + self._sensorIP + " --> FAILED to disconnect")                
            else:
                if self.showMessages: print("   " + self._sensorIP + " --> incorrect disconnect response")
    

    def _query(self):
        
        self._waitForIdle()
        self._cmdSocket.sendto(self._CMD_QUERY,(self._sensorIP, 65000))
        
        #check for proper response from query request
        if select.select([self._cmdSocket],[],[],0.1)[0]:
            binData, addr = self._cmdSocket.recvfrom(20)
            _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

            if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "2":
                ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                if ret_code == 1:
                    if self.showMessages: print("   " + self._sensorIP + " --> FAILED to receive query results")
                elif ret_code == 0:
                    AA = str(int.from_bytes(ret_code_bin[1], byteorder='little')).zfill(2)
                    BB = str(int.from_bytes(ret_code_bin[2], byteorder='little')).zfill(2)
                    CC = str(int.from_bytes(ret_code_bin[3], byteorder='little')).zfill(2)
                    DD = str(int.from_bytes(ret_code_bin[4], byteorder='little')).zfill(2)
                    self._firmware = AA + "." + BB + "." + CC + DD
            else:
                if self.showMessages: print("   " + self._sensorIP + " --> incorrect query response")
    
    
    def _info(self, binData):
    
        goodData, cmdMessage, dataMessage, dataID, dataBytes = self._parseResp(binData)
    
        device_serial, typeMessage = "", ""
        
        if goodData:
            #received broadcast message
            if cmdMessage == "MSG (message)" and dataMessage == "General" and dataID == "0":
                device_broadcast_code = ""
                for i in range(0,16):
                    device_broadcast_code += dataBytes[i].decode('ascii')
                
                ipRangeCode = int(device_broadcast_code[14:15])     #used to define L,M,R sensors in the Mid-100
                device_serial = device_broadcast_code[:-2]
                device_type = int.from_bytes(dataBytes[i+1], byteorder='little')
                
                typeMessage = ""
                if device_type == 0:
                    typeMessage = "Hub"
                elif device_type == 1:
                    typeMessage = "Mid-40"
                elif device_type == 2:
                    typeMessage = "Tele-15"
                elif device_type == 3:
                    typeMessage = "Horizon"
                            
        return goodData, cmdMessage, dataMessage, device_serial, typeMessage, ipRangeCode
    
    
    def _parseResp(self, binData):
    
        dataBytes = []
        dataString = ""
        dataLength = len(binData)
        for i in range(0, dataLength):
            dataBytes.append(binData[i:i+1])
            dataString += (binascii.hexlify(binData[i:i+1])).decode("utf-8")
        
        crc16Data = b''
        for i in range(0,7):
            crc16Data += binascii.hexlify(dataBytes[i])
    
        crc16DataA = bytes.fromhex((crc16Data).decode('ascii'))
        checkSum16I = self._crc16(crc16DataA)
        
        frame_header_checksum_crc16 = int.from_bytes((dataBytes[7] + dataBytes[8]), byteorder='little')
        
        cmdMessage, dataMessage, dataID, = "", "", ""
        data = []
        
        goodData = True
        
        if frame_header_checksum_crc16 == checkSum16I:
        
            crc32Data = b''
            for i in range(0,dataLength-4):
                crc32Data += binascii.hexlify(dataBytes[i])
        
            crc32DataA = bytes.fromhex((crc32Data).decode('ascii'))
            checkSum32I = self._crc32(crc32DataA)
            
            frame_header_checksum_crc32 = int.from_bytes((dataBytes[dataLength-4] + dataBytes[dataLength-3] + dataBytes[dataLength-2] + dataBytes[dataLength-1]), byteorder='little')
            
            if frame_header_checksum_crc32 == checkSum32I:
            
                frame_sof = int.from_bytes(dataBytes[0], byteorder='little') #should be 170 = '\xAA'
                frame_version = int.from_bytes(dataBytes[1], byteorder='little') #should be 1
                frame_length = int.from_bytes((dataBytes[2] + dataBytes[3]), byteorder='little') #max value = 1400
                
                if frame_sof == 170:
                    if frame_version == 1:
                        if frame_length <= 1400:
                            frame_cmd_type = int.from_bytes(dataBytes[4], byteorder='little')
            
                            cmdMessage = ""
                            if frame_cmd_type == 0:
                                cmdMessage = "CMD (request)"
                            elif frame_cmd_type == 1:
                                cmdMessage = "ACK (response)"
                            elif frame_cmd_type == 2:
                                cmdMessage = "MSG (message)"
                            else:
                                goodData = False
                            
                            frame_data_cmd_set = int.from_bytes(dataBytes[9], byteorder='little')
                            
                            dataMessage = ""
                            if frame_data_cmd_set == 0:
                                dataMessage = "General"
                            elif frame_data_cmd_set == 1:
                                dataMessage = "Lidar"
                            elif frame_data_cmd_set == 2:
                                dataMessage = "Hub"
                            else:
                                goodData = False
                            
                            dataID = str(int.from_bytes(dataBytes[10], byteorder='little'))
                            data = dataBytes[11:]
                            
                        else:
                            goodData = False
                    else:
                        goodData = False
                else:
                    goodData = False
            else:
                goodData = False
                if self.showMessages: print("CRC32 Checksum Error")
        else:
            goodData = False
            if self.showMessages: print("CRC16 Checksum Error")
                            
        return goodData, cmdMessage, dataMessage, dataID, data
    
    
    def _crc16(self, data):
        
        crc16 = crcmod.mkCrcFun(0x11021, rev=True, initCrc=0x4C49)
        checkSum = crc16(data)
        return checkSum
    
    
    def _crc16fromStr(self, binString):
        
        crcDataA = bytes.fromhex((binString).decode('ascii'))
        checkSum = self._crc16(crcDataA)
        strHexCheckSum = str(hex(checkSum))[2:]
        
        strLen = len(strHexCheckSum)
        for i in range(strLen,4):
            strHexCheckSum = "0" + strHexCheckSum
        
        byte1 = strHexCheckSum[2:4]
        byte2 = strHexCheckSum[0:2]
        
        checkSumB = (byte1 + byte2)
        
        return checkSumB
    
    
    def _crc32(self, data):
        
        crc32 = crcmod.mkCrcFun(0x104C11DB7, rev=True, initCrc=0x564F580A, xorOut=0xFFFFFFFF)
        checkSum = crc32(data)
        return checkSum
    
    
    def _crc32fromStr(self, binString):
        
        crcDataA = bytes.fromhex((binString).decode('ascii'))
        checkSum = self._crc32(crcDataA)
        strHexCheckSum = str(hex(checkSum))[2:]
        
        strLen = len(strHexCheckSum)
        for i in range(strLen,8):
            strHexCheckSum = "0" + strHexCheckSum
        
        byte1 = strHexCheckSum[6:8]
        byte2 = strHexCheckSum[4:6]
        byte3 = strHexCheckSum[2:4]
        byte4 = strHexCheckSum[0:2]
        
        checkSumB = (byte1 + byte2 + byte3 + byte4)
        
        return checkSumB
        
    
    def connect(self, computerIP, sensorIP, dataPort, cmdPort):
        
        numFound = 0
        
        if not self._isConnected:

            self._computerIP = self._checkIP(computerIP)
            self._sensorIP = self._checkIP(sensorIP)
            self._dataPort = self._checkPort(dataPort)
            self._cmdPort = self._checkPort(cmdPort)
            
            if self._computerIP and self._sensorIP and self._dataPort != -1 and self._cmdPort != -1:
                numFound = self._reinit()
                time.sleep(0.1)
                
                self._dataPort, self._cmdPort = self._bindPorts()
                
                IP_parts = self._computerIP.split(".")
                IPhex = str(hex(int(IP_parts[0]))).replace('0x','').zfill(2)
                IPhex += str(hex(int(IP_parts[1]))).replace('0x','').zfill(2)
                IPhex += str(hex(int(IP_parts[2]))).replace('0x','').zfill(2)
                IPhex += str(hex(int(IP_parts[3]))).replace('0x','').zfill(2)
                dataHexAll = str(hex(int(self._dataPort))).replace('0x','').zfill(4)
                dataHex = dataHexAll[2:] + dataHexAll[:-2]
                cmdHexAll = str(hex(int(self._cmdPort))).replace('0x','').zfill(4)
                cmdHex = cmdHexAll[2:] + cmdHexAll[:-2]
                cmdString = "aa01170000000064390001" + IPhex + dataHex + cmdHex
                binString = bytes(cmdString, encoding='utf-8')
                crc32checksum = self._crc32fromStr(binString)
                cmdString += crc32checksum
                binString = bytes(cmdString, encoding='utf-8')
                
                connect_request = bytes.fromhex((binString).decode('ascii'))
                self._cmdSocket.sendto(connect_request,(self._sensorIP, 65000))
                
                #check for proper response from connection request
                if select.select([self._cmdSocket],[],[],0.1)[0]:
                    binData, addr = self._cmdSocket.recvfrom(16)
                    _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
    
                    if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "1":
                        ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                        if ret_code == 0:
                            self._isConnected = True
                            self._heartbeat = _heartbeatThread(1, self._cmdSocket, self._sensorIP, 65000, self._CMD_HEARTBEAT, self.showMessages)
                            time.sleep(0.15)
                            self._query()
                            if self.showMessages: print("Connected to Mid-40 sensor at IP: " + self._sensorIP + " (F/W: " + self._firmware + ")")
                        else:
                            if self.showMessages: print("FAILED to connect to Mid-40 sensor at IP: " + self._sensorIP)
                    else:
                        if self.showMessages: print("FAILED to connect to Mid-40 sensor at IP: " + self._sensorIP)
            else:
                if self.showMessages: print("Invalid connection parameter(s)")       
        else:
            if self.showMessages: print("Already connected to Mid-40 sensor at IP: " + self._sensorIP)
            
        return numFound
                     
            
    def auto_connect(self, manualComputerIP = ""):
        
        numFound = 0
        
        if not manualComputerIP:
            self._auto_computerIP()
        else:
            self._computerIP = manualComputerIP
            
        if self._computerIP:
            if self.showMessages: print("Using computer IP address: " + self._computerIP)
            
            lidarSensorIPs, _ , _ = self._searchForMid40s(True)
            
            if len(lidarSensorIPs) > 0:
                
                if self.showMessages: print("Attempting to auto-connect to a sensor at IP: " + lidarSensorIPs[0])
                
                self.showMessages = False
                numFound = self.connect(self._computerIP, lidarSensorIPs[0], 0, 0)
                self.resetShowMessages()
                
                if self.showMessages: print("Connected to Mid-40 sensor at IP: " + self._sensorIP + " (F/W: " + self._firmware + ")")
                
        else:
            if self.showMessages: print("*** ERROR: Failed to auto determine computer IP address ***")
            
        return numFound
    
    
    def disconnect(self):
        
        if self._isConnected:
            self._isConnected = False
            self._isData = False
            if self._captureStream is not None:
                self._captureStream.stop()
                self._captureStream = None
            time.sleep(0.1)
            self._isWriting = False
                
            try:
                self._disconnectSensor()
                self._heartbeat.stop()
                time.sleep(0.2)
                self._heartbeat = None
                
                self._dataSocket.close()
                self._cmdSocket.close()
                if self.showMessages: print("Disconnected from Mid-40 sensor at IP: " + self._sensorIP)
                
            except:
                if self.showMessages: print("*** Error trying to disconnect from Mid-40 sensor at IP: " + self._sensorIP)
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
                
    
    def lidarSpinUp(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_LIDAR_START,(self._sensorIP, 65000))
            if self.showMessages: print("   " + self._sensorIP + " <-- sent lidar spin up request")
            
            #check for proper response from lidar start request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

                if ack == "ACK (response)" and cmd_set == "Lidar" and cmd_id == "0":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to spin up the lidar")
                    elif ret_code == 0:
                        if self.showMessages: print("   " + self._sensorIP + " --> lidar is ready")
                        time.sleep(0.1)
                    elif ret_code == 2:
                        if self.showMessages: print("   " + self._sensorIP + " --> lidar is spinning up, please wait...")
                        while True:
                            time.sleep(0.1)
                            if self._heartbeat.work_state == 1:
                                if self.showMessages: print("   " + self._sensorIP + " --> lidar is ready") 
                                time.sleep(0.1)
                                break
                            
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect lidar spin up response")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
            
    def lidarSpinDown(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_LIDAR_POWERSAVE,(self._sensorIP, 65000))
            if self.showMessages: print("   " + self._sensorIP + " <-- sent lidar spin down request")
            
            #check for proper response from lidar stop request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

                if ack == "ACK (response)" and cmd_set == "Lidar" and cmd_id == "0":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to spin down the lidar")
                    else:
                        self._isData = False
                        if self._captureStream is not None:
                            self._captureStream.stop()
                            self._captureStream = None
                        self._isWriting = False
                        time.sleep(0.1)
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect lidar spin down response")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
    
    
    def lidarStandBy(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_LIDAR_STANDBY,(self._sensorIP, 65000))
            if self.showMessages: print("   " + self._sensorIP + " <-- sent lidar stand-by request")
            
            #check for proper response from lidar stand-by request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

                if ack == "ACK (response)" and cmd_set == "Lidar" and cmd_id == "0":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to set lidar to stand-by")
                    else:
                        self._isData = False
                        if self._captureStream is not None:
                            self._captureStream.stop()
                            self._captureStream = None
                        self._isWriting = False
                        time.sleep(0.1)
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect lidar stand-by response")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
            
    def dataStart(self):
        
        if self._isConnected:
            if not self._isData:
                self._captureStream = _dataCaptureThread(self._sensorIP, self._dataSocket, "",0 ,0, 0, 0, self.showMessages)
                time.sleep(0.12)
                self._waitForIdle()
                self._cmdSocket.sendto(self._CMD_DATA_START,(self._sensorIP, 65000))
                if self.showMessages: print("   " + self._sensorIP + " <-- sent start data stream request")
            
                #check for proper response from data start request
                if select.select([self._cmdSocket],[],[],0.1)[0]:
                    binData, addr = self._cmdSocket.recvfrom(16)
                    _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
    
                    if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "4":
                        ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                        if ret_code == 1:
                            if self.showMessages: print("   " + self._sensorIP + " --> FAILED to start data stream")
                            if self._captureStream is not None:
                                self._captureStream.stop()
                            time.sleep(0.1)
                            self._isData = False
                        else:
                            self._isData = True
                    else:
                        if self.showMessages: print("   " + self._sensorIP + " --> incorrect start data stream response")
            else:
                if self.showMessages: print("   " + self._sensorIP + " --> data stream already started")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
    def dataStart_RT(self):
        
        if self._isConnected:
            if not self._isData:
                self._captureStream = _dataCaptureThread(self._sensorIP, self._dataSocket, "",1 ,0, 0, 0, self.showMessages)
                time.sleep(0.12)
                self._waitForIdle()
                self._cmdSocket.sendto(self._CMD_DATA_START,(self._sensorIP, 65000))
                if self.showMessages: print("   " + self._sensorIP + " <-- sent start data stream request")
            
                #check for proper response from data start request
                if select.select([self._cmdSocket],[],[],0.1)[0]:
                    binData, addr = self._cmdSocket.recvfrom(16)
                    _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
    
                    if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "4":
                        ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                        if ret_code == 1:
                            if self.showMessages: print("   " + self._sensorIP + " --> FAILED to start data stream")
                            if self._captureStream is not None:
                                self._captureStream.stop()
                            time.sleep(0.1)
                            self._isData = False
                        else:
                            self._isData = True
                    else:
                        if self.showMessages: print("   " + self._sensorIP + " --> incorrect start data stream response")
            else:
                if self.showMessages: print("   " + self._sensorIP + " --> data stream already started")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
    
    def dataStop(self):
        
        if self._isConnected:
            if self._isData:
                self._waitForIdle()
                self._cmdSocket.sendto(self._CMD_DATA_STOP,(self._sensorIP, 65000))
                if self.showMessages: print("   " + self._sensorIP + " <-- sent stop data stream request")
                
                #check for proper response from data stop request
                if select.select([self._cmdSocket],[],[],0.1)[0]:
                    binData, addr = self._cmdSocket.recvfrom(16)
                    _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
    
                    if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "4":
                        ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                        if ret_code == 1:
                            if self.showMessages: print("   " + self._sensorIP + " --> FAILED to stop data stream")
                        else:
                            self._isData = False
                            if self._captureStream is not None:
                                self._captureStream.stop()
                                self._captureStream = None
                            self._isWriting = False
                            time.sleep(0.1)
                    else:
                        if self.showMessages: print("   " + self._sensorIP + " --> incorrect stop data stream response")
            else:
                if self.showMessages: print("   " + self._sensorIP + " --> data stream already stopped")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
    
    def setDynamicIP(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_DYNAMIC_IP,(self._sensorIP, 65000))
            
            #check for proper response from dynamic IP request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

                if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "8":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 0:
                        if self.showMessages: print("Changed IP from " + self._sensorIP + " to dynamic IP (DHCP assigned)")
                        self.disconnect()
                        
                        if self.showMessages: print("\n********** PROGRAM ENDED - MUST REBOOT SENSOR **********\n")
                        sys.exit(4)
                    else:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to change to dynamic IP (DHCP assigned)")
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> FAILED to change to dynamic IP (DHCP assigned)")
            
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
    
    
    def setStaticIP(self, ipAddress):
        
        if self._isConnected:
            ipRange = ""
            if self._ipRangeCode == 1:
                ipRange = "192.168.1.11 to .80"
            elif self._ipRangeCode == 2:
                ipRange = "192.168.1.81 to .150"
            elif self._ipRangeCode == 3:
                ipRange = "192.168.1.151 to .220"
            ipAddress = self._checkIP(ipAddress)
            if ipAddress:
                IP_parts = ipAddress.split(".")
                IPhex1 = str(hex(int(IP_parts[0]))).replace('0x','').zfill(2)
                IPhex2 = str(hex(int(IP_parts[1]))).replace('0x','').zfill(2)
                IPhex3 = str(hex(int(IP_parts[2]))).replace('0x','').zfill(2)
                IPhex4 = str(hex(int(IP_parts[3]))).replace('0x','').zfill(2)
                formattedIP = IP_parts[0].strip() + "." + IP_parts[1].strip() + "." + IP_parts[2].strip() + "." + IP_parts[3].strip()
                cmdString = "aa011400000000a824000801" + IPhex1 + IPhex2 + IPhex3 + IPhex4
                binString = bytes(cmdString, encoding='utf-8')
                crc32checksum = self._crc32fromStr(binString)
                cmdString += crc32checksum
                binString = bytes(cmdString, encoding='utf-8')
                
                staticIP_request = bytes.fromhex((binString).decode('ascii'))
                self._waitForIdle()
                self._cmdSocket.sendto(staticIP_request,(self._sensorIP, 65000))
                
                #check for proper response from static IP request
                if select.select([self._cmdSocket],[],[],0.1)[0]:
                    binData, addr = self._cmdSocket.recvfrom(16)
                    _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
    
                    if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "8":
                        ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                        
                        if ret_code == 0:
                            if self.showMessages: print("Changed IP from " + self._sensorIP + " to a static IP of " + formattedIP)
                            self.disconnect()
                            
                            if self.showMessages: print("\n********** PROGRAM ENDED - MUST REBOOT SENSOR **********\n")
                            sys.exit(5)
                        else:
                            if self.showMessages: print("   " + self._sensorIP + " --> FAILED to change static IP (must be " + ipRange + ")")
                    else:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to change static IP (must be " + ipRange + ")")
            else:
                if self.showMessages: print("   " + self._sensorIP + " --> FAILED to change static IP (must be " + ipRange + ")")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
        
    
    def setCartesianCS(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_CARTESIAN_CS,(self._sensorIP, 65000))
            if self.showMessages: print("   " + self._sensorIP + " <-- sent change to Cartesian coordinates request")
            
            #check for proper response from change coordinate system request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
                
                if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "5":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to set Cartesian coordinate output")
                    elif ret_code == 0:
                        self._coordSystem = 0
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect change coordinate system response (Cartesian)")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
            
    def setSphericalCS(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_SPHERICAL_CS,(self._sensorIP, 65000))
            if self.showMessages: print("   " + self._sensorIP + " <-- sent change to Spherical coordinates request")
            
            #check for proper response from change coordinate system request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
                
                if ack == "ACK (response)" and cmd_set == "General" and cmd_id == "5":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to set Spherical coordinate output")
                    elif ret_code == 0:
                        self._coordSystem = 1
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect change coordinate system response (Spherical)")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)


    def readExtrinsic(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_READ_EXTRINSIC,(self._sensorIP, 65000))
            if self.showMessages: print("   " + self._sensorIP + " <-- sent read extrinsic parameters request")
            
            #check for proper response from read extrinsics request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(40)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

                if ack == "ACK (response)" and cmd_set == "Lidar" and cmd_id == "2":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to read extrinsic parameters")                
                    elif ret_code == 0:
                        roll = struct.unpack('<f',binData[12:16])[0]
                        pitch = struct.unpack('<f',binData[16:20])[0]
                        yaw = struct.unpack('<f',binData[20:24])[0]
                        x = float(struct.unpack('<i',binData[24:28])[0]) / 1000.
                        y = float(struct.unpack('<i',binData[28:32])[0]) / 1000.
                        z = float(struct.unpack('<i',binData[32:36])[0]) / 1000.
                        
                        self._x = x
                        self._y = y
                        self._z = z
                        self._roll = roll
                        self._pitch = pitch
                        self._yaw = yaw
                        
                        #called only to print the extrinsic parameters to the screen if showMessages = True
                        ack = self.extrinsicParameters()
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect read extrinsics response")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
    
    
    def setExtrinsicToZero(self):
        
        if self._isConnected:
            self._waitForIdle()
            self._cmdSocket.sendto(self._CMD_WRITE_ZERO_EXTRINSIC,(self._sensorIP, 65000))
            if self.showMessages: print("   " + self._sensorIP + " <-- sent set extrinsic parameters to zero request")
            
            #check for proper response from write extrinsics request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)

                if ack == "ACK (response)" and cmd_set == "Lidar" and cmd_id == "1":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to set extrinsic parameters to zero")                
                    elif ret_code == 0:
                        self.readExtrinsic()
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect set extrinsics to zero response")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
    
    def setExtrinsicTo(self, x, y, z, roll, pitch, yaw):
        
        if self._isConnected:
            goodValues = True
            try:
                xi = int(np.floor(x * 1000.))    #units of millimeters
                yi = int(np.floor(y * 1000.))    #units of millimeters
                zi = int(np.floor(z * 1000.))    #units of millimeters
                rollf = float(roll)
                pitchf = float(pitch)
                yawf = float(yaw)
                
            except:
                goodValues = False
                if self.showMessages: print("*** Error - one or more of the extrinsic values specified are not of the correct type ***")
            
            if goodValues:
                h_x = str(binascii.hexlify(struct.pack('<i',xi)))[2:-1]
                h_y = str(binascii.hexlify(struct.pack('<i',yi)))[2:-1]
                h_z = str(binascii.hexlify(struct.pack('<i',zi)))[2:-1]
                h_roll = str(binascii.hexlify(struct.pack('<f',rollf)))[2:-1]
                h_pitch = str(binascii.hexlify(struct.pack('<f',pitchf)))[2:-1]
                h_yaw = str(binascii.hexlify(struct.pack('<f',yawf)))[2:-1]

                cmdString = "aa012700000000b5ed0101" + h_roll + h_pitch + h_yaw + h_x + h_y + h_z
                binString = bytes(cmdString, encoding='utf-8')
                crc32checksum = self._crc32fromStr(binString)
                cmdString += crc32checksum
                binString = bytes(cmdString, encoding='utf-8')
                setExtValues = bytes.fromhex((binString).decode('ascii'))
                
                self._waitForIdle()
                self._cmdSocket.sendto(setExtValues,(self._sensorIP, 65000))
                if self.showMessages: print("   " + self._sensorIP + " <-- sent set extrinsic parameters request")
                
                #check for proper response from write extrinsics request
                if select.select([self._cmdSocket],[],[],0.1)[0]:
                    binData, addr = self._cmdSocket.recvfrom(16)
                    _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
    
                    if ack == "ACK (response)" and cmd_set == "Lidar" and cmd_id == "1":
                        ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                        if ret_code == 1:
                            if self.showMessages: print("   " + self._sensorIP + " --> FAILED to set extrinsic parameters")                
                        elif ret_code == 0:
                            self.readExtrinsic()
                    else:
                        if self.showMessages: print("   " + self._sensorIP + " --> incorrect set extrinsic parameters response")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
    
    def setRainFogSuppression(self, OnOff):
        
        if self._isConnected:
            self._waitForIdle()
            if OnOff:
                self._cmdSocket.sendto(self._CMD_RAIN_FOG_ON,(self._sensorIP, 65000))
                if self.showMessages: print("   " + self._sensorIP + " <-- sent turn on rain/fog suppression request")
            else:
                self._cmdSocket.sendto(self._CMD_RAIN_FOG_OFF,(self._sensorIP, 65000))
                if self.showMessages: print("   " + self._sensorIP + " <-- sent turn off rain/fog suppression request")
            
            #check for proper response from change rain/fog suppression request
            if select.select([self._cmdSocket],[],[],0.1)[0]:
                binData, addr = self._cmdSocket.recvfrom(16)
                _,ack,cmd_set,cmd_id,ret_code_bin = self._parseResp(binData)
                
                if ack == "ACK (response)" and cmd_set == "Lidar" and cmd_id == "3":
                    ret_code = int.from_bytes(ret_code_bin[0], byteorder='little')
                    if ret_code == 1:
                        if self.showMessages: print("   " + self._sensorIP + " --> FAILED to set rain/fog suppression value")
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> incorrect set rain/fog suppression response")
        else:
            if self.showMessages: print("Not connected to Mid-40 sensor at IP: " + self._sensorIP)
            
            
    def saveDataToCSV(self, filePathAndName, secsToWait, duration):
        
        if self._isConnected:
            if self._isData:
                if self._firmware != "UNKNOWN":
                    try:
                        firmwareType = self._SPECIAL_FIRMWARE_TYPE_DICT[self._firmware]
                    except:
                        firmwareType = 1
    
                    if duration < 0:
                        if self.showMessages: print("   " + self._sensorIP + " --> * ISSUE: saving data, negative duration")
                    else:
                        #max duration = 4 years - 1 sec
                        if duration >= 126230400:
                            if self.showMessages: print("   " + self._sensorIP + " --> * ISSUE: saving data, duration too big")
                        else:
                        
                            if secsToWait < 0:
                                if self.showMessages: print("   " + self._sensorIP + " --> * ISSUE: saving data, negative time to wait")
                            else:
                                #max time to wait = 15 mins
                                if secsToWait > 900:
                                    if self.showMessages: print("   " + self._sensorIP + " --> * ISSUE: saving data, time to wait too big")
                                else:
                                
                                    if filePathAndName == "":
                                        if self.showMessages: print("   " + self._sensorIP + " --> * ISSUE: saving data, file path and name missing")
                                    else:
                                        
                                        if filePathAndName[-4:].upper() != ".CSV":
                                            filePathAndName += ".csv"

                                        self._isWriting = True
                                        self._captureStream.filePathAndName = filePathAndName
                                        self._captureStream.secsToWait = secsToWait
                                        self._captureStream.duration = duration
                                        self._captureStream.firmwareType = firmwareType
                                        self._captureStream.showMessages = self.showMessages
                                        time.sleep(0.1)
                                        self._captureStream.isCapturing = True
                else:
                    if self.showMessages: print("   " + self._sensorIP + " --> unknown firmware version")
            else:
                if self.showMessages: print("   " + self._sensorIP + " --> WARNING: data stream not started, no CSV file created")

        
    def closeCSV(self):
        if self._isConnected:
            if self._isWriting:
                if self._captureStream is not None:
                    self._captureStream.stop()
                self._isWriting = False
    
    
    def resetShowMessages(self):
        self.showMessages = self._init_showMessages
    
    
    def connectionParameters(self):
        if self._isConnected:
            if self.showMessages:
                print("      Computer IP Address: " + self._computerIP)
                print("      Sensor IP Address:   " + self._sensorIP)
                print("      Data Port Number:    " + str(self._dataPort))
                print("      Command Port Number: " + str(self._cmdPort))
                
            return [self._computerIP, self._sensorIP, str(self._dataPort), str(self._cmdPort)]
    
    
    def extrinsicParameters(self):
        if self._isConnected:
            if self.showMessages: 
                print("      x: " + str(self._x) + " m")
                print("      y: " + str(self._y) + " m")
                print("      z: " + str(self._z) + " m")
                print("      roll: " + "{0:.2f}".format(self._roll) + " deg")
                print("      pitch: " + "{0:.2f}".format(self._pitch) + " deg")
                print("      yaw: " + "{0:.2f}".format(self._yaw) + " deg")
        
            return [self._x, self._y, self._z, self._roll, self._pitch, self._yaw]
    
    
    def firmware(self):
        if self._isConnected:
            if self.showMessages: print("   " + self._sensorIP + " --> F/W Version: " + self._firmware)
        
            return self._firmware
    
    
    def serialNumber(self):
        if self._isConnected:
            if self.showMessages: print("   " + self._sensorIP + " --> Serial # " + self._serial)
        
            return self._serial
    
    
    def lidarStatusCodes(self):
        if self._isConnected:
            if self._captureStream is not None:
                codes = self._captureStream.statusCodes()
                if self.showMessages:
                    sys_mess = "UNKNOWN"
                    if codes[0] == 0:
                        sys_mess = "OK"
                    elif codes[0] == 1:
                        sys_mess = "* WARNING *"
                    elif codes[0] == 2:
                        sys_mess = "*** ERROR ***"
                        
                    temp_mess = "UNKNOWN"
                    if codes[1] == 0:
                        temp_mess = "OK"
                    elif codes[1] == 1:
                        temp_mess = "High/Low Warning"
                    elif codes[1] == 2:
                        temp_mess = "Extremely High/Low Error"
                        
                    volt_mess = "UNKNOWN"
                    if codes[2] == 0:
                        volt_mess = "OK"
                    elif codes[2] == 1:
                        volt_mess = "High Warning"
                    elif codes[2] == 2:
                        volt_mess = "Extremely High Error"
                        
                    motor_mess = "UNKNOWN"
                    if codes[3] == 0:
                        motor_mess = "OK"
                    elif codes[3] == 1:
                        motor_mess = "Warning State"
                    elif codes[3] == 2:
                        motor_mess = "Error State"
                        
                    dirty_mess = "UNKNOWN"
                    if codes[4] == 0:
                        dirty_mess = "OK"
                    elif codes[4] == 1:
                        dirty_mess = "Dirty/Blocked Warning"
                        
                    firmware_mess = "UNKNOWN"
                    if codes[5] == 0:
                        firmware_mess = "OK"
                    elif codes[5] == 1:
                        firmware_mess = "Abnormal Error"
                        
                    pps_mess = "UNKNOWN"
                    if codes[6] == 0:
                        pps_mess = "OK, but not detected"
                    elif codes[6] == 1:
                        pps_mess = "OK"
                        
                    device_mess = "UNKNOWN"
                    if codes[7] == 0:
                        device_mess = "OK"
                    elif codes[7] == 1:
                        device_mess = "Approaching End of Service Life Warning"
                    
                    print("      System Status:       " + sys_mess)
                    print("      Temperature Status:  " + temp_mess)
                    print("      Voltage Status:      " + volt_mess)
                    print("      Motor Status:        " + motor_mess)
                    print("      Clean Status:        " + dirty_mess)
                    print("      Firmware Status:     " + firmware_mess)
                    print("      PPS Status:          " + pps_mess)
                    print("      Device Status:       " + device_mess)
                
                return codes
            
            else:
                if self.showMessages:
                    
                    print("      System Status:       UNKNOWN")
                    print("      Temperature Status:  UNKNOWN")
                    print("      Voltage Status:      UNKNOWN")
                    print("      Motor Status:        UNKNOWN")
                    print("      Dirty Status:        UNKNOWN")
                    print("      Firmware Status:     UNKNOWN")
                    print("      PPS Status:          UNKNOWN")
                    print("      Device Status:       UNKNOWN")
                    
                return [-1,-1,-1,-1,-1,-1,-1,-1]
    
    
    def doneCapturing(self):
        #small sleep to ensure this command isn't continuously called if in a while True loop
        time.sleep(0.01)
        if self._captureStream is not None:
            if self._captureStream.duration != 126230400:
                return not(self._captureStream.started)
            else:
                return True
        else:
            return True
        
        
def allDoneCapturing(sensors):
    
    stop = []
    for i in range(0,len(sensors)):
        if sensors[i]._captureStream is not None:
                stop.append(sensors[i].doneCapturing())
    
    #small sleep to ensure this command isn't continuously called if in a while True loop
    time.sleep(0.01)
    return all(stop)
    
    
    
        
                
    
    

