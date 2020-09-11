#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

Started on Mon. May 13th 2019

@author: Ryan Brazeal
@email: ryan.brazeal@ufl.edu

Program Name: livox_controller_demo.py
Version: 1.1.0
Last Updated: Fri. Sept. 11th 2020 (NEVER FORGET!)

Description: Python3 demo for controlling a single or multiple Livox sensor(s) using OpenPyLivox


"""

# openpylivox library import
import openpylivox as opl

# only used for this demo
import time
import sys


# demo operations for a single Livox Sensor
def singleSensorDemo():

    # create an openpylivox object
    sensor = opl.openpylivox(True)  # optional True/False, is used to have sensor messages printed to screen (default = False)

    # automatically discover if any Livox Sensors are available on the network
    # sensor.discover()

    # automatically discover if any Livox Sensors are available on the network, using a specific computer IP address
    # sensor.discover("192.168.1.23")

    # the sensor object's showMessages method can be used to turn on & off its information messages (example further down)
    sensor.showMessages(False)

    # sensor object's information messages can always be reset back to the initial state (on/off) as defined at the time of instantiation
    sensor.resetShowMessages()

    # easiest to try to automatically set the openpylivox sensor connection parameters and connect
    connected = sensor.auto_connect()

    # or if your computer has multiple IP address you can force the computer IP to a manual address
    # connected = sensor.auto_connect("192.168.1.23")

    # or manually define all IP addresses and ports (still need to properly configure your IP, Subnet, etc. on your computer)
    #                            computer IP       sensorIP    data port  command port  IMU port
    # connected = sensor.connect("192.168.1.23", "192.168.1.118",  60001,     50001,      40001)

    # make sure a sensor was connected
    if connected:

        # the sensor's connection parameters can be returned as a list of strings
        connParams = sensor.connectionParameters()

        # the sensor's firmware version can be returned as a list of strings
        firmware = sensor.firmware()

        # the sensor's serial number can be returned as a string
        serial = sensor.serialNumber()

        sensor.showMessages(True)

        # set the output coordinate system to Spherical
        # sensor.setSphericalCS()

        # set the output coordinate system to Cartesian (sensor default)
        # sensor.setCartesianCS()

        # read current extrinsic values from the sensor
        # sensor.readExtrinsic()

        # set all the sensor's extrinsic parameters equal to zero
        # (*** IMPORTANT: does not affect raw point cloud data stream, seems to only be used in Livox-Viewer? ***)
        # sensor.setExtrinsicToZero()

        x = 12.345  # units of meters
        y = -23.456  # units of meters
        z = 34.567  # units of meters
        roll = -0.05  # units of degrees
        pitch = 8.77  # units of degrees
        yaw = -174.14  # units of degrees

        # set the sensor's extrinsic parameters to specific values
        # (*** IMPORTANT: does not affect raw point cloud data stream, seems to only be used in Livox-Viewer? ***)
        # sensor.setExtrinsicTo(x, y, z, roll, pitch, yaw)

        # the sensor's extrinsic parameters can be returned as a list of floats
        # extParams = sensor.extrinsicParameters()

        sensor.resetShowMessages()

        # make sure the lidar is spinning (ie., in normal mode), safe to call if the lidar is already spinning
        sensor.lidarSpinUp()

        ##########################################################################################
        # if you want to set a static IP address on the sensor
        # program is force killed as the sensor needs to be rebooted for IP changes to be set

        # sensor.setStaticIP("192.168.1.40")

        # if you want to set IP address on the sensor to dynamic (ie., assigned by a DHCP server)
        # program is force killed as the sensor needs to be rebooted for IP changes to be set

        # sensor.setDynamicIP()

        ##########################################################################################

        # start data stream (real-time writing of point cloud data to a BINARY file)
        sensor.dataStart_RT_B()

        # send UTC time update (only works in conjunction with a hardware-based PPS pulse)
        sensor.updateUTC(2001, 1, 1, 1, 0)

        # set lidar return mode (0 = single 1st return, 1 = single strongest return, 2 = dual returns)
        sensor.setLidarReturnMode(2)

        # activate the IMU data stream (only for Horizon and Tele-15 sensors)
        sensor.setIMUdataPush(True)

        # turn on (True) or off (False) rain/fog suppression on the sensor
        sensor.setRainFogSuppression(True)

        # the sensor's lidar status codes can be returned as a list of integers (data stream needs to be started first)
        # status = sensor.lidarStatusCodes()

        # not exactly sure what this does as the fan does not respond to On(True) or Off(False) requests?
        # sensor.setFan(True)

        filePathAndName = "test.bin"  # file extension is NOT used to automatically determine if ASCII or Binary data is stored
        secsToWait = 0.1  # seconds, time delayed data capture start
        duration = 3.0  # seconds, zero (0) specifies an indefinite duration

        # (*** IMPORTANT: this command starts a new thread, so the current program (thread) needs to exist for the 'duration' ***)
        # capture the data stream and save it to a file (if applicable, IMU data stream will also be saved to a file)
        sensor.saveDataToFile(filePathAndName, secsToWait, duration)

        # simulate other operations being performed
        while True:
            # 1 + 1 = 2

            # time.sleep(3)   #example of time < (duration + secsToWait), therefore early data capture stop

            # close the output data file, even if duration has not occurred (ideally used when duration = 0)
            # sensor.closeFile()
            # break

            # exit loop when capturing is complete (*** IMPORTANT: ignores (does not check) sensors with duration set to 0)
            if sensor.doneCapturing():
                break

        # NOTE: Any one of the following commands will also close the output data file (if still being written)

        # stop data stream
        sensor.dataStop()

        # if you want to put the lidar in stand-by mode, not sure exactly what this does, lidar still spins?
        # sensor.lidarStandBy()

        # if you want to stop the lidar from spinning (ie., lidar to power-save mode)
        sensor.lidarSpinDown()

        # if you want to reboot the sensor
        # sensor.reboot()

        # properly disconnect from the sensor
        sensor.disconnect()

        # convert BINARY point data to LAS file and IMU data (if applicable) to CSV file
        # only works in conjunction with .dataStart_RT_B()
        # designed so the efficiently collected binary point data can be converted to LAS at any time after data collection
        opl.convertBin2LAS(filePathAndName, deleteBin=True)

        # convert BINARY point data and IMU data (if applicable) to CSV files
        # only works in conjunction with .dataStart_RT_B()
        # designed so the efficiently collected binary data can be converted to CSV at any time after data collection
        # opl.convertBin2CSV(filePathAndName, deleteBin=True)

    else:
        print("\n***** Could not connect to a Livox sensor *****\n")


# demo operations for automatically connecting to multiple Livox Sensors
def multipleSensorsDemo():

    # simple list to collect individual sensor objects
    sensors = []
    filePathAndNames = []

    connected = 0
    # loop through and find and connect to all sensors
    while connected != 1:

        # instantiate an openpylivox sensor object
        sensor = opl.openpylivox(True)

        # automatically find and connect to a sensor
        connected = sensor.auto_connect("192.168.1.23")

        if connected:
            # initial commands for each sensor (no harm if the parameter is not supported by sensor)
            sensor.setCartesianCS()
            sensor.setRainFogSuppression(False)
            sensor.setLidarReturnMode(0)
            sensor.setIMUdataPush(True)

            # append to sensor objects list
            sensors.append(sensor)

    # make sure a sensor was found and connected
    if sensors:
        # spin up all the sensors
        for i in range(0, len(sensors)):
            sensors[i].lidarSpinUp()

        # start all their data streams
        for i in range(0, len(sensors)):
            sensors[i].dataStart_RT_B()

        # save data from all the sensors to individual data files, using sensor's serial # as filename
        for i in range(0, len(sensors)):
            sensors[i].showMessages(False)
            # for Mid-100 sensors the filename specified is used for the data from the left sensor
            # filename_M for the middle sensor, and filename_R for the right sensor
            filename = sensors[i].serialNumber() + ".bin"
            filePathAndNames.append(filename)
            sensors[i].resetShowMessages()

            sensors[i].saveDataToFile(filename, 0.0, 5.0)

        # simulate other operations being performed
        while True:
            # 1 + 1 = 2

            # utility function to exit loop when capturing is complete from ALL sensors (*** IMPORTANT: ignores
            # (does not check) sensors with duration set to 0)
            if opl.allDoneCapturing(sensors):
                break

        # stop data on all the sensors
        for i in range(0, len(sensors)):
            sensors[i].dataStop()

        # spin down all the sensors
        for i in range(0, len(sensors)):
            sensors[i].lidarSpinDown()

        # disconnect all the sensors and convert binary data files to ASCII-based files
        for i in range(0, len(sensors)):
            sensors[i].disconnect()

            # convert BINARY point data to LAS file and IMU data (if applicable) to CSV file
            # no harm done if filePathAndName is an ASCII CSV file, the conversion will be skipped
            opl.convertBin2LAS(filePathAndNames[i], deleteBin=True)


if __name__ == '__main__':

    singleSensorDemo()
    # multipleSensorsDemo()
