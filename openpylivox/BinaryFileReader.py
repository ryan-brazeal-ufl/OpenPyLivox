#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Started on September 11th, 2020

@author: Anjali Zope
@email: anju.zope@gmail.com

Program Name: BinaryFileReader.py
Version: 1.0.0

Description: class for reading livox horizon binary data files

Change Log:
    - v1.0.0 released - Sept. 11th 2020

"""

# Required Import Statements
import struct
import numpy as np



class BinaryReaders:
    '''
    A class used for reading binary LIDAR datasets and point clouds
    ----------
    '''

    def __init__(self, datapoints, imudata, datatype=None):
        if datapoints is None:
            self.datapoints = []
        else:
            self.datapoints = datapoints
        if imudata is None:
            self.imudata = []
        else:
            self.imudata = imudata

        self.datatype = datatype


    @staticmethod
    def simplecloudreader(pathtofile, showmessages=False):
        '''
        A method which reads 3D point clouds (NOT livox horizon files) with a header.
        The public header block is 27 bytes long and consists of
        the string 9 char string "point_num", the 6 byte character string "x_coor", the 6 byte char string "y-coor" and
        the 6 byte char string "z_coor".
        After the public header block are data blocks that are 28 bytes long which contain a 4 byte int type point
        number, an 8 byte double type x coordinate, an 8 byte double type y coordinate and an 8 byte double type z
        coordinate.

        Parameters
        ----------
        pathtofile : str
            String containing the path to the binary point cloud file.

        Returns
        -------
        pointcloud : numpy.ndarray
            A single numpy array of the point numbers, x, y and z coordinates. Each entry in this array is of type
            numpy.void. Access the columns of this array using indices (0,1,2,3) after converting to a list and then a
            numpy array OR - more straightforwardly - use the column "field names" as before with other numpy arrays
            i.e. pointcloud['fieldname'] returns a column vector of that particular fieldname. To check the field names,
            use pointcloud.dtypes.names as before.

        '''

        # How many blocks have we gone through (keep track of our progress through the file)
        counter = 0

        # The size (in bytes) of the public header block
        headersize = 27
        # The size (in bytes) of the subsequent data blocks
        datablocksize = 28

        # Format character string for the header block buffer
        # the < means the binary file block is little endian
        # the 9 means there are 9 of the next data type
        # the s means a string data type, so 9s means read in a string of 9 characters
        # the subsequent repeated 6s mean the same thing - the next chunk of data in this block is a character string
        # of length 6 the number of bytes in the block being read must match the number of bytes required to store the
        # format string
        hblockfms = '<9s6s6s6s'

        # Format character string for the data block buffer
        dblockfms = '<iddd'

        # Open the binary file with the open statement and the flag 'rb' as f. 'r' in 'rb' indicates the file should be
        # opened as read only and 'b' in 'rb' indicates the file should be opened as a binary file
        with open(pathtofile, 'rb') as f:
            # While I can still read data from the file
            while True:
                # If we're reading in the first block (the header block)
                if counter < 1:
                    # read in a block of size headersize
                    buffer = f.read(headersize)

                    # If the buffer returns nothing, stop reading the file
                    if not buffer:
                        break
                    # unpack the binary data in the buffer using the format string hblockfms
                    header = struct.unpack(hblockfms, buffer)
                    # decode the entries in the header block to the 'utf-8' format
                    header1 = header[0].decode('utf-8')
                    header2 = header[1].decode('utf-8')
                    header3 = header[2].decode('utf-8')
                    header4 = header[3].decode('utf-8')
                    if showmessages:
                        print("The header values are:", header1, header2, header3, header4)

                    # Create a numpy.void array for holding our values
                    # first define the field 'names' for the numpy array which were read in from the header block,
                    # and the data types of each column of the numpy array (known beforehand from the PDF)
                    dt = {'names': [header1, header2, header3, header4],
                          'formats': [np.int64, np.double, np.double, np.double]}

                    # Now actually create the array
                    # We've initialized it as an array of 4 zeros, and will append in the elif statements
                    # Specifically it is one row (hence the 1 in the statement below) of 4 zeros with each column of
                    # that row having the datatype specified by dt above
                    pointcloud = np.zeros(1, dtype=dt)
                    if showmessages:
                        print("The pointcloud array looks like:")
                        print(pointcloud)
                    # How could this statement be written more pythonically? Google increment/decrement operators in
                    # python
                    counter += 1
                # If we're reading in the first block of data (data blocks)
                elif counter == 1:
                    # Read in the numerical data from the buffer of size datablocksize
                    buffer = f.read(datablocksize)

                    # If the buffer returns nothing, stop reading the file (shouldn't happen)
                    if not buffer:
                        break
                    # Unpack the numerical data in the buffer using the formatstring dblockfms
                    # Because the data is numeric, we don't need to do any decoding
                    datablock = struct.unpack(dblockfms, buffer)
                    dblock1 = datablock[0]
                    dblock2 = datablock[1]
                    dblock3 = datablock[2]
                    dblock4 = datablock[3]

                    # throw the first value (the integer) from the first datablock that we're reading into
                    # the first entry (0) of the column with the label header1
                    pointcloud[header1][0] = dblock1
                    # throw the second value (the x coordinate double) from the first datablock that we're reading into
                    # the first entry (0) of the column with the label header2
                    pointcloud[header2][0] = dblock2
                    # throw the third value (the y coordinate double) from the first datablock that we're reading into
                    # the first entry (0) of the column with the label header3
                    pointcloud[header3][0] = dblock3
                    # throw the fourth value (the z coordinate double) from the first datablock that we're reading into
                    # the first entry (0) of the column with the label header4
                    pointcloud[header4][0] = dblock4

                    # increment our counter as we read through the blocks of binary data
                    counter = counter + 1

                # If we're reading any subsequent blocks of binary data, let's just append the data to our numpy array
                elif counter > 1:
                    # Read in the data from the buffer as in the previous elif statement
                    buffer = f.read(datablocksize)

                    # If the buffer returns nothing, stop reading the file
                    if not buffer:
                        break

                    # unpack everything as before
                    datablock = struct.unpack(dblockfms, buffer)
                    dblock1 = datablock[0]
                    dblock2 = datablock[1]
                    dblock3 = datablock[2]
                    dblock4 = datablock[3]

                    # Put these 4 entries into a little numpy array with the same datatypes as declared in the first if
                    # statement
                    temp = np.array((dblock1, dblock2, dblock3, dblock4), dtype=dt)

                    # Append the temporary array to the pointcloud array accordingly:
                    pointcloud = np.append(pointcloud, temp)

                    # Increment the counter
                    counter = counter + 1

        # return the 4 column numpy array with field names that were read in from the public header block
        return pointcloud

    @staticmethod
    def data_type0_reader(fobj, totalbytesread, datatype_points, datasize_0=13, showmessages=False):
        '''
                A method for reading in data type 0 points

                Parameters
                ----------
                fobj : BinaryIO
                    File object for the .lvx file being read
                totalbytesread : int
                    The total bytes read by the file reader up to this package
                datatype_points : list
                    list containing all points of data type 0 in the entire file
                datasize_0 : int
                    Size of the type 0 points data type (page 5 .lvx documentation)

                Returns
                -------
                totalbytesread : int
                    Number of bytes read in the file including this package
                points_type_0 : list
                    A python list of all 100 points in this data type 0 package
                datatype_points : list
                    list containing all points of data type 0 in the entire file (read so far)

                '''
        newcounter = 1
        if showmessages:
            print("Type 0 points being read...")
        # create empty list
        points_type_0 = [];
        while newcounter <= 100:
            # print("Point", newcounter)
            buf6 = fobj.read(datasize_0)
            value6 = struct.unpack('<iiiB', buf6)
            # add values to list
            points_type_0.append(value6)
            if (value6[0] != 0 and value6[1] != 0 and value6[2] != 0):
                # X, Y, Z, reflect., return num.
                pt = [0, 0, 0, 0, 0]
                pt[0] = value6[0] / 1000.
                pt[1] = value6[1] / 1000.
                pt[2] = value6[2] / 1000.
                pt[3] = value6[3]
                #return num (for Mid-40/100 special firmwares)
                pt[4] = 1
                if pt[3] == 200:
                    pt[4] = 2
                elif pt[3] == 250:
                    pt[4] = 3
                datatype_points.append(pt)
            # increment counter
            newcounter = newcounter + 1
            # add to total amount of bytes read
            totalbytesread += datasize_0
        # print the type 1 points in this package
        # print(points_type_0)

        return [totalbytesread, points_type_0, datatype_points]

    @staticmethod
    def data_type1_reader(fobj, totalbytesread, datatype_points, datasize_1=9, showmessages=False):
        '''
        A method for reading in data type 1 points

        Parameters
        ----------
        fobj : BinaryIO
            File object for the .lvx file being read
        totalbytesread : int
            The total bytes read by the file reader up to this package
        datatype_points : list
            a python list containing all points of data type 1 in the entire file
        datasize_1 : int
            Size of the type 1 points data type (page 6 .lvx documentation)

        Returns
        -------
        totalbytesread : int
            Number of bytes read in the file including this package
        points_type_1 : list of tuples
            A python list of all 100 points in this data type 1 package
            - we know 100 points per package of data type 1 from lvx documentation page 6
        datatype_points : list
            list containing all points of data type 1 in the file (read so far)

        '''
        newcounter2 = 1
        if showmessages:
            print("Type 1 points being read...")
        # create empty list
        points_type_1 = []
        while newcounter2 <= 100:
            # print("Point", newcounter2)
            buf6 = fobj.read(datasize_1)
            value6 = struct.unpack('<iHHB', buf6)
            # add values to list
            points_type_1.append(value6)
            if (value6[0] != 0 and value6[1] != 0 and value6[2] != 0):
                # Dist., Zen., Azi, reflect., return num.
                pt = [0, 0, 0, 0, 0]
                pt[0] = value6[0] / 1000.
                pt[1] = value6[1] / 100.
                pt[2] = value6[2] / 100.
                pt[3] = value6[3]
                # return num (for Mid-40/100 special firmwares)
                pt[4] = 1
                if pt[3] == 200:
                    pt[4] = 2
                elif pt[3] == 250:
                    pt[4] = 3
                datatype_points.append(pt)
            # increment counter
            newcounter2 = newcounter2 + 1
            # add to total amount of bytes read
            totalbytesread += datasize_1
        # print the type 1 points in this package
        # print(points_type_1)

        return [totalbytesread, points_type_1, datatype_points]

    @staticmethod
    def data_type2_reader(fobj, totalbytesread, datatype_points, datasize_2=14, showmessages=False):
        '''
        A method for reading in data type 2 points

        Parameters
        ----------
        fobj : BinaryIO
            File object for the .lvx file being read
        totalbytesread : int
            The total bytes read by the file reader up to this package
        datatype_points : list
            list containing all points of data type 2 in the entire file
        datasize_2 : int
            Size of the type 2 points data type (page 6 .lvx documentation)

        Returns
        -------
        totalbytesread : int
            Number of bytes read in the file including this package
        points_type_2 : list
            A python list of all 96 points in this
            data type 2 package
        datatype_points : list
            list containing all points of data type 2 in the entire file (read so far)

        '''
        newcounter2 = 1
        if showmessages:
            print("Type 2 points being read...")

        # create empty list
        points_type_2 = []
        while newcounter2 <= 96:
            # print("Point", newcounter2)
            buf6 = fobj.read(datasize_2)
            value6 = struct.unpack('<3i2B', buf6)
            # add values to list
            points_type_2.append(value6)
            if (value6[0] != 0 and value6[1] != 0 and value6[2] != 0):
                # X, Y, Z, reflect., return num., tag info.
                pt = [0, 0, 0, 0, 0, 0]
                pt[0] = value6[0] / 1000.
                pt[1] = value6[1] / 1000.
                pt[2] = value6[2] / 1000.
                pt[3] = value6[3]
                pt[4] = 1           #return number
                pt[5] = value6[4]   #tag information
                datatype_points.append(pt)
            # increment counter
            newcounter2 = newcounter2 + 1
            # add to total amount of bytes read
            totalbytesread += datasize_2
        # print the type 2 points in this package
        # print(points_type_2)

        return [totalbytesread, points_type_2, datatype_points]

    @staticmethod
    def data_type3_reader(fobj, totalbytesread, datatype_points, datasize_3=10, showmessages=False):
        '''
        A method for reading in data type 3 points

        Parameters
        ----------
        fobj : BinaryIO
            File object for the .lvx file being read
        totalbytesread : int
            The total bytes read by the file reader up to this package
        datatype_points : list
            list containing all points of data type 3 in the file
        datasize_3 : int
            Size of the type 3 points data type (page 6 .lvx documentation)

        Returns
        -------
        totalbytesread : int
            Number of bytes read in the file including this package
        points_type_3 : list
            A python list of all 96 points in this data type 3 package
        datatype_points : list
            A list of all points of data type 3 in the entire file (read so far)

        '''
        newcounter2 = 1
        if showmessages:
            print("Type 3 points being read...")
        # create empty list
        points_type_3 = []
        while newcounter2 <= 96:
            # print("Point", newcounter2)
            buf6 = fobj.read(datasize_3)
            value6 = struct.unpack('<iHHBB', buf6)
            # add values to list
            points_type_3.append(value6)
            if (value6[0] != 0 and value6[1] != 0 and value6[2] != 0):
                # Dist., Zen., Azi, reflect., return num., tag info.
                pt = [0, 0, 0, 0, 0, 0]
                pt[0] = value6[0] / 1000.
                pt[1] = value6[1] / 100.
                pt[2] = value6[2] / 100.
                pt[3] = value6[3]
                pt[4] = 1  # return number
                pt[5] = value6[4]  # tag information
                datatype_points.append(pt)
            # increment counter
            newcounter2 = newcounter2 + 1
            # add to total amount of bytes read
            totalbytesread += datasize_3
        # print the type 3 points in this package
        # print(points_type_3)

        return [totalbytesread, points_type_3, datatype_points]

    @staticmethod
    def data_type4_reader(fobj, totalbytesread, datatype_points, datasize_4=28, showmessages=False):
        '''
        A method for reading in data type 4 points

        Parameters
        ----------
        fobj : BinaryIO
            File object for the .lvx file being read
        totalbytesread : int
            The total bytes read by the file reader up to this package
        datatype_points : list
            list containing all points of data type 4 in the entire file
        datasize_4 : int
            Size of the type 4 points data type (page 6 .lvx documentation)

        Returns
        -------
        totalbytesread : int
            Number of bytes read in the file including this package
        points_type_4 : list
            A python list of all 48 points in this data type 4 package
        datatype_points : list
            A list of all data type 4 points in the file (read so far)

        '''
        newcounter2 = 1
        if showmessages:
            print("Type 4 points being read...")
        # create empty list
        points_type_4 = []
        while newcounter2 <= 48:
            # print("Point", newcounter2)
            buf6 = fobj.read(datasize_4)
            value6 = struct.unpack('<iiiBBiiiBB', buf6)
            # add values to list
            points_type_4.append(value6)
            if (value6[0] != 0 and value6[1] != 0 and value6[2] != 0):
                #X, Y, Z, reflect., return num., tag info.
                pt = [0, 0, 0, 0, 0, 0]
                pt[0] = value6[0] / 1000.
                pt[1] = value6[1] / 1000.
                pt[2] = value6[2] / 1000.
                pt[3] = value6[3]
                pt[4] = 1
                pt[5] = value6[4]
                datatype_points.append(pt)
            if (value6[5] != 0 and value6[6] != 0 and value6[7] != 0):
                #X, Y, Z, reflect., return num., tag info.
                pt = [0, 0, 0, 0, 0, 0]
                pt[0] = value6[5] / 1000.
                pt[1] = value6[6] / 1000.
                pt[2] = value6[7] / 1000.
                pt[3] = value6[8]
                pt[4] = 2
                pt[5] = value6[9]
                datatype_points.append(pt)
            # increment counter
            newcounter2 = newcounter2 + 1
            # add to total amount of bytes read
            totalbytesread += datasize_4
        # print the type 4 points in this package
        # print(points_type_4)

        return [totalbytesread, points_type_4, datatype_points]

    @staticmethod
    def data_type5_reader(fobj, totalbytesread, datatype_points, datasize_5=16, showmessages=False):
        '''
        A method for reading in data type 5 points

        Parameters
        ----------
        fobj : BinaryIO
            File object for the .lvx file being read
        totalbytesread : int
            The total bytes read by the file reader up to this package
        datatype_points : list
            list containing all points of data type 5 in the entire file
        datasize_5 : int
            Size of the type 5 points data type (page 7 .lvx documentation)

        Returns
        -------
        totalbytesread : int
            Number of bytes read in the file including this package
        points_type_5 : list
            A python list of all 48 points in this data type 5 package
        datatype_points : list
            A list of all data type 5 points in the file (read so far)

        '''
        newcounter2 = 1
        if showmessages:
            print("Type 5 points being read...")
        points_type_5 = []
        while newcounter2 <= 48:
            # print("Point", newcounter2)
            buf6 = fobj.read(datasize_5)
            value6 = struct.unpack('<HHiBBiBB', buf6)
            # add values to list
            points_type_5.append(value6)
            if (value6[0] != 0 and value6[1] != 0 and value6[2] != 0):
                # Dist., Zen., Azi, reflect., return num., tag info.
                pt = [0, 0, 0, 0, 0, 0]
                pt[0] = value6[2] / 1000.
                pt[1] = value6[0] / 100.
                pt[2] = value6[1] / 100.
                pt[3] = value6[3]
                pt[4] = 1
                pt[5] = value6[4]
                datatype_points.append(pt)
            if (value6[0] != 0 and value6[1] != 0 and value6[5] != 0):
                # Dist., Zen., Azi, reflect., return num., tag info.
                pt = [0, 0, 0, 0, 0, 0]
                pt[0] = value6[5] / 1000.
                pt[1] = value6[0] / 100.
                pt[2] = value6[1] / 100.
                pt[3] = value6[6]
                pt[4] = 2
                pt[5] = value6[7]
                datatype_points.append(pt)
            newcounter2 = newcounter2 + 1
            totalbytesread += datasize_5
        # print the type 5 points in this package
        # print(points_type_5)

        return [totalbytesread, points_type_5, datatype_points]

    @staticmethod
    def data_type6_reader(fobj, totalbytesread, datatype_points, datasize_6=24, showmessages=False):
        '''
        A method for reading in data type 6 points (IMU data)

        Parameters
        ----------
        fobj : BinaryIO
            File object for the .lvx file being read
        totalbytesread : int
            The total bytes read by the file reader up to this package
        datatype_points : list
            list containing all points of data type 6 in the entire file
        datasize_6 : int
            Size of the type 6 points data type (page 7 .lvx documentation)

        Returns
        -------
        totalbytesread : int
            Number of bytes read in the file including this package
        points_type_6 : list
            A python list (or you can choose another data type - np.ndarray would also work) of the single points in this
            data type 6 package - page 7 of documentation says that type 6 data only have one point per package
        datatype_points: list
            A list of all data type 6 points in the file

        '''
        # load the type 6 data block into buffer
        buf31 = fobj.read(datasize_6)
        # unpack it, little endian 6 floating point numbers
        value31 = struct.unpack('<6f', buf31)
        # debug print statements
        if showmessages:
            print("IMU")
        points_type_6 = list(value31)
        if showmessages:
            print(points_type_6)
        datatype_points.append(points_type_6)
        totalbytesread += datasize_6
        return [totalbytesread, points_type_6, datatype_points]

    @staticmethod
    def read_package_header(fobj, totalbytesread, packagedatasize=19, showmessages=False):
        '''
        A method for reading package headers

        Parameters
        ----------
        fobj : BinaryIO
            File object of the .lvx file being read
        totalbytesread : int
            number of bytes read in the file up to this package header
        packagedatasize : int
            Size of the package "header" in bytes

        Returns
        -------
        totalbytesread : int
            number of bytes read in the file including this package header
        data type: int
            the data type of points in the package

        '''
        # read in the block to the buffer
        buf7 = fobj.read(11)
        #reading in the timestamp - 8 bytes
        buf7b = fobj.read(8)
        # unpack the block as little endian with data types from .lvx documentation
        value7 = struct.unpack('<5BI2B', buf7)
        #decoding the timestamp specific for Timestamp Type 0 -- dtype differs for each timestamp type, see livox documentation
        print(np.frombuffer(buf7b, dtype=np.uint64))
        if showmessages:
            print("Package")
            print(list(value7))
        totalbytesread += packagedatasize
        # We know that value7[7] (per the Livox lvx documentation - see the table on page 8)
        # contains the data type that is going to be read
        datatype = value7[7]
        return [totalbytesread, datatype]

    @staticmethod
    def read_frame_header(fobj, frblocksize=24, showmessages=False):
        '''
        A method for reading frame headers of Livox .lvx binary files

        Parameters
        ---------
        fobj :  BinaryIO
            The file object corresponding to the binary .lvx file being read
        frblocksize : int
            The size of a frame header in bytes - it is set to a default value of 24, but can be modified if necessary

        Returns
        ------
        value4 : list
            A list of the elements of the frame header as described in the lvx file documentation

        '''

        # read in a frame header block
        buf4 = fobj.read(frblocksize)
        if not buf4:
            if showmessages:
                print("Done with frame reading")
            return []
        # unpack the values using the known data type - little endian, 3 unsigned long long types
        value4 = struct.unpack('<3q', buf4)
        # for debugging:
        if showmessages:
            print("Frame Header")
            print(list(value4))
        value4 = list(value4)
        # return the list of values from the frame header
        return value4

    @classmethod
    def lvxreader(cls, pathtofile, blockstoread=1, showmessages=False):
        '''
        A method which reads in Livox proprietary .lvx binary files and instantiates a BinaryReaders object.
        The structure of the binary file is detailed in the
        Livox technical document located here:
        https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Download/LVX%20Specifications.pdf

        Parameters
        ----------
        pathtofile : str
            String containing the path to the .lvx file
        blockstoread : int
            Number of blocks to read from the .lvx file.

        Returns
        ------------
        cls(datatype_points):
            list of lists (can be converted into numpy array for plotting) that contains the data from packages (unknown data type)
        cls(imudata):
            list of lists containing IMU data (data type 6)
        '''

        # The size of various blocks in the binary file - refer to the Livox documentation for the .lvx file format to see
        # where these numbers come from.
        # Sum of the size column, on page 2 of the .lvx documentation
        # 16 + 1 + 1 + 1 + 1 + 4 = 24
        hblocksize = 24
        # Sum of the size column, private header block table on page 3 of the .lvx documentation
        # 4 + 1 = 5
        pblocksize = 5
        # Sum of the size column, Devices Info Block table on page 3/4 of the .lvx documentation
        # 59 = 59
        diblocksize = 59
        # Frame header block size - Each frame header is accompanied by N packages
        # Bottom table on page 4 of .lvx documentaiton
        # 8 + 8 + 8 = 24
        frblocksize = 24
        # The beginning of each package begins with a 19 byte "header" so to speak - that is everything in the
        # size column on page 5 from the package table added up (neglecting the "depends" values):
        # 1 + 1 + 1 + 1 + 1 + 4 + 1 + 1 + 8 = 19
        packagedatasize = 19
        # Data type zero size (page 5)
        # 4 + 4 + 4 + 1 = 13
        datasize_0 = 13
        # Data type one size (page 6)
        # 4 + 2 + 2 + 1 = 9
        datasize_1 = 9
        # Data type two size (page 6)
        # 4 + 4 + 4 + 1 + 1 = 14
        datasize_2 = 14
        # Data type three size (page 6)
        # 4 + 2 + 2 + 1 + 1 = 10
        datasize_3 = 10
        # Data type four size (page 6)
        # 4 + 4 + 4 + 1 + 1 + 4 + 4 + 4 + 1 + 1 = 28
        datasize_4 = 28
        # Data type five size (page 7)
        # 2 + 2 + 4 + 1 + 1 + 4 + 1 + 1 = 16
        datasize_5 = 16
        # Data type six size (page 7)
        # 4 + 4 + 4 + 4 + 4 + 4 = 24
        datasize_6 = 24

        # data type zero (x, y, z, reflectivity)
        # data type 1 (depth, theta, phi, reflectivity)
        # data type 2 (x, y, z, reflectivity, tag)
        # data type 3 (depth, theta, phi, reflectivity, tag)
        # data type 4 (2 returns - x, y, z, reflectivity and tags)
        # data type 5 (2 returns - depth, theta, phi, reflectivity, tags)
        # data type 6 (IMU data - gyro x, y, z and accelerations x, y, z)

        datatype0_points = []
        datatype1_points = []
        datatype2_points = []
        datatype3_points = []
        datatype4_points = []
        datatype5_points = []
        datatype6_points = []

        # A counter to keep track of blocks
        counter = 0

        with open(pathtofile, "rb") as f:
            while counter < blockstoread:
                # We know that each Livox file has only one Public Header block
                buf = f.read(hblocksize)
                # We know that each Livox file has only one Private Header block
                buf2 = f.read(pblocksize)

                if not buf:
                    break

                hblock = struct.unpack('<16s4cI', buf)
                pblock = struct.unpack('<IB', buf2)

                if showmessages:
                    print("Public Header block")
                hblock = list(hblock)
                hblock[0] = hblock[0].decode('utf-8')
                hblock[1] = hblock[1].decode('utf-8')
                if showmessages:
                    print(hblock)

                if showmessages:
                    print("Private Header Block")
                pblock = list(pblock)
                # private header gives number of devices
                devicecount = pblock[1]
                if showmessages:
                    print(pblock)

                # place device info in a list
                dcounter = 0;
                deviceinfo = []
                while (dcounter < devicecount):
                    buf3 = f.read(diblocksize)
                    diblock = struct.unpack('<16s16s3B6f', buf3)
                    diblock = list(diblock)
                    diblock[0] = diblock[0].decode('utf-8')
                    diblock[1] = diblock[1].decode('utf-8')
                    deviceinfo.append(diblock)
                    dcounter += 1
                if showmessages:
                    print("Devices Info Block")
                    print(deviceinfo)

                counter += 1

                # counter to keep track of how many frames read
                fcounter = 0;
                while True:
                    # Call the static method read_frame_header defined above
                    fheaderdata = BinaryReaders.read_frame_header(f, frblocksize)
                    if not fheaderdata:
                        break
                    currentoffset = fheaderdata[0] + frblocksize
                    nextoffset = fheaderdata[1]
                    frameindex = fheaderdata[2]
                    # check if frame index matches up with frame count -- if not, stop frame reading
                    if (frameindex != fcounter):
                        break

                    fcounter += 1

                    # counter to keep track of how many packages read
                    pcounter = 0;
                    totalbytesread = 0;
                    totalbytestoread = nextoffset - currentoffset
                    if showmessages:
                        print(totalbytestoread)
                    while (totalbytesread < totalbytestoread):
                        # read all packages until next frame
                        pheader = BinaryReaders.read_package_header(f, totalbytesread, packagedatasize)
                        totalbytesread = pheader[0]
                        datatype = pheader[1]
                        if (datatype == 0):
                            package = BinaryReaders.data_type0_reader(f, totalbytesread, datatype0_points, datasize_0)
                            totalbytesread = package[0]
                            datatype0_points = package[2]
                        if (datatype == 1):
                            package = BinaryReaders.data_type1_reader(f, totalbytesread, datatype1_points, datasize_1)
                            totalbytesread = package[0]
                            datatype1_points = package[2]
                        if (datatype == 2):
                            package = BinaryReaders.data_type2_reader(f, totalbytesread, datatype2_points, datasize_2)
                            totalbytesread = package[0]
                            datatype2_points = package[2]
                        if (datatype == 3):
                            package = BinaryReaders.data_type3_reader(f, totalbytesread, datatype3_points, datasize_3)
                            totalbytesread = package[0]
                            datatype3_points = package[2]
                        if (datatype == 4):
                            package = BinaryReaders.data_type4_reader(f, totalbytesread, datatype4_points, datasize_4)
                            totalbytesread = package[0]
                            datatype4_points = package[2]
                        if (datatype == 5):
                            package = BinaryReaders.data_type5_reader(f, totalbytesread, datatype5_points, datasize_5)
                            totalbytesread = package[0]
                            datatype5_points = package[2]
                        if (datatype == 6):
                            package = BinaryReaders.data_type6_reader(f, totalbytesread, datatype6_points, datasize_6)
                            totalbytesread = package[0]
                            datatype6_points = package[2]
                        # print("Total Bytes Read")
                        # print(totalbytesread)
                        # print("Total bytes to read")
                        # print(totalbytestoread)
                        # print("Package index:", pcounter)
                        pcounter += 1
        if datatype != 6:
            if showmessages:
                print('Data type', datatype)
        if len(datatype0_points) != 0:
            return cls(datatype0_points, datatype6_points, 0)
        if len(datatype1_points) != 0:
            return cls(datatype1_points, datatype6_points, 1)
        if len(datatype2_points) != 0:
            return cls(datatype2_points, datatype6_points, 2)
        if len(datatype3_points) != 0:
            return cls(datatype3_points, datatype6_points, 3)
        if len(datatype4_points) != 0:
            return cls(datatype4_points, datatype6_points, 4)
        if len(datatype5_points) != 0:
            return cls(datatype5_points, datatype6_points, 5)

        # TO PLOT LIVOX DATA
        # in tester class
        # test2 = binreaders.lvxreader(livoxfilelocation, 1)
        # points = np.array(test.datapoints)
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        #change limits on axises depending on user
        # ax.set_zlim(-50000,30000)
        # plt.xlim(0, 100000)
        # #plt.ylim(-50000, 50000)
        # plt.xlabel('x')
        # plt.ylabel('y')
        # ax.scatter(points[:, 0], points[:, 2], -points[:, 1], s = 0.1)
        # plt.show()





