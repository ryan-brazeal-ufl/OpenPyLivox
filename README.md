# OpenPyLivox
The ***unofficial*** Python3 driver for Livox lidar sensors ;)

The OpenPyLivox library is a near complete, fully pythonic, implementation of the Livox SDK. This means that almost all the functionality available within official Livox software (e.g., Livox-Viewer) and their C++ based API, has been included here within the OpenPyLivox library. *Ok, ok ... maybe not quite as cool or as functional as the Livox-Viewer, ... yet!*

This library and associated documentation, is designed for **ANYONE and EVERYONE** to be able to use it. From students and teachers interested in using lidar/laser scanning in STEM curriculum or projects, to researchers investigating lidar for autonomous vehicle/robot navigation!

***See the [Wiki](../../wiki) Pages for complete documentation!***

Check out the [livox_controller_demo.py](./livox_controller_demo.py) file for a very detailed example on how to use the OpenPyLivox library to control and capture point cloud data from a single Livox Mid-40 sensor, as well as from multiple Livox Mid-40 sensors simultaneously!

*NOTES:* 
- OpenPyLivox v1.0 has only been tested using Mid-40 sensors
- Simultaneous operation of multiple Mid-40 sensors has been tested, but NOT using a Livox Hub
- The library has been tested on Mac OS X, Linux (GalliumOS on HP Chromebook), and Windows 7 and 10
- The library has been tested to work with Livox Mid-40 firmwares:
  - 03.03.0001 to 03.03.0005 (all [special Livox firmwares](https://github.com/Livox-SDK/Special-Firmwares-for-Livox-LiDARs), including multiple returns)
  - 03.05.0000 to 03.06.0000 (standard versions)
- The CSV output file's header record, allows the point cloud data to be easily opened in the <b>amazing</b> open source software, CloudCompare (download at https://cloudcompare.org)

`//X,Y,Z,Inten-sity,Time,ReturnNum`      (ReturnNum is only included when using firmwares 03.03.0001 or .0002)

**Quirky Fact:** Intensity (a.k.a., Reflectivity in the Livox documentation) has a hyphen in the CSV header record in order to 'trick' CloudCompare into assigning the field as a scalar type by default. This enables displaying the point cloud in CloudCompare using a more visually appealing colour spectrum (e.g., left image below). It also provides a way to (possibly) help filter out some unwanted noisy data. Of course, the colour scheme can be changed to many other options, after importing the point cloud in CloudCompare (e.g., greyscale in right image below)

<table style="border:0px;">
  <tr style="border:0px;">
    <td style="border:0px;"><img src="./images/image1_rs.png"></td>
    <td style="border:0px;"><img src="./images/image2_rs.png"></td>
  </tr>
</table>

## Change Log:
- v1.0 released, **FRIDAY** Sept. **the 13th** 2019
- Wiki for v1.0 completed, Monday Sept. 23rd 2019
- Wiki v1.0 minor updates, Saturday Nov. 30th 2019
