# sawSartoriusScale

This SAW component contains code for interfacing with Sartorius scales
(www.sartorius.com).  It has been tested on a Sartorius GC 2502 with a
serial connection (USB).  It compiles on Windows, Linux and likely
MacOS.  This repository provides a core component as well as:
* Example application with Qt based GUI
* ROS node (also with Qt based GUI)

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS and ROS CRTK (optional) - works with ROS 1 and ROS 2!

# Compilation and configuration

## Linux permissions

Sartorius scales use a serial port to communicate.  When connecting your scale to your computer, a pseudo device will be added to the `/dev` directory.   Usually something like `/dev/ttyS01`, `/dev/ttyUSB0` or `/dev/ttyACM0`.  Using the command `dmesg` can help identify which device is used.  Check the file permissions on said device, e.g.,
```sh
ls -al /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 0 Jan  3 09:32 /dev/ttyUSB0
```

On Ubuntu, the OS usually sets the ownership of `/dev/ttyUSB0` to
`root` and the group to `dialout`.  To grant permissions to read and
write to the device, use the command `sudo adduser <user_id> dialout`
to add users to the `dialout` group.  Please note that the user has to
logout/login for the new group membership to take effect.

## Build

You can find some documentation re. compiling cisst and SAW components
for in the dVRK wiki:
* [ROS 1](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild)
* [ROS 2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2)

For Linux with ROS, we provide a VCS files to retrieve all the git
repositories you need for sawSartoriusScale in the `vcs` directory.

## Configuring the scale

For the scale and the PC to communicate properly, they need to have the same settings.  On the PC side, the defaults are hard coded in the `SetSerialPortDefaults` method:
```c++
    this->SerialPort.SetBaudRate(osaSerialPort::BaudRate9600);
    this->SerialPort.SetCharacterSize(osaSerialPort::CharacterSize7);
    this->SerialPort.SetParityChecking(osaSerialPort::ParityCheckingOdd);
    this->SerialPort.SetStopBits(osaSerialPort::StopBitsOne);
    this->SerialPort.SetFlowControl(osaSerialPort::FlowControlHardware);
```

On the scale side, you need to make sure it is configured to use the same settings (see manual in the `manual` directory).  The serial port settings are on page 30.

# Running the examples

## Main example

The main example provided is `sawSartoriusScaleQtExample`.  The command line options are:
```sh
sawSartoriusScaleQtExample:
 -s <value>, --serial-port <value> : serial port (e.g. /dev/ttyUSB0, COM...) (optional)
 -l, --log-serial : log all serial port read/writes in cisstLog.txt (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
 -m, --component-manager : JSON file to configure component manager (optional)
```

To run the example for a given serial port
```sh
sawSartoriusScaleQtExample -s /dev/ttyUSB0
```

## ROS

If you also want to use the ROS node for ROS 1, run:
```sh
rosrun sartorius_scale sartorius_scale
```

For ROS 2, run:
```sh
ros2 run sartorius_scale sartorius_scale
```

## Other "middleware"

Besides ROS, the ForceDimension component can also stream data to your application using the *sawOpenIGTLink* or *sawSocketStreamer* components.  See:
* [sawOpenIGTLink](https://github.com/jhu-saw/sawOpenIGTLink)
* [sawSocketStreamer](https://github.com/jhu-saw/sawSocketStreamer)
