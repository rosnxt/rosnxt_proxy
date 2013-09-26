rosnxt
======

rosnxt is a collection of software packages for using the NXT brick and sensor in ROS.

From [ROS.org](http://www.ros.org/wiki/): ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. 

rosnxt_proxy
============

rosnxt_proxy implements the communication between the computer and the NXT brick, and it exposes the functionality of the NXT via ROS topics, services and params.

On startup, rosnxt_proxy configures the ports of the NXT, load the required drivers, and processes the data.

Configuration is done via ROS params:

* brick_name: specifies the name of the NXT brick to connect to;
* port/<PORT>/type: specifies the kind of device and driver to attach to the specified sensor/motor/misc port;
* port/<PORt>/slot/<SLOT>/pollPeriod: configure the specified port to send some predefined packet of data (slot) to the rosnxt_proxy every pollPeriod milliseconds;

<PORT> may be one of s1, s2, s3, s4, a, b, c, misc. (misc is used for information not not coming from a device attached to sensor or motor ports).

<SLOT> is a number dependent on the type of device, starting from 0.

