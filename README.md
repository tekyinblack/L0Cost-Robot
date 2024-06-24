# L0Cost-Robot
This repository holds code for the L0Cost robot project which aims to create designs for robots which don't exceed $20. Inevitably this limit will get broken, but the limit will only increase with general inflation, not wild spending budgets!
The current target platform is the ESP32-CAM board as it provides a good mix of faclities without needing modification. The aim is not to make this platform totally autonomous, but to place control elsewhere such that, say, it might be controlled from a program on a PC or tablet, and as such be controlled in conjunction with other robots. Some basic video recognition code is included to facilitate basic line following and blob location using the camera.

It is also fully expected that the basic platform wil be augmented with a co-processor to handle more extensive I/O operations

The ESP32-CAM is a powerful microcontroller, with a fitted camera, Wi-Fi, Bluetooth and an SD card reader and while worldwide prices will vary, it does represent very good value for money. 
This module can be used just as a remote video camera with SD card configuration, upgraded to have a pan and tilt mount, then added to a small robot, examples of which will be loaded later. 
One target audience is educators who want an easily configurable and repeatable platform at a low cost to aid in STEM robot deployment.

Example configurations are
Pan & Tilt platform controller from web page
Basic RC robot using TT motors controlled from webpage with video feed
Extended robot with control from PS3, web page or self guided

Further configurations extend the capabilities with a Raspberry Pi Pico 

The file SDcard files.docx details the files on the SD card which are used to configure the robot without having to reload the code to the controller
The file  Command Processing.docx details the current state of the command set used to automate the robot controller.

A structure has not yet been designed to support command input instigation of self guidance features so these may appear in the code as switches but are not yet finalised.
