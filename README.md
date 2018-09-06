# walkerbotweb_pkg
Files for WalkerBot web control 

## Set up your Base Station

In order to view the video stream from the WalkerbotWeb's camera, you need to have a computer to act as your base station. 

Base station requirements:
  1. Any operating system
  2. Must have SSH (https://en.wikipedia.org/wiki/Secure_Shell) installed 
  3. Chrome (this is the browser used in testing. Any browser should theoretically suffice)

## User Interface
  1. index.html webpage
  
## Hardware
  1. Raspberry Pi Model B V1.2 
  2. 16GB SanDisk SD card
  3. Raspberri Pi Camera V2
  4. 3.7V 1200 mAh battery cell 
  5. PowerBoost 1000 charger
  6. SmartBeam projector
  7. HDMI to HDMI mini cable to connect the Pi and the SmartBeam

## Wiring
  Raspberry Pi ---> Raspberry Pi Camera ribbon cable 
  Raspberry Pi HDMI port ---> HDMI to HDMI mini cable ---> "BEAM" port of SmartBeam projector
  Raspberry Pi USB mini port ---> USB mini to PowerBoost 1000 Charger ---> 3.7V 1200mAh battery cell

## Software
  1. Lubuntu image (2018-06-27-ubiquity-xenial-lxde-raspberry-pi.im) from UbiquityRobotics with ROS KineticKame preinstalled.      Once flashed onto an SD card and inserted into the Pi, the Pi is automatically on AP (access point mode). However, you        should enable WiFi and put it on the same WiFi network as your base station so that you can SSH into the Pi from your        base station. 
  2. Google Chrome console for troubleshooting
  
## How to Use
  1. Flash the pi image onto the SD card.
  2. Connect all the hardware. Feel free to use a keyboard and mouse with the Pi to ensure that it is on the same WiFi network as your base station 
  3. Open up a terminal on your base station and ssh into the pi by doing `ssh pi@***`
  4. Go to the src folder of your catkin workspace `~/catkin_ws/src` and clone the github repository by typing `git clone https://github.com/njoseph1/walkerbotweb_pkg`
  5. Navigate to you catkin director again by typing ~/catkin_ws 
  6. Now run `catkin_make --pkg walkerbotweb_pkg` to create messages.
  7. if you've change you robot's hostname, edit `~/catkin_ws/src/walkerbotweb_pkg/setup.sh` so that ROS_HOSTNAME is your chosen hostname.
  8. Now we need to start screen. Do so by going to `~/catkin_ws/src/walkerbotweb_pkg` and running `screen -c walkerbotweb.screenrc`. This is a custom congifuation file with four screens.
    ~0 is running roscore
    ~1 is running rosbrdige
    ~2 is running web_video_server
    ~3 is running raspicam_node
    ~4 is running camera_service
    
   Make sure these programs started properly by cycling through the screen terminals. If roscore failed to start (usually due    to another instance of roscore/rosmaster aready running), type killall -9 roscore to kill old roscore processes. You then    need to start the processes in each screen over again. You can see what the commands are in the screen windows by            navigating to the windows and pressing the up key, or by checking out the walkerbotweb.screenrc file in the repository.
    
  9. Open up the index.html file and click connect. Please change your host name in the dialog box if you edited it from the original name. 
  10. Click connect. The status should change to a green-colored "Connected!" message and a video should start streaming in the camera area. If the green "Connected!" eventually changes to the red "Connection Closed" message, do not worry. This is a bug that does not affect anything. Your video should still be streaming.
  
## To Do
  1. Change keyboard input in the index file to button input.
  2. The index.html file has the base code for publishing twist commands to WalkerBot via keyboard input. Figure out the correct syntax for assigning the twistMessage linear x and twistMessage angular z (for instance, twistMsg.angular.z = turnSpeed is logically correct sudo code but needs to be properly formatted to be integrated with the exisiting WalkerBot infastructure).
  3. Integrate the WalkerBot web system with the existing walkerbot infrastructure and test if the Twist messages make WalkerBot move as expected! 
 
