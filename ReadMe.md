Written by Leon RyuWoon Jung

This package refers to the [MIT Duckie Town source](https://github.com/duckietown/Software)

You can find where the progress stopped by searching 'progress 3 stopped here' statement.

# Install
The following are needed to be installed or downloaded

* turtlebot3_RBIZ package

  ```bash
  $ ~/catkin_ws/src && git clone https://github.com/hyunoklee/turtlebot3_RBIZ.git
  ```
* install the others

  ```bash
  $ sudo apt-get install libv4l-dev
  ```

  ```bash
  $ sudo apt-get install ros-kinetic-tf-conversions ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-camera-info-manager ros-kinetic-theora-image-transport ros-kinetic-joy ros-kinetic-image-proc -y
  ```

  ```bash
  $ sudo apt-get install ros-kinetic-compressed-image-transport -y
  ```

  ```bash
  $ sudo apt-get install libyaml-cpp-dev -y
  ```

  ```bash
  $ sudo apt-get install libblas-dev liblapack-dev libatlas-base-dev gfortran
  ```

# Run
These packages can be launched by following commands

* run roscore node ( remote pc side )

  ```bash
  $ roscore
  ```

* run camera node ( turtle_bot side )

  ```bash
  $ roslaunch turtlebot3_auto_sensor image_get_usb_camera.launch
  ```

* run color balancing node ( remote pc side ) (if necessary... you don't need to run this in some cases)

  ```bash
  $ roslaunch turtlebot3_auto_sensor image_proc_color_balancing.launch
  ```

* send Bool 'True' message for activating color balancing node  ( remote pc side )  (if necessary... you don't need to run this in some cases)

  ```bash
  $ rostopic pub /click std_msgs/Bool True
  ```

* run line detection node  ( remote pc side ) 

  ```bash
  $ roslaunch line_detect detect_line.launch
  ```

  ```bash
  $ Check the operation up to here 
  ```

* run line filter node  ( remote pc side ) 

  ```bash
  $ roslaunch lane_filter lane_filter_node.launch
  ```

* run motor control node  ( remote pc side ) 

  ```bash
  $ roslaunch motor_control motor_controller_node.launch


# Tool

* view image

  ```bash
  $ rviz
  $ add  ->  By topic -> 
  ```

* view topic

  ```bash
  $ rqt
  $ 
  ```

# Camera Calibration

* Calibration install package

  ```bash
  $ rosdep install camera_calibration
  ```

* Calibration package run
  ```bash
  $ roslaunch turtlebot3_auto_sensor image_get_usb_camera.launch
  ```
  ```bash
  $ rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.031 image:=/image_raw camera:=/
  ```

  ```bash
  $ use calibration pattern calibration pattern.pdf 
  and save calibration data  -> turtlebot3_auto/line_detect/src/callibra_ros.yaml
  ```
  

