Assuming you have your ROS environment set, instructions to build ROS node
* `cd ROS`
* `mkdir cone_finder/src`
* `cp -r cone_finder $your_ROS_Workspace/src`
* `cd $your_ROS_Workspace`
* `catkin_make`
* `catkin_make install`
   
To run ROS node
* `source $your_ROS_Workspace/install/setup.bash`
* `rosrun cone_finder detect_cones.py [-d] [r]`
* # -d/--debug is optional and will show real time video with cones marked
* # -r/--use_ros_topic to use published topics from realsense_camera node
* Topic published:
    * /cone_finder/locations
    * /cone_finder/rgbImage
    * /cone_finder/depthImage
* See cone_finder/msg/locations_msg.msg for details

