Assuming you have your ROS environment set, instructions to build ROS node
   cd ROS
   mkdir cone_finder/src
   cp -r cone_finder $your_ROS_Workspace/src
   cd $your_ROS_Workspace
   catkin_make
   
To run ROS node
   source $your_ROS_Workspace/devel/setup.bash
   rosrun cone_finder detect_cones.py [-d]
   # -d/--debug is optional and will show real time video with cones marked
   Topic published: /locations
   See cone_finder/msg/locations_msg.msg for details


   
    
	
