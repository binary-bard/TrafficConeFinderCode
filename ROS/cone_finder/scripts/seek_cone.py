#!/usr/bin/env python
#
# Node using cone_finder/location messages and
# Publishes /mavros_msgs/OverrideRCIn messages
#
import sys, argparse, rospy, math
from mavros_msgs.msg import OverrideRCIn
from cone_finder.msg import location_msgs as Locations
from cone_code import ConeSeeker

msg = """
cone_seeker: reading cone_finder location messages and 
             publishing OverrideRCIn messages
"""

#settings = termios.tcgetattr(sys.stdin)
hard_limits = [1000, 1500, 2000]  # microseconds for servo signal
steering_limits = [1135, 1435, 1735]  # middle is neutral
# throttle_limits = [1200, 1500, 1800]  # middle is neutral
throttle_limits = [1650, 1650, 1800]  # fwd range only; for testing; middle is NOT neutral
 
# We will get angle between +pi/2 to -pi/2 for steering
# We will get 480 pixels range for throttle but should limit this
class Args(object):
    # Typically less than 1 unless the range isn't responsive
    throttle_factor = 1.0
    steering_factor = 1.0

args = Args()
      
def drive_to_cone(loc):
    cs = ConeSeeker(0.3)
    (cone_loc, confidence, sd, td) = cs.seek_cone(loc.poses)
    rospy.loginfo('Confidence (%d, %d) = %f' % (cone_loc.x, cone_loc.y, confidence))

    steering = steering_limits[1] + (steering_limits[2] - steering_limits[1]) * sd
    # Slowest approach to cone
    throttle = throttle_limits[1] + (throttle_limits[2] - throttle_limits[1]) * td

    # Everything must be bounded    
    if(steering > steering_limits[2]):
        steering = steering_limits[2]
    if(steering < steering_limits[0]):
        steering = steering_limits[0]
    if(throttle > throttle_limits[2]):
        throttle = throttle_limits[2]
    if(throttle < throttle_limits[0]):
        throttle = throttle_limits[0]

    rc = OverrideRCIn()
    rc.channels = [int(steering), 0, int(throttle), 0, 0, 0, 0, 0]
    pub.publish(rc)

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Drive to cone found')
    parser.add_argument('--throttle_factor', '-t', default=1.0, type=float, 
                         help='Throttle step size factor')    
    parser.add_argument('--steering_factor', '-s', default=1.0, type=float,
                         help='Steering step size factor')
    parser.parse_args(rospy.myargv(sys.argv[1:]), args)
    try:
        pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.init_node('cone_seeker', anonymous=True)
        rospy.Subscriber('/cone_finder/locations', Locations, drive_to_cone)
        rospy.loginfo('cone_seeker init.')
        rc = OverrideRCIn()
        rc.channels = [1435,0,1500,0,0,0,0,0]
        pub.publish(rc)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
