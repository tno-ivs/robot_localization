#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from tf import transformations
from geometry_msgs.msg import TwistStamped, Quaternion
from math import atan2, hypot

from robot_localization.srv import SetDatum, SetDatumRequest

TWIST = None
GPS = None
VELOCITY_MAGNITUDE_THRESHOLD = 2.0


def twist_callback(msg):
    """
    Twist callback from GPS
    :param msg: The twist msg
    """
    global TWIST
    if hypot(msg.twist.linear.x, msg.twist.linear.y) > VELOCITY_MAGNITUDE_THRESHOLD:
        rospy.loginfo("Received valid Velocity")
        TWIST = msg


def gps_callback(msg):
    """
    NavsatFix callback from gps
    :param msg: The gps msg
    """
    global GPS
    rospy.loginfo("Received GPS")
    GPS = msg

if __name__ == '__main__':
    # Initialize ROS comm
    rospy.init_node("send_datum_on_gps_velocity_fix")
    VELOCITY_MAGNITUDE_THRESHOLD = rospy.get_param('~velocity_magnitude_threshold', 2.0)
    sub = rospy.Subscriber("gps/fix", NavSatFix, gps_callback, queue_size=1)
    sub_vel = rospy.Subscriber("gps/vel", TwistStamped, twist_callback, queue_size=1)

    srv_name = "datum"
    rospy.loginfo("Waiting for %s srv" % srv_name)
    rospy.wait_for_service(srv_name)

    rospy.loginfo("Connecting to %s srv" % srv_name)
    set_datum = rospy.ServiceProxy(srv_name, SetDatum)

    # Spin until we have our required inputs
    rospy.loginfo("Waiting for GPS and Velocity fix > %.2f.." % VELOCITY_MAGNITUDE_THRESHOLD)
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and (TWIST is None or GPS is None):
        r.sleep()

    # Convert
    q = transformations.quaternion_from_euler(0, 0, atan2(TWIST.twist.linear.y, TWIST.twist.linear.x))
    
    r = SetDatumRequest()
    r.geo_pose.position.latitude = GPS.latitude
    r.geo_pose.position.longitude = GPS.longitude
    r.geo_pose.position.altitude = GPS.altitude
    r.geo_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    
    rospy.loginfo("Calling setDatum")
    try:
        resp1 = set_datum(r)
        
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)

    rospy.loginfo("Successfully updated datum")
