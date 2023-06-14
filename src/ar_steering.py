#!/usr/bin/python3

import rospy
import tf
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarkers

last_heartbeat = 0

def normalise(in_min, in_max, out_min, out_max, data):
    return (out_max - out_min) / (in_max - in_min) * (data - in_min) + out_min

def fit_roll_to_linear(roll):
    roll_min = 3
    roll_max = 2
    linear_min = 0
    linear_max = 1

    normalised_roll = normalise(roll_min, roll_max, linear_min, linear_max, abs(roll))
    constrained_linear = max(linear_min, min(linear_max, normalised_roll))

    if roll > -2.9 and roll < 2.9:
        return constrained_linear * (roll / abs(roll)) * -1 # robot speed multiplied by 1 if robot should go forward, -1 if backward
    else: #probably hand shaking or something like that, not clear whether robot should drive
        return 0

def fit_pitch_to_angular(pitch):
    pitch_min = -1.0
    pitch_max = 1.0
    angular_min = -3
    angular_max = 3

    normalised_pitch = normalise(pitch_min, pitch_max, angular_min, angular_max, pitch)

    if pitch < -0.1 or pitch > 0.1: #avoid tiny flickerings having an effect on turning
        return normalised_pitch
    else:
        return 0

def callback(data):
    global last_heartbeat, marker_id
    
    #rospy.loginfo("I got a new datapoint: "+str(data.markers))
    
    if len(data.markers) > 0:
        for marker in data.markers:
            # Check the marker id
            if marker.id != marker_id:
                continue

            marker_ori = (
                marker.pose.pose.orientation.x,
                marker.pose.pose.orientation.y,
                marker.pose.pose.orientation.z,
                marker.pose.pose.orientation.w)

            #print("marker_ori:", marker_ori)
            euler = tf.transformations.euler_from_quaternion(marker_ori)
            #linear_speed = round(fit_roll_to_linear(euler[0]), 2)
            linear_speed = fit_roll_to_linear(euler[0])
            angular_speed = round(fit_pitch_to_angular(euler[2]), 2)

            #print("roll: ", round(euler[0], 5), "pitch: ", round(euler[2], 5), "robot speed: ", linear_speed, "turning speed: ", angular_speed)

            # Create a Twist message from the normalised speeds
            twist_msg = Twist()
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed

            #print("Updating cmd vel %s", twist_msg)
            global cmd_vel_pub
            last_heartbeat = rospy.get_time()
            cmd_vel_pub.publish(twist_msg)


# Publish zero cmd_vel when no AR info has been received within given period
def timer_callback(event):
    global last_heartbeat
    if (rospy.get_time() - last_heartbeat) >= 0.5:
        cmd_vel_pub.publish(Twist())


def ar_demo():
    global marker_id
    # Initialize this ROS node
    rospy.init_node('ar_steering')
    # get target marker id
    marker_id = rospy.get_param("marker_id")

    #initialize heartbeat
    global last_heartbeat
    last_heartbeat = rospy.get_time()

    # Create publisher for command velocity
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    #Register heartbeat timer
    t = rospy.Timer(rospy.Duration(0.1), timer_callback)

    # Set up subscriber for /ar_pose_marker
    rospy.loginfo("Subscribing to /ar_pose_marker")
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

    rospy.spin()


if __name__ == '__main__':
    ar_demo()
