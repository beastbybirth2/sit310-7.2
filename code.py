#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray

class TargetFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Set up the shutdown hook
        rospy.on_shutdown(self.clean_shutdown)

        # Publisher to send movement commands
        self.cmd_vel_pub = rospy.Publisher('/duckieshop/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        # Subscriber to receive AprilTag detections
        self.tag_sub = rospy.Subscriber('/duckieshop/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin()  # Keep the node active

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    # Clean shutdown handler
    def clean_shutdown(self):
        rospy.loginfo("Shutting down... Stopping the robot.")
        self.stop_robot()

    # Function to stop the robot
    def stop_robot(self):
        stop_msg = Twist2DStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.v = 0.0
        stop_msg.omega = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    def move_robot(self, detections):
        if not detections:
            self.stop_robot()
            return

        self.stop_robot()  # Ensure the robot stops before processing new detections

        # Use the first detection for movement decision
        x = detections[0].transform.translation.x
        y = detections[0].transform.translation.y
        z = detections[0].transform.translation.z
        rospy.loginfo("Detected tag at x: %f, y: %f, z: %f", x, y, z)
        rospy.sleep(0.5)

        # Decide movement based on tag's position
        if z > 0.2:
            self.send_velocity(0.1, 0.0)  # Move forward slowly
            rospy.sleep(0.3)
            self.stop_robot()
        elif z < 0.1:
            self.send_velocity(-0.1, 0.0)  # Move backward slowly
            rospy.sleep(0.3)
            self.stop_robot()
        elif x > 0.1:
            self.send_velocity(0.0, -0.3)  # Turn right gently
            rospy.sleep(0.3)
            self.stop_robot()
        elif x < -0.1:
            self.send_velocity(0.0, 0.3)  # Turn left gently
            rospy.sleep(0.3)
            self.stop_robot()

    # Helper function to publish movement commands
    def send_velocity(self, linear_velocity, angular_velocity):
        velocity_cmd = Twist2DStamped()
        velocity_cmd.header.stamp = rospy.Time.now()
        velocity_cmd.v = linear_velocity
        velocity_cmd.omega = angular_velocity
        self.cmd_vel_pub.publish(velocity_cmd)

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass
