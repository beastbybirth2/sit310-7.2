#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray

class AprilTagFollower:
    def __init__(self):
        # Initialize the ROS node with a unique name
        rospy.init_node('april_tag_follower_node', anonymous=True)

        # Setup shutdown handler to ensure the robot stops on exit
        rospy.on_shutdown(self.shutdown_handler)

        # Publishers and Subscribers for robot control and AprilTag detection
        self.velocity_publisher = rospy.Publisher('/duckieshop/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.tag_subscriber = rospy.Subscriber('/duckieshop/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_detected_callback, queue_size=1)

        rospy.spin()  # Keep the node alive

    def tag_detected_callback(self, msg):
        self.handle_tag_detection(msg.detections)

    def shutdown_handler(self):
        rospy.loginfo("Shutting down. Stopping the robot.")
        self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist2DStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.v = 0.0
        stop_msg.omega = 0.0
        self.velocity_publisher.publish(stop_msg)

    def handle_tag_detection(self, detections):
        if not detections:
            # Rotate the robot if no tag is detected
            self.send_velocity(0.0, 0.4)  # Rotate in place
            return

        # Stop rotation once a tag is detected
        self.stop_robot()

        # Assume the first detection for simplicity
        detection = detections[0]
        x_position = detection.transform.translation.x
        z_distance = detection.transform.translation.z
        rospy.loginfo("Tag detected at x: %f, z: %f", x_position, z_distance)

        if z_distance > 0.15:
            self.send_velocity(0.2, 0.0)  # Move forward
        elif z_distance < 0.10:
            self.send_velocity(-0.2, 0.0)  # Move backward
        else:
            self.stop_robot()

    def send_velocity(self, linear_velocity, angular_velocity):
        velocity_command = Twist2DStamped()
        velocity_command.header.stamp = rospy.Time.now()
        velocity_command.v = linear_velocity
        velocity_command.omega = angular_velocity
        self.velocity_publisher.publish(velocity_command)

if __name__ == '__main__':
    try:
        april_tag_follower = AprilTagFollower()
    except rospy.ROSInterruptException:
        pass
