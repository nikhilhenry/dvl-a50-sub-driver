#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from mavros_dvl_extras.msg import PositionDelta


class VisionPositionDeltaPublisher:
    def __init__(self):
        rospy.init_node("vision_position_delta_publisher")

        self.pub = rospy.Publisher(
            "/mavros/vision_position/delta", PositionDelta, queue_size=0
        )

        self.rate = rospy.Rate(30)

    def create_delta_message(self):
        msg = PositionDelta()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        msg.position_delta = Vector3(0, 0, 0)  # x, y, z (m)
        msg.angle_delta = Vector3(0, 0, 0.1)  # roll, pitch, yaw (rad)

        msg.confidence = 100  # @todo: compute from fom

        return msg

    def run(self):
        while not rospy.is_shutdown():
            msg = self.create_delta_message()
            self.pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        publisher = VisionPositionDeltaPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
