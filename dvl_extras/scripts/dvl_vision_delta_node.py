#!/usr/bin/env python

import rospy
from waterlinked_a50_ros_driver.msg import DVL
from mavros_dvl_extras.msg import PositionDelta
from std_msgs.msg import Header

class DVLToVisionPose:
    def __init__(self):
        rospy.init_node('dvl_vision_pose_node', anonymous=True)
        self.pub = rospy.Publisher('/mavros/vision_position/delta', PositionDelta, queue_size=1)
        self.sub = rospy.Subscriber('/dvl/data',DVL, self.dvl_callback)

        self.seq = 0
        rospy.loginfo("DVL to vision_pose publisher initialized.")

    def dvl_callback(self, msg:DVL):
        print(msg)
        try:
            if not msg.velocity_valid:
                rospy.loginfo_throttle(1.0, "Ignoring invalid velocity readings from DVL")
                return

            velocity = msg.velocity
            fom = msg.fom
            dt = msg.time / 1000 # convert from milliseconds to seconds

            dx = velocity.x * dt
            dy = velocity.y * dt
            dz = velocity.z * dt

            """
            convert fom (Figure of Merit) to confidence
            see: https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/blob/master/dvl-a50/dvl.py#L376-L377
            """

            _fom_max = 0.4
            confidence = 100 * (1 - min(_fom_max, fom) / _fom_max)

            delta_msg = PositionDelta()
            delta_msg.header = Header()
            delta_msg.header.stamp = rospy.Time.now()
            delta_msg.header.frame_id = "map"
            delta_msg.header.seq = self.seq
            self.seq += 1

            delta_msg.position_delta.x = dx
            delta_msg.position_delta.y = dy
            delta_msg.position_delta.z = dz

            """
            angular data is not consumed by ardusub 
            see https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/pull/42#issuecomment-2472684300
            """

            delta_msg.angle_delta.x = 0 # roll
            delta_msg.angle_delta.y = 0 # pitch
            delta_msg.angle_delta.z = 0 # yaw

            delta_msg.confidence = confidence

            self.pub.publish(delta_msg)
            rospy.loginfo_throttle(1.0, f"Published delta: dx={dx}, y={dy}, z={dz}")

        except Exception as e:
            rospy.logwarn(f"Failed to parse DVL JSON data: {e}")

if __name__ == '__main__':
    try:
        DVLToVisionPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
