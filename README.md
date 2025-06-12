# DVL A50 <> ArduSub - ROS1 Package

A ROS package to forward data from the Water Linked DVL A50 to ArduSub for EKF fusion.

To set the initial location of the ROV:
```bash
rostopic pub /mavros/global_position/set_gp_origin geographic_msgs/GeoPointStamped "header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
position:
  latitude: 43.7038
  longitude: -72.2945
  altitude: 35.0" --once
```
