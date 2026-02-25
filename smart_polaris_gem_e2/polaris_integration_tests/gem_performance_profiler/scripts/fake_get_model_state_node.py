#!/usr/bin/env python3
"""
Fake /gazebo/get_model_state service for profiling and integration tests without Gazebo.

Gazebo often runs into segmentation faults, especially on resource contrained systems.
Best not to rely on it for large-scale scenario testing where we need to reliably generate
a significant amount of data.

Advertises the same /gazebo/get_model_state service as Gazebo
"""
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from gazebo_msgs.srv import GetModelState, GetModelStateResponse


def handle_get_model_state(req):
    resp = GetModelStateResponse()
    resp.header.stamp = rospy.Time.now()
    resp.header.frame_id = "world"
    resp.pose = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    resp.twist = Twist(
        linear=Vector3(x=0.0, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=0.0),
    )
    resp.success = True
    resp.status_message = "fake"
    return resp


def main():
    rospy.init_node("fake_get_model_state_node", anonymous=False)
    srv = rospy.Service("/gazebo/get_model_state", GetModelState, handle_get_model_state)
    rospy.loginfo("Advertising fake /gazebo/get_model_state")
    rospy.spin()


if __name__ == "__main__":
    main()
