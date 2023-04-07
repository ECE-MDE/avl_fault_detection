from __future__ import annotations
import rospy
import copy
from avl_msgs.msg import VehicleStateMsg, NavigationMsg
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Vector3
from avl_fault_detection.util import Flag, MovingAvg, MultiMovingAvg

def main():

    rospy.init_node('kf_fault_detector', anonymous=True)

    # Fault status flags
    depth_sensor_zero_status = Flag()

    # Fault status publishers
    depth_sensor_zero = rospy.Publisher('fault_pred/kf/depth_sensor_zero', Float64, queue_size=100, latch=True)

    # Fault residual std publishers
    depth_sensor_res_std = rospy.Publisher('fault_pred/kf/std/depth_sensor_zero', Float64, queue_size=100, latch=True)

    gps_residual = MultiMovingAvg(10, 3)
    dvl_residual = MultiMovingAvg(10, 3)
    range_residual = MultiMovingAvg(10, 3)
    depth_residual = MovingAvg(10)
    state_cov = MultiMovingAvg(10, 15)

    def update_gps_residual(msg: Vector3):
        gps_residual.add((msg.x, msg.y, msg.z))

    def update_dvl_residual(msg: Vector3):
        dvl_residual.add((msg.x, msg.y, msg.z))

    def update_range_residual(msg: Vector3):
        range_residual.add((msg.x, msg.y, msg.z))

    def update_depth_residual(msg: Float64):
        depth_residual.add(msg.data)
        depth_sensor_res_std.publish(Float64(depth_residual.std()))
        if depth_residual.std() > 2:
            depth_sensor_zero_status.set(True)

        depth_sensor_zero.publish(Float64(1.0 if depth_sensor_zero_status.get() else 0.0))

    def update_state_cov(msg: Float64MultiArray):
        state_cov.add(tuple(msg.data[i] for i in range(15)))

    # State residual subscribers
    rospy.Subscriber('nav/gps_residual', Vector3, update_gps_residual)
    rospy.Subscriber('nav/depth_residual', Float64, update_depth_residual)
    rospy.Subscriber('nav/dvl_residual', Vector3, update_dvl_residual)
    rospy.Subscriber('nav/range_residual', Vector3, update_range_residual)
    rospy.Subscriber('nav/state_cov', Float64MultiArray, update_state_cov)

    rospy.spin()


if __name__ == '__main__':
    main()