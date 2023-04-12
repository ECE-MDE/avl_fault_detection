from __future__ import annotations
import rospy
import copy
from avl_msgs.msg import VehicleStateMsg, NavigationMsg, Float64SetpointMsg
from avl_msgs.srv import VehicleDynamicsSrv
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Vector3
from avl_fault_detection.util import Flag, MovingAvg, MultiMovingAvg

def main():

    rospy.init_node('kf_fault_detector', anonymous=True)

    # Fault status flags
    depth_sensor_zero_status = Flag()
    rpm_sensor_zero_state = Flag()
    rpm_zero_state = Flag()
    hull_drag_state = Flag()

    # Fault status publishers
    depth_sensor_zero = rospy.Publisher('fault_pred/kf/depth_sensor_zero', Float64, queue_size=100, latch=True)
    rpm_sensor_zero = rospy.Publisher('fault_pred/kf/rpm_zero', Float64, queue_size=100, latch=True)
    rpm_zero = rospy.Publisher('fault_pred/kf/rpm_zero', Float64, queue_size=100, latch=True)
    hull_drag = rospy.Publisher('fault_pred/kf/hull_drag', Float64, queue_size=100, latch=True)

    # Fault residual std publishers
    depth_sensor_res_std = rospy.Publisher('fault_pred/kf/std/depth_sensor_zero', Float64, queue_size=100, latch=True)
    rpm_sensor_error_std = rospy.Publisher('fault_pred/kf/std/rpm_zero', Float64, queue_size=100, latch=True)
    hull_drag_std = rospy.Publisher('fault_pred/kf/std/hull_drag', Float64, queue_size=100, latch=True)
    hull_drag_avg = rospy.Publisher('fault_pred/kf/avg/hull_drag', Float64, queue_size=100, latch=True)

    dynamics = rospy.ServiceProxy('sim/dynamics', VehicleDynamicsSrv)

    gps_residual = MultiMovingAvg(10, 3)
    dvl_residual = MultiMovingAvg(10, 3)
    range_residual = MultiMovingAvg(10, 3)
    depth_residual = MovingAvg(10)
    accel = MovingAvg(10)
    rpm = MovingAvg(10)
    vel = MovingAvg(30)
    state_cov = MultiMovingAvg(10, 15)
    latest_state = NavigationMsg()

    rpm_setpoint_update_time = 0
    rpm_setpoint = 0
    rpm_sensor_error = MovingAvg(20)

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

    def update_rpm_setpoint(msg: Float64SetpointMsg):
        nonlocal rpm_setpoint
        if msg.data != rpm_setpoint:
            rpm_sensor_error.reset()
            # vel.reset()
            # accel.reset()
            rpm_setpoint = msg.data

    def update_rpm_reading(msg: Float64):
        nonlocal rpm_setpoint
        rpm_sensor_error.add(rpm_setpoint - msg.data)
        if rpm_sensor_error.full():
            if rpm_sensor_error.std() > 800:
                rpm_sensor_zero_state.set(True)
            rpm_sensor_error_std.publish(rpm_sensor_error.std())

        rpm_sensor_zero.publish(Float64(1.0 if rpm_sensor_zero_state.get() else 0.0))


    def update_throttle(msg: Float64):
        nonlocal latest_state
        if latest_state and vel.num_samples() > 0:
            rpm.add(msg.data)
            last_vel = vel.avg()
            this_vel = vel.add(latest_state.vn)
            accel.add((this_vel - last_vel)*100)
            hull_drag_std.publish(accel.std())
            hull_drag_avg.publish(accel.get_latest())
            if vel.avg() > 0.1 and accel.avg()*100 < -0.05:
                hull_drag_state.set(True)
        else:
            vel.add(latest_state.vn)

        hull_drag.publish(Float64(1.0 if hull_drag_state.get() else 0.0))

    def update_inertial_nav(msg: NavigationMsg):
        nonlocal latest_state
        latest_state = msg
        # nonlocal latest_state
        # latest_state = msg
        # if vel.num_samples() > 5:
        #     old = vel.get_latest()
        #     new = vel.add(msg.vn)
        #     est_accel.add(new-old)
        #     hull_drag_std.publish(est_accel.avg() * 100)
        #     hull_drag_avg.publish(vel.avg())
        #     if vel.avg() > 0.1 and est_accel.avg()*100 < -0.05:
        #         hull_drag_state.set(True)
        # else:
        #     vel.add(msg.vn)

        # hull_drag.publish(Float64(1.0 if hull_drag_state.get() else 0.0))
        ...

    # State residual subscribers
    rospy.Subscriber('nav/gps_residual', Vector3, update_gps_residual)
    rospy.Subscriber('nav/depth_residual', Float64, update_depth_residual)
    rospy.Subscriber('nav/dvl_residual', Vector3, update_dvl_residual)
    rospy.Subscriber('nav/range_residual', Vector3, update_range_residual)
    rospy.Subscriber('nav/state_cov', Float64MultiArray, update_state_cov)
    rospy.Subscriber('nav/inertial_nav', NavigationMsg, update_inertial_nav)

    rospy.Subscriber('setpoint/rpm', Float64SetpointMsg, update_rpm_setpoint)
    rospy.Subscriber('device/rpm', Float64, update_rpm_reading)
    rospy.Subscriber('device/motor', Float64, update_throttle)

    rospy.spin()


if __name__ == '__main__':
    main()