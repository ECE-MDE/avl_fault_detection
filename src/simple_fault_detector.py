from __future__ import annotations
import rospy
import copy
from avl_msgs.msg import VehicleStateMsg, NavigationMsg
from std_msgs.msg import Float64
from avl_fault_detection.util import Flag, Timeseries

def main():
    state_initialized = Flag(False)

    rospy.init_node('simple_fault_detector', anonymous=True)

    rospy.Subscriber('nav/inertial_nav', NavigationMsg, lambda _: state_initialized.set(True))    
    depth_sensor_zero = rospy.Publisher('fault_gen/pred/depth_sensor_zero', Float64, queue_size=100)

    latest_state: NavigationMsg | None = None
    state_on_depth_meas: NavigationMsg | None = None
    meas_depth: float | None = None
    meas_depth_ts: float | None = None
    pred_depth: float | None = None

    # Inertial nav estimates update fast
    def update_meas_state(msg: NavigationMsg):
        nonlocal latest_state
        latest_state = msg

    # Depth updates very slowly, so do our prediction here
    def update_meas_depth(msg: Float64):
        nonlocal meas_depth
        nonlocal state_on_depth_meas
        nonlocal meas_depth_ts
        if meas_depth is not None:
            dt = (rospy.Time.now() - meas_depth_ts).to_sec()
            pred_depth = meas_depth + state_on_depth_meas.vd * dt
            print(f"{pred_depth}\t{msg.data}\t{abs(pred_depth - msg.data)}")
            depth_sensor_zero.publish(Float64(abs(pred_depth - msg.data)))

        state_on_depth_meas = copy.deepcopy(latest_state)
        meas_depth = msg.data
        meas_depth_ts = rospy.Time.now()

    # Wait for inertial nav init
    while not state_initialized.get():
        rospy.sleep(0.1)

    rospy.Subscriber('nav/inertial_nav', NavigationMsg, update_meas_state)
    rospy.Subscriber('device/depth', Float64, update_meas_depth)

    rospy.spin()


if __name__ == '__main__':
    main()