import math
from typing import Dict
import rospy
from avl_fault_detection import logger
from avl_fault_detection.util import Flag
from avl_msgs.msg import VehicleStateMsg, Float64SetpointMsg, ActuatorControlMsg
from std_msgs.msg import Bool
"""
surge velocity
heave velocity
depth rate

roll angle
pitch angle
yaw angle

roll rate
pitch rate
yaw rate
"""

class Fault:
    def __init__(self, topic_name: str, publish_rate_hz: int = 50):
        self._topic_name = topic_name
        self._enabled = False
        self._publish_rate_hz = publish_rate_hz
        self._publisher = rospy.Publisher(topic_name, Bool, queue_size=100)

    def _publish(self, event):
        self._publisher.publish(Bool(self._enabled))

    def start_publisher(self):
        rospy.Timer(rospy.Duration(1.0 / self._publish_rate_hz), lambda e: self._publish(e))

    def enable(self):
        self._enabled = True
    
    def disable(self):
        self._enabled = False

    def get_topic_name(self) -> str:
        return self._topic_name

    def is_enabled(self) -> bool:
        return self._enabled

def main():
    # Create a node
    rospy.init_node('data_collector', anonymous=True)

    state_initialized = Flag()
    start_time = rospy.Time.from_sec(0)
    logging_rate_hz = rospy.Rate(2)

    fault_trigger_time = rospy.Duration(rospy.get_param("/fault_trigger_time"))

    fault_depth_sensor_zero = Fault("fault_gen/depth_sensor_zero")

    log = logger.AvlLogger('data_collector')
    log.write_msg_header([VehicleStateMsg], [fault_depth_sensor_zero.get_topic_name()], units=["bool"])


    # Create a subscriber for the sim state topic and print the sim state to the terminal every 1 second
    def log_sim_state(msg: VehicleStateMsg):
        log.write_msg_data([msg], [fault_depth_sensor_zero.is_enabled()])
        log.flush() # for debugging purposes
        print(f"{fault_trigger_time}\n{(rospy.Time.now() - start_time).secs}\n{rospy.Time.now().secs}\n{msg}\r")
        # print(f"{15} {math.radians(15)} {math.degrees(msg.pitch)} {msg.pitch}\r")
        logging_rate_hz.sleep()

    # Create RPM and pitch setpoint publishes to get the AUV moving
    pub_rpm_setpt = rospy.Publisher("setpoint/rpm", Float64SetpointMsg, queue_size=100)
    pub_yaw_setpt = rospy.Publisher("setpoint/yaw", Float64SetpointMsg, queue_size=100)
    pub_pit_setpt = rospy.Publisher("setpoint/pitch", Float64SetpointMsg, queue_size=100)
    pub_rol_setpt = rospy.Publisher("setpoint/roll", Float64SetpointMsg, queue_size=100)
    pub_act_ctl = rospy.Publisher("device/actuator_control", ActuatorControlMsg, queue_size=100)

    def publish_setpoints(event):
        pub_rpm_setpt.publish(Float64SetpointMsg(True, 2000)) # 2000 is max rpm for 690
        pub_yaw_setpt.publish(Float64SetpointMsg(True, math.radians(0)))
        pub_pit_setpt.publish(Float64SetpointMsg(True, math.radians(15)))
        pub_rol_setpt.publish(Float64SetpointMsg(True, math.radians(0)))

    def publish_actuator_enable(event):
        pub_act_ctl.publish(ActuatorControlMsg(True))

    def set_state_initialized(msg: VehicleStateMsg):
        state_initialized.set(True)

    # listen for first vehicle state message
    rospy.Subscriber("/sim/state", VehicleStateMsg, set_state_initialized)

    # Wait until set_state_initialized is called
    while not state_initialized.get():
        rospy.sleep(0.1)

    # Wait for a bit before we start publishing
    rospy.sleep(10) 
    print("\n\nBeginning to publish setpoints!\n\n")
    # Record start time so we can log when fault starts
    now = rospy.Time.now()
    start_time.set(now.secs, now.nsecs)

    # Print sim state to stdout
    rospy.Subscriber("/sim/state", VehicleStateMsg, log_sim_state)

    # Start publishing fault status
    fault_depth_sensor_zero.start_publisher()

    # Set up timers to repeatedly call the publishing methods
    rospy.Timer(rospy.Duration(1.0 / 10.0), publish_setpoints)
    rospy.Timer(rospy.Duration(1.0 / 0.5), publish_actuator_enable)
    # Set up timer to start fault generation
    rospy.Timer(fault_trigger_time, lambda _: fault_depth_sensor_zero.enable(), oneshot=True)

    try:
        rospy.spin()
    finally:
        log.close()
    

if __name__ == '__main__':
    main()