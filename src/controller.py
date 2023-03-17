import math
import rospy
from avl_fault_detection import logger
from avl_msgs.msg import VehicleStateMsg, Float64SetpointMsg, ActuatorControlMsg
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

def main():
    # Create a node
    rospy.init_node('controller', anonymous=True)

    start_time = rospy.Time.now()
    logging_rate_hz = rospy.Rate(10)
    setpt_pub_rate_hz = rospy.Rate(1)

    log = logger.AvlLogger('controller')
    log.write_msg_header([VehicleStateMsg], ["fault_type"], units=["none"])

    # Create a subscriber for the sim state topic and print the sim state to the terminal every 1 second
    def log_sim_state(msg: VehicleStateMsg):
        fault_type = "none" if rospy.Time.now() - start_time < rospy.Duration(15) else "depth_sensor_zero"
        log.write_msg_data([msg], [fault_type])
        log.flush() # for debugging purposes
        print(f"{msg}\r")
        # print(f"{15} {math.radians(15)} {math.degrees(msg.pitch)} {msg.pitch}\r")
        logging_rate_hz.sleep()

    # Create RPM and pitch setpoint publishes to get the AUV moving
    pub_rpm_setpt = rospy.Publisher("setpoint/rpm", Float64SetpointMsg, queue_size=100)
    pub_pit_setpt = rospy.Publisher("setpoint/pitch", Float64SetpointMsg, queue_size=100)
    pub_act_ctl = rospy.Publisher("device/actuator_control", ActuatorControlMsg, queue_size=100)

    rospy.Subscriber("/sim/state", VehicleStateMsg, log_sim_state)

    # Print a list of all currently published topics
    # for name, topic_type in rospy.get_published_topics():
    #     print(f"{name} {topic_type}")

    while not rospy.is_shutdown():
        pub_act_ctl.publish(ActuatorControlMsg(True))
        pub_rpm_setpt.publish(Float64SetpointMsg(True, 10))
        pub_pit_setpt.publish(Float64SetpointMsg(True, math.radians(15)))
        setpt_pub_rate_hz.sleep()
        ...
    
    log.close()

if __name__ == '__main__':
    main()