from pathlib import Path
from typing import Dict
import roslaunch
import time
import sys
import rospy
import roslibpy
import signal
from roslibpy import Ros, Message
from avl_fault_detection import logger, util
from avl_msgs.msg import VehicleStateMsg
from std_msgs.msg import Bool  
from rosgraph_msgs.msg import Clock

SRC_PATH = "/workspaces/AUV-Fault-Detection/avl/src"

class Fault:
    def __init__(self, ros: Ros, topic_name: str):
        self._topic_name = topic_name
        self._enabled = False
        self._fault_topic = roslibpy.Topic(ros, topic_name, 'std_msgs/Bool', queue_size=100)

    def _publish(self):
        self._fault_topic.publish(Message({'data': self._enabled}))

    def enable(self):
        self._enabled = True
        self._publish()
    
    def disable(self):
        self._enabled = False
        self._publish()

    def get_topic_name(self) -> str:
        return self._topic_name

    def is_enabled(self) -> bool:
        return self._enabled
"""
TODO
- Randomize fault_trigger_time for each trial
- Figure out why starting roscore from this process freezes rospy.Time.now()
- Allow setting n_trials, trial_duration_s, etc from cmd-line args
- Add flag for whether AvlLogger.remove_current() is called
"""

def main():
    ros = roslibpy.Ros('localhost', 9090)
    ros.on_ready(lambda: print('Is ROS connected?', ros.is_connected))

    # SIM PARAMETERS
    n_trials = 2
    trial_duration_s = 60
    fault_trigger_times: Dict[Fault, float] = {
        Fault(ros, "fault_gen/depth_sensor_zero"): 30,
    }
    hull_drag_coeff = -3.2440

    for n_trial in range(n_trials):
        print(f"\n\nStarting trial {n_trial}\n\n")

        # For whatever reason /var/avl_logs/current isn't deleted when we shut down the simulation via the API
        # Keeping this allows the logger to append to the same log
        # Deleting this forces the logger to create a new folder for logs
        logger.AvlLogger.remove_current()

        # Create AVL log directories. This is what avl start does before running roslaunch.
        logger.AvlLogger.split()

        # Launch .launch file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [f"{SRC_PATH}/avl_fault_detection/launch/data_collection.launch"], roslaunch_strs=[f'<launch><param name="/auv_dynamics_node/X_uu" type="double" value="{hull_drag_coeff}"/></launch>'], is_core=False)
        
        # Handle Ctrl+C
        def handle_exit(signum, frame):
            launch.shutdown()
            ros.close()
            util.kill_all_nodes()
            sys.exit()

        signal.signal(signal.SIGINT, handle_exit)
        
        # Start the launch
        launch.start()

        # After the launch is finished, connect to rosbridge
        ros.connect()
        ros.run(timeout=30)

        sim_state = roslibpy.Topic(ros, "/sim/state", 'avl_msgs/VehicleStateMsg')
        sim_state_initialized = False

        def wait_for_sim_state(msg: VehicleStateMsg):
            nonlocal sim_state_initialized
            if not sim_state_initialized:
                sim_state_initialized = True

        print("Waiting for first vehicle state...")
        # for topic in ros.get_topics():
        #     print(f"{topic}\t{ros.get_topic_type(topic)}")
        # Wait for the sim to be initialized
        sim_state.subscribe(wait_for_sim_state)
        while not sim_state_initialized:
            time.sleep(0.1)
        print("Starting trial!")

        start_time = ros.get_time().secs
        current_time = start_time
        while current_time - start_time < trial_duration_s:
            # Enable faults if trigger time has passed
            for fault, trigger_time in fault_trigger_times.items():
                # Ignore faults with trigger_time = None
                if trigger_time and not fault.is_enabled() and current_time > trigger_time:
                    print(f"Enabling {fault.get_topic_name()} at {current_time}")
                    fault.enable()

            # Run 1 cycle of roslaunch process
            launch.spin_once()
            # Update current time
            current_time = ros.get_time().to_sec()
        
        print(f"\n\nEnding trial {n_trial}\n\n")
        launch.shutdown()
        ros.close()

    sys.exit()

if __name__ == '__main__':
    main()