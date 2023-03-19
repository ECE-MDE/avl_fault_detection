from pathlib import Path
import roslaunch
import time
import sys
import rospy
import roslibpy
import signal
from avl_fault_detection import logger

SRC_PATH = "/workspaces/AUV-Fault-Detection/avl/src"

"""
TODO
- Randomize fault_trigger_time for each trial
- Figure out why starting roscore from this process freezes rospy.Time.now()
- Allow setting n_trials, trial_duration_s, etc from cmd-line args
- Add flag for whether AvlLogger.remove_current() is called
"""

def main():

    n_trials = 2
    trial_duration_s = 60
    fault_trigger_time = 30

    ros = roslibpy.Ros('localhost', 9090)
    ros.on_ready(lambda: print('Is ROS connected?', ros.is_connected))

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
        launch = roslaunch.parent.ROSLaunchParent(uuid, [f"{SRC_PATH}/avl_fault_detection/launch/data_collection.launch"], roslaunch_strs=[f'<launch><param name="/fault_trigger_time" type="int" value="{fault_trigger_time}"/></launch>'], is_core=False)
        
        # Handle Ctrl+C
        def handle_exit(signum, frame):
            launch.shutdown()
            ros.close()
            sys.exit()

        signal.signal(signal.SIGINT, handle_exit)
        
        # Start the launch
        launch.start()

        # After the launch is finished, connect to rosbridge
        ros.connect()
        ros.run(timeout=30)

        # Publish sim_ready: True when launch is finished. This tells the data collector node to start publishing setpoints
        launch_finished_topic = roslibpy.Topic(ros, 'fault_gen/sim_ready', 'std_msgs/Bool')
        launch_finished_topic.publish(roslibpy.Message({'data': True}))

        start_time = ros.get_time().secs
        while ros.get_time().secs - start_time < trial_duration_s:
            # print(f"{ros.get_time().secs}\r")
            launch.spin_once()
        
        print(f"\n\nEnding trial {n_trial}\n\n")
        launch.shutdown()
        ros.close()

    sys.exit()

if __name__ == '__main__':
    main()