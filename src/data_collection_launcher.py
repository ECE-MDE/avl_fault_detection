from pathlib import Path
import roslaunch
import time
import sys
import rospy
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
    trial_duration_s = rospy.Duration(240)
    fault_trigger_time = 120

    def clean_exit():
        launch.shutdown()
        sys.exit("Execution completed")

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
        launch.start()

        # Start a node so we can hook into ROS and wait for shutdown
        rospy.init_node('data_collection_launcher', anonymous=True)
        rospy.on_shutdown(clean_exit)

        # rospy.Timer(trial_duration_s, lambda: launch.shutdown(), oneshot=True)
        # launch.spin()
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < trial_duration_s and not rospy.is_shutdown():
            print(rospy.Time.now().secs)
            launch.spin_once()
            # rospy.spin_once()
        
        print(f"\n\nEnding trial {n_trial}\n\n")
        launch.shutdown()


if __name__ == '__main__':
    main()