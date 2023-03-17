from pathlib import Path
import roslaunch
import time
import sys
import rospy
from avl_fault_detection import logger

SRC_PATH = "/workspaces/AUV-Fault-Detection/avl/src"

def main():

    n_trials = 1
    trial_duration_s = float('inf')
    fault_trigger_time = 120

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [f"{SRC_PATH}/avl_fault_detection/launch/data_collection.launch"], is_core=True)

    def clean_exit():
        launch.shutdown()
        while True:
            sys.exit(0)

    for n_trial in range(n_trials):
        print(f"Starting trial {n_trial}")

        # For whatever reason /var/avl_logs/current isn't deleted when we shut down the simulation via the API
        # Keeping this allows the logger to append to the same log
        # Deleting this forces the logger to create a new folder for logs
        logger.AvlLogger.remove_current()

        # Create AVL log directories. This is what avl start does before running roslaunch.
        logger.AvlLogger.split()

        # This is a dumb hack - write fault trigger time param to file referenced by launch file so that it's loaded before nodes init
        with open(f"{SRC_PATH}/avl_fault_detection/src/fault_generation.config", "w") as f:
            f.writelines([f"fault_trigger_time: {fault_trigger_time}"])

        # Launch .launch file
        launch.start()
        # Start a node so we can hook into ROS and wait for shutdown
        rospy.init_node('data_collection_launcher', anonymous=True)
        rospy.on_shutdown(clean_exit)
        start_time = time.time()

        while time.time() - start_time < trial_duration_s and not rospy.is_shutdown():
            launch.spin_once()
        
        print(f"Ending trial {n_trial}")
        launch.shutdown()


if __name__ == '__main__':
    main()