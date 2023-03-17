from pathlib import Path
import roslaunch
import time
import subprocess
import rospy

SRC_PATH = "/workspaces/AUV-Fault-Detection/avl/src"
CURRENT_LOG_PATH = Path("/var/avl_logs/current")

def main():
    n_trials = 1
    trial_duration_s = float('inf')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    for n_trial in range(n_trials):
        roslaunch.configure_logging(uuid)
        # launch = roslaunch.scriptapi.ROSLaunch()
        launch = roslaunch.parent.ROSLaunchParent(uuid, [f"{SRC_PATH}/avl_fault_detection/launch/full_system_simulation.launch"], is_core=True)

        # Start another node
        # node = roslaunch.core.Node(package, executable)
        # launch.launch(node)

        print(f"Starting trial {n_trial}")
        # For whatever reason /var/avl_logs/current isn't deleted when we shut down the simulation via the API
        # Keeping this allows the logger to append to the same log
        # Deleting this forces the logger to create a new folder for logs
        if CURRENT_LOG_PATH.exists():
            CURRENT_LOG_PATH.unlink()
        # Create AVL log directories. This is what avl start does before running roslaunch.
        subprocess.run(f"{SRC_PATH}/avl_tools/scripts/avl.sh split".split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, text=True)
        # Launch .launch file
        launch.start()
        start_time = time.time()

        while time.time() - start_time < trial_duration_s and not rospy.is_shutdown():
            launch.spin_once()
        
        print(f"Ending trial {n_trial}")
        launch.shutdown()


if __name__ == '__main__':
    main()