import uuid
import roslaunch
import time
import subprocess

n_trials = 10
trial_duration_s = 30
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

for n_trial in range(n_trials):
    roslaunch.configure_logging(uuid)
    # launch = roslaunch.scriptapi.ROSLaunch()
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/workspaces/AUV-Fault-Detection/avl/src/avl_fault_detection/launch/full_system_simulation.launch"], is_core=True)

    # Start another node
    # node = roslaunch.core.Node(package, executable)
    # launch.launch(node)

    print(f"Starting trial {n_trial}")
    # Create AVL log directories. This is what avl start does before running roslaunch.
    subprocess.run("avl split".split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, text=True)
    # Launch .launch file
    launch.start()
    start_time = time.time()

    while time.time() - start_time < trial_duration_s:
        launch.spin_once()
    
    print(f"Ending trial {n_trial}")
    launch.shutdown()