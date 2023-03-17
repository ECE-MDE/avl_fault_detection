//==============================================================================
// ECE MDE 31 
//
// Description: Processes published node data real-time through an extended 
//              kalman filter, analyzes the residuals to detect faults, and 
//              stops the vehicle with an error message if a fault is detected
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: device/imu (avl_msgs/ImuMsg)
//              device/ahrs (avl_msgs/AhrsMsg)
//              device/velocity (geometry_msgs/Vector3)
//              device/depth (std_msgs/Float64)
//              nav/range (avl_msgs/RangeMsg)
//              device/multibeam (avl_msgs/MultibeamMsg)
//              device/gps (avl_msgs/GpsMsg)
//              nav/gyrocompass (avl_msgs/GyrocompassMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// ROS message includes
#include <avl_msgs/GpsMsg.h>
using namespace avl_msgs;

// Extended kalman filter
#include <avl_navigation/filter/ekf.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ekfFaultDetectionNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ekfFaultDetectionNode constructor
    //--------------------------------------------------------------------------
    ekfFaultDetectionNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:
    Ekf filter;

private:

    // Read messages from sensors and setpoints
    // Get residuals from filter
    // Check residuals for error (threshold or std deviation check)
    
    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log the data header and units
        log_data("[ekf fault detection] ");
        log_data("[ekf fault detection]");

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    ekfFaultDetectionNode node(argc, argv);
    node.start();
    return 0;
}

