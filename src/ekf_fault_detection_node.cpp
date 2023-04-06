//==============================================================================
// ECE MDE 31 
//
// Description: Processes published filter residual values in real time to detect
//              possible vehicle faults and stops the vehicle with an error 
//              message if a fault is detected
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: nav/gps_residual (geometry_msgs/Vector3)
//              nav/range_residual (geometry/msgs/Vector3)
//              nav/dvl_residual (geometry/msgs/Vector3)
//              nav/depth_residual (std_msgs/Float64)
//==============================================================================

// Node base class
#include <avl_core/node.h>
#include <avl_core/monitored_subscriber.h>
#include <avl_navigation/algorithm/moving_avg.h>


// ROS message includes
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
// using namespace avl_msgs;
using namespace avl;
using namespace Eigen;
using namespace geometry_msgs;
using namespace std_msgs;


//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class EkfFaultDetectionNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        EkfFaultDetectionNode constructor
    //--------------------------------------------------------------------------
    EkfFaultDetectionNode(int argc, char **argv) : Node(argc, argv), depth_res_avg(10)
    {

    }

private:
    // Subscribers for filter residuals
    ros::Subscriber gps_res_sub;
    ros::Subscriber range_res_sub;
    ros::Subscriber dvl_res_sub;
    ros::Subscriber depth_res_sub;

    // Publishers for fault status
    ros::Publisher depth_zero_fault;

    MovingAvg depth_res_avg;

private:
    //--------------------------------------------------------------------------
    // Name:        gps_res_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void gps_res_msg_callback(const geometry_msgs::Vector3& message)
    {

        Eigen::Vector3d res = {message.x, message.y, message.z};

    }

    //--------------------------------------------------------------------------
    // Name:        range_res_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void range_res_msg_callback(const geometry_msgs::Vector3& message)
    {

        Vector3d res = {message.x, message.y, message.z};

    }

    //--------------------------------------------------------------------------
    // Name:        dvl_res_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void dvl_res_msg_callback(const geometry_msgs::Vector3& message)
    {

        Vector3d res = {message.x, message.y, message.z};

    }

    //--------------------------------------------------------------------------
    // Name:        depth_res_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void depth_res_msg_callback(const std_msgs::Float64& message)
    {
        depth_res_avg.add_sample(message.data);
        if(depth_res_avg.get_stddev() > 10) {
            Float64 fault_msg;
            fault_msg.data = 1.0;
            depth_zero_fault.publish(fault_msg);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {
        gps_res_sub =   node_handle->subscribe("nav/gps_residual",      1,  &EkfFaultDetectionNode::gps_res_msg_callback, this);
        range_res_sub = node_handle->subscribe("nav/range_residual",    1,  &EkfFaultDetectionNode::range_res_msg_callback, this);
        dvl_res_sub =   node_handle->subscribe("nav/dvl_residual",      1,  &EkfFaultDetectionNode::dvl_res_msg_callback, this);
        depth_res_sub = node_handle->subscribe("nav/depth_residual",    1,  &EkfFaultDetectionNode::depth_res_msg_callback, this);

        // Configure as latched so we only have to update once
        depth_zero_fault = node_handle->advertise<Float64>("fault_pred/kf/depth_zero_fault", 1, true);
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
    EkfFaultDetectionNode node(argc, argv);
    node.start();
    return 0;
}

