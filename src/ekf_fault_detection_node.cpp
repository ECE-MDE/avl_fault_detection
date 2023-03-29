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
// Subscribers: device/depth (std_msgs/Float64)
//              navigation/nav (avl_navigation/NavigationMsg)
//              device/rpm (std_msgs/Float64)
//              device/height (std_msgs/Float64)

// All available setpoint messages:
//              setpoint/depth (avl_msgs/Float64SetpointMsg)
//              setpoint/elevator (avl_msgs/Float64SetpointMsg)
//              setpoint/ground_speed 
//              setpoint/height (avl_msgs/Float64SetpointMsg)
//              setpoint/line
//              setpoint/orbit
//              setpoint/pitch (avl_msgs/Float64SetpointMsg)
//              setpoint/roll (avl_msgs/Float64SetpointMsg)
//              setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              setpoint/rudder (avl_msgs/Float64SetpointMsg)
//              setpoint/water_speed
//              setpoint/yaw (avl_msgs/Float64SetpointMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>
#include <avl_core/monitored_subscriber.h>


// ROS message includes
#include <std_msgs/Float64.h>
#include <avl_msgs/GpsMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/AhrsMsg.h>
using namespace avl_msgs;

// Extended kalman filter
#include <avl_navigation/filter/ekf.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class EkfFaultDetectionNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        EkfFaultDetectionNode constructor
    //--------------------------------------------------------------------------
    EkfFaultDetectionNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Subscribers for sensor data
    MonitoredSubscriber<NavigationMsg> nav_sub;
    MonitoredSubscriber<std_msgs::Float64> depth_sub;
    MonitoredSubscriber<std_msgs::Float64> rpm_sub;
    MonitoredSubscriber<std_msgs::Float64> height_sub;

    // Subscribers for vehicle setpoints
    MonitoredSubscriber<Float64SetpointMsg> depth_cmd_sub;
    MonitoredSubscriber<Float64SetpointMsg> height_cmd_sub;
    MonitoredSubscriber<Float64SetpointMsg> roll_cmd_sub;
    MonitoredSubscriber<Float64SetpointMsg> pitch_cmd_sub;
    MonitoredSubscriber<Float64SetpointMsg> yaw_cmd_sub;
    MonitoredSubscriber<Float64SetpointMsg> elevator_cmd_sub;
    MonitoredSubscriber<Float64SetpointMsg> rudder_cmd_sub;
    MonitoredSubscriber<Float64SetpointMsg> rpm_cmd_sub;

    // Timer and its duration for iterations
    ros::Timer iteration_timer;
    ros::Duration iteration_duration;

    // Variables to store latest sensor data
    double roll = NAN;
    double pitch = NAN;
    double yaw = NAN;
    double vn = NAN;
    double ve = NAN;
    double vd = NAN;
    double lat = NAN;
    double lon = NAN;
    double alt = NAN;
    double depth = NAN;
    double height = NAN;
    double rpm = NAN;

    // Variables to store latest vehicle commands
    double roll_cmd = NAN;
    double pitch_cmd = NAN;
    double yaw_cmd = NAN;
    double vn_cmd = NAN;
    double ve_cmd = NAN;
    double vd_cmd = NAN;
    double lat_cmd = NAN;
    double lon_cmd = NAN;
    double alt_cmd = NAN;
    double depth_cmd = NAN;
    double height_cmd = NAN;
    double rpm_cmd = NAN;
    double elevator_cmd = NAN;
    double rudder_cmd = NAN;

//TODO use filter
    // EKF for calculating residuals
    // Ekf filter;

private:

    //--------------------------------------------------------------------------
    // Name:        depth_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void depth_msg_callback(const std_msgs::Float64& message)
    {
        depth = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        height_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void height_msg_callback(const std_msgs::Float64& message)
    {
        height = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        rpm_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void rpm_msg_callback(const std_msgs::Float64& message)
    {
        rpm = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        nav_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void nav_msg_callback(const NavigationMsg& message)
    {
        roll =  message.roll;
        pitch = message.pitch;
        yaw =   message.yaw;
        // vn =    message.vn;
        // ve =    message.ve;
        // vd =    message.vd;
        lat =   message.lat;
        lon =   message.lon;
        alt =   message.alt;
    }

    //--------------------------------------------------------------------------
    // Name:        depth_setpoint_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void depth_setpoint_callback(const Float64SetpointMsg &message)
    {
        depth_cmd = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        height_setpoint_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void height_setpoint_callback(const Float64SetpointMsg &message)
    {
        height_cmd = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        roll_setpoint_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void roll_setpoint_callback(const Float64SetpointMsg &message)
    {
        roll_cmd = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        pitch_setpoint_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void pitch_setpoint_callback(const Float64SetpointMsg &message)
    {
        // If there is an elevator setpoint, it takes precedence over
        // the fin angles from attitude setpoints
        pitch_cmd = std::isnan(elevator_cmd) ? message.data : elevator_cmd;
    }

    //--------------------------------------------------------------------------
    // Name:        yaw_setpoint_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void yaw_setpoint_callback(const Float64SetpointMsg &message)
    {
        // If there is a rudder setpoint, it takes precedence over
        // the fin angles from attitude setpoints
        yaw_cmd = std::isnan(rudder_cmd) ? message.data : rudder_cmd;
    }

    //--------------------------------------------------------------------------
    // Name:        elevator_cmd_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void elevator_cmd_callback(const Float64SetpointMsg &message)
    {
        elevator_cmd = message.data;
        pitch_cmd = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        rudder_cmd_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void rudder_cmd_callback(const Float64SetpointMsg &message)
    {
        rudder_cmd = message.data;
        yaw_cmd = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        rpm_setpoint_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void rpm_setpoint_callback(const Float64SetpointMsg &message)
    {
        rpm_cmd = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by the monitored subscriber.
    // Arguments:   - fault: fault event structure
    //--------------------------------------------------------------------------
    void fault_callback(const avl::SubscriberFault& fault)
    {


        if (fault.topic == "device/depth")
        {
            depth = NAN;
            depth_sub.reset();
        }
        else if (fault.topic == "device/height")
        {
            height = NAN;
            height_sub.reset();
        }
        else if (fault.topic == "device/rpm")
        {
            rpm = NAN;
            rpm_sub.reset();
        }
        else if (fault.topic == "navigation/nav")
        {
            roll =  NAN;
            pitch = NAN;
            yaw =   NAN;
            vn =    NAN;
            ve =    NAN;
            vd =    NAN;
            lat =   NAN;
            lon =   NAN;
            alt =   NAN;
            nav_sub.reset();
        }
        else if (fault.topic == "setpoint/depth")
        {
            depth_cmd = NAN;
            depth_cmd_sub.reset();
        }
        else if (fault.topic == "setpoint/height")
        {
            height_cmd = NAN;
            height_cmd_sub.reset();
        }
        else if (fault.topic == "setpoint/roll")
        {
            roll_cmd = NAN;
            roll_cmd_sub.reset();
        }
        else if (fault.topic == "setpoint/pitch")
        {
            pitch_cmd = NAN;
            pitch_cmd_sub.reset();
        }
        else if (fault.topic == "setpoint/yaw")
        {
            yaw_cmd = NAN;
            yaw_cmd_sub.reset();
        }
        else if (fault.topic == "setpoint/elevator")
        {
            elevator_cmd = NAN;
            elevator_cmd_sub.reset();
        }
        else if (fault.topic == "setpoint/rudder")
        {
            rudder_cmd = NAN;
            rudder_cmd_sub.reset();
        }
        else if (fault.topic == "setpoint/rpm")
        {
            rpm_cmd = NAN;
            rpm_cmd_sub.reset();
        }

    }

    //--------------------------------------------------------------------------
    // Name:        iteration_callback
    // Description: Called when the iteration timer expires. Runs EKF to find
    //              residual values and check for faults.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void ekf_iteration(const ros::TimerEvent& event)
    {
//TODO 
        // Run EKF filter
        // Get residuals from filter
        // Check residuals for error (threshold or std deviation check)
            // Can store thresholds in config file
        double depth_thresh =  get_param<double>("~thresholds/depth");
        double height_thresh = get_param<double>("~thresholds/height");

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {
        // Set up the publishers and subscribers
        depth_sub.subscribe("device/depth", 1,
            &EkfFaultDetectionNode::depth_msg_callback,
            &EkfFaultDetectionNode::fault_callback, this);
        height_sub.subscribe("device/height", 1,
            &EkfFaultDetectionNode::height_msg_callback,
            &EkfFaultDetectionNode::fault_callback, this);
        rpm_sub.subscribe("device/rpm", 1,
            &EkfFaultDetectionNode::rpm_msg_callback,
            &EkfFaultDetectionNode::fault_callback, this);
        nav_sub.subscribe("navigation/nav", 1,
            &EkfFaultDetectionNode::nav_msg_callback,
            &EkfFaultDetectionNode::fault_callback, this);
        depth_cmd_sub.subscribe("setpoint/depth", 8,
                        &EkfFaultDetectionNode::depth_setpoint_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
        height_cmd_sub.subscribe("setpoint/height", 8,
                        &EkfFaultDetectionNode::height_setpoint_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
        roll_cmd_sub.subscribe("setpoint/roll", 8,
                        &EkfFaultDetectionNode::roll_setpoint_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
        pitch_cmd_sub.subscribe("setpoint/pitch", 8,
                        &EkfFaultDetectionNode::pitch_setpoint_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
        yaw_cmd_sub.subscribe("setpoint/yaw", 8,
                        &EkfFaultDetectionNode::yaw_setpoint_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
        elevator_cmd_sub.subscribe("setpoint/elevator", 8,
                        &EkfFaultDetectionNode::elevator_cmd_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
        rudder_cmd_sub.subscribe("setpoint/rudder", 8,
                        &EkfFaultDetectionNode::rudder_cmd_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
        rpm_cmd_sub.subscribe("setpoint/rpm", 8,
                        &EkfFaultDetectionNode::rpm_setpoint_callback,
                        &EkfFaultDetectionNode::fault_callback, this);
                        
        depth_sub.set_message_rate(0.5);
        height_sub.set_message_rate(0.5);
        rpm_sub.set_message_rate(0.5);
        nav_sub.set_message_rate(0.5);
        depth_cmd_sub.set_message_rate(0.5);
        height_cmd_sub.set_message_rate(0.5);
        roll_cmd_sub.set_message_rate(0.5);
        pitch_cmd_sub.set_message_rate(0.5);
        yaw_cmd_sub.set_message_rate(0.5);
        elevator_cmd_sub.set_message_rate(0.5);
        rudder_cmd_sub.set_message_rate(0.5);
        rpm_cmd_sub.set_message_rate(0.5);

        depth_sub.enable_message_rate_check(true);
        height_sub.enable_message_rate_check(true);
        rpm_sub.enable_message_rate_check(true);
        nav_sub.enable_message_rate_check(true);
        depth_cmd_sub.enable_message_rate_check(true);
        height_cmd_sub.enable_message_rate_check(true);
        roll_cmd_sub.enable_message_rate_check(true);
        pitch_cmd_sub.enable_message_rate_check(true);
        yaw_cmd_sub.enable_message_rate_check(true);
        elevator_cmd_sub.enable_message_rate_check(true);
        rudder_cmd_sub.enable_message_rate_check(true);
        rpm_cmd_sub.enable_message_rate_check(true);

        // Set up the iteration timer
        double iteration_rate = get_param<double>("~iteration_rate");
        iteration_duration = ros::Duration(1.0/iteration_rate);
        iteration_timer = node_handle->createTimer(iteration_duration,
            &EkfFaultDetectionNode::ekf_iteration, this);

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
    EkfFaultDetectionNode node(argc, argv);
    node.start();
    return 0;
}

