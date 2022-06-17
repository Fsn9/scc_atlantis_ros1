#include "ros/ros.h"
#include "scc_atlantis_ros1/SetMode.h"
#include "scc_atlantis_ros1/Arm.h"
#include "scc_atlantis_ros1/Takeoff.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include <geometry_msgs/PoseStamped.h>
#include "gnc_functions.h"
#include "robots.h"

class SCC
{
    public:
        SCC(std::shared_ptr<ros::NodeHandle> nh)
        {
            // Services
            set_mode_srv_ = nh->advertiseService("/scc/set_mode", &SCC::set_mode_cb, this);
            arm_srv_ = nh->advertiseService("/scc/arm", &SCC::arm_cb, this);
            takeoff_srv_ = nh->advertiseService("/scc/takeoff", &SCC::takeoff_cb, this);

            // Clients
            set_mode_client_ = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
            arm_client_ = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

            // Pubs
            takeoff_pub_ = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

            // Saving own copy of node handle
            nh_ = nh;

            // Some parameters
            // TODO: put these as arg in launch
            max_altitude_ = 10;

            // Initialize robots
            raven_ = std::make_shared<Raven>();
            crow_ = std::make_shared<Crow>();
        }
        bool set_mode_cb(scc_atlantis_ros1::SetMode::Request &req, scc_atlantis_ros1::SetMode::Response &res)
        {
            mavros_msgs::SetMode set_mode_srv;
            set_mode_srv.request.custom_mode = req.mode.c_str();
            
            if (set_mode_client_.call(set_mode_srv))
            {
                ROS_INFO("%s mode was set!", req.mode.c_str());
                return true;
            }
            else
            {
                ROS_ERROR("Failed to set %s mode", req.mode.c_str());
                return false;
            }
        }
        bool arm_cb(scc_atlantis_ros1::Arm::Request &req, scc_atlantis_ros1::Arm::Response &res)
        {
            mavros_msgs::CommandBool arm_srv;
            arm_srv.request.value = req.value;
            
            if (arm_client_.call(arm_srv) && arm_srv.response.success)
            {
                ROS_INFO("arm order was sent!");
                return true;
            }
            else
            {
                ROS_ERROR("Failed to order arm");
                return false;
            }
        }
        bool takeoff_cb(scc_atlantis_ros1::Takeoff::Request &req, scc_atlantis_ros1::Takeoff::Response &res)
        {
            if(req.altitude > max_altitude_)
            {
                ROS_ERROR("Altitude bigger than %d not allowed!", max_altitude_);
                return false;
            }

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = req.altitude;
            takeoff_pub_.publish(pose);

            //send a few setpoints before starting
            ros::Rate rate(20.0);
            for(int i = 100; ros::ok() && i > 0; --i){
                takeoff_pub_.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            
            ROS_INFO("Takeoff order was sent!");
            return true;
        }

    private:
        std::shared_ptr<ros::NodeHandle> nh_;
        // Servers
        ros::ServiceServer set_mode_srv_;
        ros::ServiceServer takeoff_srv_;
        ros::ServiceServer arm_srv_;
        // Clients
        ros::ServiceClient set_mode_client_;
        ros::ServiceClient arm_client_;
        // Pubs
        ros::Publisher takeoff_pub_;
        
        // Parameters
        uint8_t max_altitude_;

        // Robots
        std::shared_ptr<Crow> crow_;
        std::shared_ptr<Raven> raven_;

};

int main(int argc, char **argv)
{
    // Initialize ros node object
    ros::init(argc, argv, "scc");
    // Node Handle will be shared between main and RTL server object
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();

    // Create Scc server
    SCC scc(nh);
    
    ROS_INFO("Ready to provide MavLink command services: arm, takeoff, set_mode");

    ros::spin();

    return 0;
}