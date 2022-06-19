#include "ros/ros.h"
#include "scc_atlantis_ros1/SetMode.h"
#include "scc_atlantis_ros1/Arm.h"
#include "scc_atlantis_ros1/Takeoff.h"
#include "scc_atlantis_ros1/Land.h"
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
            // Saving own copy of node handle
            nh_ = nh;

            // Some parameters
            // TODO: put these as arg in launch
            max_altitude_ = 10;

            // Initialize
            // robots_ : a map from namespace to robot pointer
            // pose_pubs_ : a map from namespace to pose publisher
            robots_["raven"] = std::make_shared<UAV>("raven");
            robots_["crow"] = std::make_shared<UAV>("crow");

            // Pubs
            pose_pubs_["raven"] = std::make_shared<ros::Publisher>(nh->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10));
            pose_pubs_["crow"] = std::make_shared<ros::Publisher>(nh->advertise<geometry_msgs::PoseStamped>("/crow/mavros/setpoint_position/local", 10));

            // Services
            set_mode_servers_["crow"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::SetMode::Request, scc_atlantis_ros1::SetMode::Response>("/scc/crow/set_mode", boost::bind(&SCC::set_mode_cb, this, _1, _2, robots_["crow"])));
            set_mode_servers_["raven"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::SetMode::Request, scc_atlantis_ros1::SetMode::Response>("/scc/set_mode", boost::bind(&SCC::set_mode_cb, this, _1, _2, robots_["raven"])));
            arm_servers_["crow"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::Arm::Request, scc_atlantis_ros1::Arm::Response>("/scc/crow/arm", boost::bind(&SCC::arm_cb, this, _1, _2, robots_["crow"])));
            arm_servers_["raven"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::Arm::Request, scc_atlantis_ros1::Arm::Response>("/scc/arm", boost::bind(&SCC::arm_cb, this, _1, _2, robots_["raven"])));
            takeoff_servers_["crow"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::Takeoff::Request, scc_atlantis_ros1::Takeoff::Response>("/scc/crow/takeoff", boost::bind(&SCC::takeoff_cb, this, _1, _2, robots_["crow"])));
            takeoff_servers_["raven"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::Takeoff::Request, scc_atlantis_ros1::Takeoff::Response>("/scc/takeoff", boost::bind(&SCC::takeoff_cb, this, _1, _2, robots_["raven"])));
            land_servers_["crow"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::Land::Request, scc_atlantis_ros1::Land::Response>("/scc/crow/land", boost::bind(&SCC::land_cb, this, _1, _2, robots_["crow"])));
            land_servers_["raven"] = std::make_shared<ros::ServiceServer>(nh->advertiseService<scc_atlantis_ros1::Land::Request, scc_atlantis_ros1::Land::Response>("/scc/land", boost::bind(&SCC::land_cb, this, _1, _2, robots_["raven"])));

            // Clients
            set_mode_clients_["raven"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"));
            set_mode_clients_["crow"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::SetMode>("/crow/mavros/set_mode"));
            arm_clients_["raven"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"));
            arm_clients_["crow"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::CommandBool>("/crow/mavros/cmd/arming"));
            takeoff_clients_["raven"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff"));
            takeoff_clients_["crow"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::CommandTOL>("/crow/mavros/cmd/takeoff"));
            land_clients_["raven"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land"));
            land_clients_["crow"] = std::make_shared<ros::ServiceClient>(nh->serviceClient<mavros_msgs::CommandTOL>("/crow/mavros/cmd/land"));
            
            // Subscribers
            odom_subs_["raven"] = std::make_shared<ros::Subscriber>(nh_->subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, boost::bind(&SCC::odom_cb, this, _1, robots_["raven"])));
            odom_subs_["crow"] = std::make_shared<ros::Subscriber>(nh_->subscribe<nav_msgs::Odometry>("/crow/mavros/global_position/local", 10, boost::bind(&SCC::odom_cb, this, _1, robots_["crow"])));
            state_subs_["raven"] = std::make_shared<ros::Subscriber>(nh_->subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&SCC::state_cb, this, _1, robots_["raven"])));
            state_subs_["crow"] = std::make_shared<ros::Subscriber>(nh_->subscribe<mavros_msgs::State>("/crow/mavros/state", 10, boost::bind(&SCC::state_cb, this, _1, robots_["crow"])));

            // Initialize local frames
            initialize_local_frame(robots_["raven"]);
            initialize_local_frame(robots_["crow"]);
        }
        bool set_mode_cb(scc_atlantis_ros1::SetMode::Request &req, scc_atlantis_ros1::SetMode::Response &res, std::shared_ptr<UAV> robot)
        {
            mavros_msgs::SetMode set_mode_srv;
            set_mode_srv.request.custom_mode = req.mode.c_str();
            
            if (set_mode_clients_[robot->get_namespace_name()]->call(set_mode_srv))
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
        bool arm_cb(scc_atlantis_ros1::Arm::Request &req, scc_atlantis_ros1::Arm::Response &res, std::shared_ptr<UAV> robot)
        {
            mavros_msgs::CommandBool arm_srv;
            arm_srv.request.value = req.value;
            
            if (arm_clients_[robot->get_namespace_name()]->call(arm_srv) && arm_srv.response.success)
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
        bool takeoff_cb(scc_atlantis_ros1::Takeoff::Request &req, scc_atlantis_ros1::Takeoff::Response &res, std::shared_ptr<UAV> robot)
        {
            if (robot->get_state().mode != "GUIDED")
            {
                ROS_ERROR("Before taking off set the robot mode to GUIDED");
                return false;
            }
            if(req.altitude > max_altitude_)
            {
                ROS_ERROR("Altitude bigger than %d not allowed!", max_altitude_);
                return false;
            }

            std::string robot_ns_name = robot->get_namespace_name();
            takeoff(req.altitude, robot, pose_pubs_[robot_ns_name], arm_clients_[robot_ns_name], takeoff_clients_[robot_ns_name]);
            
            ROS_INFO("Takeoff order was a success!");
            return true;
        }
        bool land_cb(scc_atlantis_ros1::Land::Request &req, scc_atlantis_ros1::Land::Response &res, std::shared_ptr<UAV> robot)
        {
            land(robot, land_clients_[robot->get_namespace_name()]);
            ROS_INFO("Land order was a success!");
            return true;
        }
        void odom_cb(const nav_msgs::Odometry::ConstPtr& msg, std::shared_ptr<UAV> robot)
        {
            robot->set_pose(*msg);
            //enu_2_local(robot->get_pose()); // TODO: check if this still applies
            float q0 = (*msg).pose.pose.orientation.w;
            float q1 = (*msg).pose.pose.orientation.x;
            float q2 = (*msg).pose.pose.orientation.y;
            float q3 = (*msg).pose.pose.orientation.z;
            float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
            robot->set_heading(psi*(180/M_PI) - robot->get_local_offset());
        }
        void state_cb(const mavros_msgs::State::ConstPtr& msg, std::shared_ptr<UAV> robot)
        {
            robot->set_state(*msg);
        }
    private:
        std::shared_ptr<ros::NodeHandle> nh_;

        // Services
        std::map<std::string, std::shared_ptr<ros::ServiceServer>> arm_servers_;
        std::map<std::string, std::shared_ptr<ros::ServiceServer>> set_mode_servers_;
        std::map<std::string, std::shared_ptr<ros::ServiceServer>> takeoff_servers_;
        std::map<std::string, std::shared_ptr<ros::ServiceServer>> land_servers_;

        // Clients
        std::map<std::string, std::shared_ptr<ros::ServiceClient>> arm_clients_;
        std::map<std::string, std::shared_ptr<ros::ServiceClient>> set_mode_clients_;
        std::map<std::string, std::shared_ptr<ros::ServiceClient>> takeoff_clients_;
        std::map<std::string, std::shared_ptr<ros::ServiceClient>> land_clients_;

        // Pubs
        std::map<std::string, std::shared_ptr<ros::Publisher>> pose_pubs_;

        // Subscribers
        std::map<std::string, std::shared_ptr<ros::Subscriber>> odom_subs_;
        std::map<std::string, std::shared_ptr<ros::Subscriber>> state_subs_;
        
        // Parameters
        uint8_t max_altitude_;

        // Robots
        std::map<std::string, std::shared_ptr<UAV>> robots_;

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