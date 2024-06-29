#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/string.hpp"
#include "yolov8_msgs/srv/move.hpp"
#include "yolov8_msgs/msg/ws.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>


using namespace std::placeholders;
using std::placeholders::_1;

using moveit::planning_interface::MoveGroupInterface;

using namespace std::literals::chrono_literals;
//class Chip {
//    public:
//        Chip():
//        {
//        float x,y;
//        }
//
//};

// Create the custom class
class CnS_Task_SpotArm : public rclcpp::Node {
    public:
        CnS_Task_SpotArm()
        : Node("chip_socket_task", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {
            // Creating callback groups
            client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            member_cb_group_ = client_cb_group_;
            //Creating the subscription to the perception topic
            Perception_sub_ = this->create_subscription<yolov8_msgs::msg::Ws>("/Ws_res", 10, std::bind(&CnS_Task_SpotArm::Estimate, this, _1));
            RCLCPP_INFO(this->get_logger(), "Perception Subscription created");
            // The following (Cmd_sub_) is only for testing
            auto subscription_options = rclcpp::SubscriptionOptions();
            subscription_options.callback_group = member_cb_group_;
            Cmd_sub_ = this->create_subscription<std_msgs::msg::String>("/test_cmd", 10, std::bind(&CnS_Task_SpotArm::Move_cmd, this, _1),subscription_options);
            move_client_ = this->create_client<yolov8_msgs::srv::Move>("/move_srv", rmw_qos_profile_services_default, client_cb_group_);
            RCLCPP_INFO(this->get_logger(), "Move Client created");
            ptr =  std::shared_ptr<CnS_Task_SpotArm>(this);
            //This is a parts needed for the ft frame
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            service_result_ = false;
            request_sent_time = this->get_clock()->now().seconds();
            request_returned_time = this->get_clock()->now().seconds();
//            MoveGroupInterface arm_g = MoveGroupInterface(ptr, "spot_arm");
        }
    private:

        void Estimate(const yolov8_msgs::msg::Ws::SharedPtr data) {
            bool chip_d = data->chip.detected;
            bool socket_d = data->socket.detected;
            float TOF = data->tof;
//            RCLCPP_INFO(this->get_logger(),"Chip: %s, Socket: %s",chip_d ? "Detected":"Notdetected",socket_d ? "Detected":"Notdetected");
//            RCLCPP_INFO(this->get_logger(),"TOF: %.2f",TOF);
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("body", "arm_link_wr1", tf2::TimePointZero);
            grip_pos.x = transform_stamped.transform.translation.x;
            grip_pos.y = transform_stamped.transform.translation.y;
            grip_pos.z = transform_stamped.transform.translation.z;
            grip_orientation = transform_stamped.transform.rotation;
//            RCLCPP_INFO(this->get_logger(),"x= %.2f, y= %.2f, z= %.2f ",grip_pos.x,grip_pos.y,grip_pos.z);
//            RCLCPP_INFO(this->get_logger(),"x= %.2f, y= %.2f, z= %.2f, w= %.2f ",grip_orientation.x,grip_orientation.y,grip_orientation.z,grip_orientation.w);

        }
        void Move_cmd(const std_msgs::msg::String::SharedPtr data) {
            RCLCPP_INFO(this->get_logger(),"Pose: %s", data->data.c_str());
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->pose_name = "home";
            request->mode =0;
            bool good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            request->pose_name = "scan_high";
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            RCLCPP_INFO(this->get_logger(),"spot_arm controller loaded");
            request->group = "grip_f";
            request->pose_name = "close";
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            request->pose_name = "open";
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            RCLCPP_INFO(this->get_logger(),"grip_f controller loaded");
            request->group = "grip_l";
            request->pose_name = "close";
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            request->pose_name = "open";
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            RCLCPP_INFO(this->get_logger(),"grip_l controller loaded");
            request->group = "grip_r";
            request->pose_name = "close";
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            request->pose_name = "open";
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            RCLCPP_INFO(this->get_logger(),"grip_r controller loaded");

        }
        bool Send_request(const auto request){
        using ServiceResponseFuture = rclcpp::Client<yolov8_msgs::srv::Move>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                auto response = future.get();
                request_returned_time = this->get_clock()->now().seconds();
                if (response->success){
                    RCLCPP_INFO(this->get_logger(), "DONE"); // Change this to your response field
                    }
                service_result_=response->success;
                request_returned_time = this->get_clock()->now().seconds();

            };
            request_sent_time = this->get_clock()->now().seconds();
            auto future = move_client_->async_send_request(request, response_received_callback);

            return service_result_;

        }

        rclcpp::Subscription<yolov8_msgs::msg::Ws>::SharedPtr Perception_sub_;//subscription to the perception node
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Cmd_sub_;// This (Cmd_sub_) is only for testing
        rclcpp::Node::SharedPtr ptr;
        rclcpp::Client<yolov8_msgs::srv::Move>::SharedPtr move_client_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::Point grip_pos;
        geometry_msgs::msg::Quaternion grip_orientation;
        bool service_result_;
        double request_sent_time, request_returned_time ;
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr member_cb_group_;




};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
 rclcpp::executors::MultiThreadedExecutor exe;
  CnS_Task_SpotArm::SharedPtr node = std::make_shared<CnS_Task_SpotArm>();
  exe.add_node(node);
  exe.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

