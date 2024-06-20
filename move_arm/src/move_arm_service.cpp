#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include "yolov8_msgs/srv/move.hpp"

using namespace std::placeholders;
using std::placeholders::_1;

using moveit::planning_interface::MoveGroupInterface;

// Create the custom class
class MoveSpotArm : public rclcpp::Node {
    public:
        MoveSpotArm()
        : Node("move_spot_arm_service", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {
            cmd_subscription_ = this->create_subscription<std_msgs::msg::String>("cmd_move", 10, std::bind(&MoveSpotArm::move_by_cmd, this, _1));
            cmd_service_ = this->create_service<yolov8_msgs::srv::Move>("move_srv",  std::bind(&MoveSpotArm::move_by_cmd_s, this, _1, _2));
            RCLCPP_INFO(this->get_logger(), "Subscription created");
            ptr =  std::shared_ptr<MoveSpotArm>(this);
//            MoveGroupInterface arm_g = MoveGroupInterface(ptr, "spot_arm");
        }

        void move_by_cmd_s(const std::shared_ptr<yolov8_msgs::srv::Move::Request> request,
        std::shared_ptr<yolov8_msgs::srv::Move::Response>  response) {

                MoveGroupInterface group = MoveGroupInterface(ptr, request->group.c_str());
                MoveGroupInterface::Plan plan;
                group.setStartStateToCurrentState();
                group.setNamedTarget(request->pose_name.c_str());
                bool good;
                good = static_cast<bool>(group.plan(plan));
                response->success = good;
                if(good)
                    {
                     good = (group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                     RCLCPP_INFO(this->get_logger(),"Bitch, plan is %s",good ? "good" : "bad");
                     if (good){
                       RCLCPP_INFO(this->get_logger(),"Plan executed");
                     }
                     else{
                        RCLCPP_ERROR(this->get_logger(),"Failed to execute plan");
                         }
                    }
                    else{
                    RCLCPP_ERROR(this->get_logger(),"Failed to create plan");
                    }
                response->success = good;
                return;
        }


    private:

        void move_by_cmd(const std_msgs::msg::String::SharedPtr msg) {
            if (msg) {
                RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
                MoveGroupInterface group = MoveGroupInterface(ptr, "spot_arm");
                MoveGroupInterface::Plan plan;
                group.setStartStateToCurrentState();
                group.setNamedTarget(msg->data.c_str());
                bool good;
                good = static_cast<bool>(group.plan(plan));
                if(good)
                    {
                     good = (group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                     if (good){
                       RCLCPP_INFO(this->get_logger(),"Plan executed");
                       return;
                     }
                     else{
                        RCLCPP_ERROR(this->get_logger(),"Failed to execute plan");
                     }
                    }
                    else{
                    RCLCPP_ERROR(this->get_logger(),"Failed to create plan");
                    }
    //            move_to_name_pos(group_arm, &arm_plan, "home")
            } else
            {
                RCLCPP_ERROR(this->get_logger(), "Received empty message");
            }


        }


        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
        rclcpp::Node::SharedPtr ptr;
        rclcpp::Service<yolov8_msgs::srv::Move>::SharedPtr cmd_service_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  MoveSpotArm::SharedPtr node = std::make_shared<MoveSpotArm>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}