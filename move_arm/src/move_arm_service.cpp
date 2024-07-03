#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include "yolov8_msgs/srv/move.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


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
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//            MoveGroupInterface arm_g = MoveGroupInterface(ptr, "spot_arm");
        }

        void move_by_cmd_s(const std::shared_ptr<yolov8_msgs::srv::Move::Request> request,
        std::shared_ptr<yolov8_msgs::srv::Move::Response>  response) {
//            rclcpp::Duration large_timeout = rclcpp::Duration::from_seconds(1e9);  // Effectively "disabling" the timeout
            if (request->mode==0){ //Move to named poses
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
                }//mode==0 named position
            if (request->mode==1){ //Move to numeric pose
                MoveGroupInterface group = MoveGroupInterface(ptr, request->group.c_str());
                MoveGroupInterface::Plan plan;
                group.setStartStateToCurrentState();
                group.setPoseTarget(request->pose_t);
                bool good;
                good = static_cast<bool>(group.plan(plan));
                response->success = good;
                if(good)
                    {
                     good = (group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
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
                }//mode==1 numeric position
            if (request->mode==2){ //Move cartesian to numeric pose
                RCLCPP_INFO(this->get_logger(), "Starting the Cartesian motion");

                MoveGroupInterface group = MoveGroupInterface(ptr, request->group.c_str());

//                group.setStartStateToCurrentState();

                geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("body", "arm_link_wr1", tf2::TimePointZero);
                geometry_msgs::msg::Pose start_pose;
                start_pose.position.x = transform_stamped.transform.translation.x;
                start_pose.position.y = transform_stamped.transform.translation.y;
                start_pose.position.z = transform_stamped.transform.translation.z;
                start_pose.orientation.x = transform_stamped.transform.rotation.z;
                start_pose.orientation.y = transform_stamped.transform.rotation.y;
                start_pose.orientation.z = transform_stamped.transform.rotation.z;
                start_pose.orientation.w = transform_stamped.transform.rotation.w;

                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(start_pose);
                waypoints.push_back(request->pose_t);
                moveit_msgs::msg::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                bool good;
                RCLCPP_INFO(this->get_logger(), "Sending to planner");
                good = static_cast<bool>(group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory));
                if(good)
                    {
                     MoveGroupInterface::Plan plan;
                     plan.trajectory_ = trajectory;
                     good = (group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
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
                }//mode==2 trajectory
            if (request->mode==3){ //Move cartesian to numeric pose
                RCLCPP_INFO(this->get_logger(), "Moving to joint positions");
                //Define the move group
                MoveGroupInterface group = MoveGroupInterface(ptr, request->group.c_str());
//                moveit::core::RobotStatePtr current_state = group.getCurrentState(10);
//                const moveit::core::JointModelGroup* joint_model_group =
//                        group.getCurrentState()->getJointModelGroup(request->group.c_str());

                std::vector<double> joint_group_positions;
//                current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//                if(request->group.c_str() == "spot_arm"){
                    for(int i=0; i<6; i++){
                        joint_group_positions.push_back(request->joint[i]);
                        RCLCPP_INFO(this->get_logger(),"Setting Joint %d pos %f",i,request->joint[i]);
                    }
//                }

                //change the joint positions
                RCLCPP_INFO(this->get_logger(),"Setting the joint positions");
                group.setJointValueTarget(joint_group_positions);
                //Set Velocity and acceleration factors
                group.setStartStateToCurrentState();
                group.setMaxVelocityScalingFactor(0.05);
                group.setMaxAccelerationScalingFactor(0.05);
                //Define the plan
                RCLCPP_INFO(this->get_logger(),"Planning the Joints");
                MoveGroupInterface::Plan plan;
                //Planning
                bool good = (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                //Executing
                if(good)
                    {
                     good = (group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
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
                }//mode==3 robot joint positions TOFIX --no current model loaded
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
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  MoveSpotArm::SharedPtr node = std::make_shared<MoveSpotArm>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}