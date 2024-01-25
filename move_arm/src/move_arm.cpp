#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

// create the custom class
//class MoveSpotArm : public rclcpp::Node
//{
//    public:
//    MoveSpotArm():Node("move_spot_arm",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
//
////        static const std::string arm_plan_group_name = "spot_arm";
////        static const std::string grip_l_plan_group_name = "grip_l";
////        static const std::string grip_r_plan_group_name = "grip_r";
////        static const std::string grip_f_plan_group_name = "grip_f";
////      creating the move_groups needed
//        auto move_group_arm = MoveGroupInterface(this, "spot_arm");
//        auto move_group_grip_l = MoveGroupInterface(this, "grip_l");
//        auto move_group_grip_r = MoveGroupInterface(this, "grip_r");
//        auto move_group_grip_f = MoveGroupInterface(this, "grip_f");
//        geometry_msgs::msg::Pose pose;
//        void setPoseTarget(float x, float y, float z)
//        {
//            pose.position.x = x;
//            pose.position.y = y;
//            pose.position.z = z;
//            //move_group_arm.setPoseTarget(target_pose);
//        };
//
//}
//



int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("move_arm",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_arm");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "spot_arm");

  // Set a target Pose
  auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.position.x = 0.5;
  msg.position.y = 0.0;
  msg.position.z = 0.5;
  return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
   move_group_interface.execute(plan);
  } else {
   RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();



  return 0;
}