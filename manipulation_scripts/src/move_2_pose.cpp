        #include <moveit/move_group_interface/move_group_interface.h>
        #include <moveit/planning_scene_interface/planning_scene_interface.h>

        #include <moveit_msgs/msg/display_robot_state.hpp>
        #include <moveit_msgs/msg/display_trajectory.hpp>

        static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_2_pose");

        int main(int argc, char **argv) {
          rclcpp::init(argc, argv);
          rclcpp::NodeOptions node_options;
          // Define the Node
          node_options.automatically_declare_parameters_from_overrides(true);
          auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
          //Set up the Exector
          rclcpp::executors::SingleThreadedExecutor executor;
          executor.add_node(move_group_node);
          std::thread([&executor]() { executor.spin(); }).detach();

          //Set up the move group
          static const std::string PLANNING_GROUP_ARM = "spot_arm";
          moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);

          static const std::string PLANNING_GROUP_Grip = "grip";
          moveit::planning_interface::MoveGroupInterface move_group_grip(move_group_node, PLANNING_GROUP_Grip);
          //
          bool success_arm_plan;
          bool success_arm_move;
          bool success_grip;
          moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm; //arm plan
          moveit::planning_interface::MoveGroupInterface::Plan my_plan_grip; //grip plan
          auto current_pose = move_group_arm.getCurrentPose("arm_link_wr1"); //current pos
          //target pose
          auto target_pose = current_pose;
          target_pose.pose.position.x -= 0.3;
          target_pose.pose.position.y -= 0.3;
          target_pose.pose.position.z += 0.3;
          move_group_arm.setPoseTarget(target_pose);
          RCLCPP_INFO(LOGGER, "X pos is %f",current_pose.pose.position.x);
          RCLCPP_INFO(LOGGER, "Y pos is %f",current_pose.pose.position.y);
          RCLCPP_INFO(LOGGER, "Z pos is %f",current_pose.pose.position.z);

           //Get a pointer to the state of the current joints
          //const moveit::core::JointModelGroup *joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);


          // Get Current State
          move_group_arm.setStartStateToCurrentState();
          //Set the ready position
          //move_group_arm.setNamedTarget("ready");
          auto [success, plan] = [&move_group_arm]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_arm.plan(msg));
            return std::make_pair(ok, msg);
          }();
          //success_arm_plan = (move_group_arm.move(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          if(success){
            success_arm_move = (move_group_arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
          }
          else{
            RCLCPP_INFO(LOGGER, "Planning failed, no execution");
          }
          current_pose = move_group_arm.getCurrentPose("arm_link_wr1");
          RCLCPP_INFO(LOGGER, "X pos is %f",current_pose.pose.position.x);
          RCLCPP_INFO(LOGGER, "Y pos is %f",current_pose.pose.position.y);
          RCLCPP_INFO(LOGGER, "Z pos is %f",current_pose.pose.position.z);
          RCLCPP_INFO(LOGGER, "X orientation is %f",current_pose.pose.orientation.x);
          RCLCPP_INFO(LOGGER, "Y orientation is %f",current_pose.pose.orientation.y);
          RCLCPP_INFO(LOGGER, "Z orientation is %f",current_pose.pose.orientation.z);
          RCLCPP_INFO(LOGGER, "W orientation is %f",current_pose.pose.orientation.w);


          target_pose = current_pose;
          target_pose.pose.position.x += 0.3;
          target_pose.pose.position.y += 0.3;
          target_pose.pose.position.z -= 0.3;
          move_group_arm.setPoseTarget(target_pose);
          RCLCPP_INFO(LOGGER, "X pos is %f",current_pose.pose.position.x);
          RCLCPP_INFO(LOGGER, "Y pos is %f",current_pose.pose.position.y);
          RCLCPP_INFO(LOGGER, "Z pos is %f",current_pose.pose.position.z);

          // Get Current State
          move_group_arm.setStartStateToCurrentState();
          //Set the ready position
          //move_group_arm.setNamedTarget("ready");
          auto [success1, plan1] = [&move_group_arm]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_arm.plan(msg));
            return std::make_pair(ok, msg);
          }();
          //success_arm_plan = (move_group_arm.move(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          if(success1){
            success_arm_move = (move_group_arm.execute(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
          }
          else{
            RCLCPP_INFO(LOGGER, "Planning failed, no execution");
          }
          current_pose = move_group_arm.getCurrentPose("arm_link_wr1");
          RCLCPP_INFO(LOGGER, "X pos is %f",current_pose.pose.position.x);
          RCLCPP_INFO(LOGGER, "Y pos is %f",current_pose.pose.position.y);
          RCLCPP_INFO(LOGGER, "Z pos is %f",current_pose.pose.position.z);
          RCLCPP_INFO(LOGGER, "X orientation is %f",current_pose.pose.orientation.x);
          RCLCPP_INFO(LOGGER, "Y orientation is %f",current_pose.pose.orientation.y);
          RCLCPP_INFO(LOGGER, "Z orientation is %f",current_pose.pose.orientation.z);
          RCLCPP_INFO(LOGGER, "W orientation is %f",current_pose.pose.orientation.w);






/*
          auto target_pose = current_pose;
          target_pose.pose.position.x += 0.1;
          target_pose.pose.position.y += 0.2;
          move_group_arm.setPoseTarget(target_pose);
          move_group_arm.setStartStateToCurrentState();
          //Plan the motion
          success_arm_plan = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          //Execute the plan

          if(success_arm_plan){
            move_group_arm.execute(my_plan_arm);
            }
          else{
            RCLCPP_INFO(LOGGER, "Planning failed, no execution");
          }

*/

          //std::vector<double> joint_group_positions_arm;
          //current_state_arm->copyJointGroupPositions(joint_model_group_arm,joint_group_positions_arm);

          //move_group_arm.setStartStateToCurrentState();
          //moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;


         // RCLCPP_INFO(LOGGER, "Pregrasp Position");

        //  move_group_arm.setNamedTarget("home");
         // move_group_arm.setPoseTarget(target_pose);
        //Plan the motion
        // success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
        //Execute the plan
        /*
        if(success_arm){
          move_group_arm.execute(my_plan_arm);
        }
        else{
        RCLCPP_INFO(LOGGER, "Planning failed, no execution");
        }
        */
          rclcpp::shutdown();
          return 0;
        }