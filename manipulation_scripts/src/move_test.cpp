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
          auto move_group_node = rclcpp::Node::make_shared("move_group_interface_spot_arm", node_options);
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

          // Pointer to the current states
          moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
          moveit::core::RobotStatePtr current_state_gripper = move_group_grip.getCurrentState(10);

          // position of the arm's end effector
          auto current_pose = move_group_arm.getCurrentPose("arm_link_wr1"); //current pos
          RCLCPP_INFO(LOGGER, "X pos is %f",current_pose.pose.position.x);
          RCLCPP_INFO(LOGGER, "Y pos is %f",current_pose.pose.position.y);
          RCLCPP_INFO(LOGGER, "Z pos is %f",current_pose.pose.position.z);

          //target pose
          //auto target_pose = current_pose;
          //Set the current robot state to the start state
          move_group_arm.setStartStateToCurrentState();
          move_group_grip.setStartStateToCurrentState();

          //Get a pointer to the state of the current joints
          //const moveit::core::JointModelGroup *joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
          // Get Current State
          move_group_arm.setStartStateToCurrentState();
          //Set the ready position
          move_group_arm.setNamedTarget("home");
          success_arm_plan = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          //success_arm_plan = (move_group_arm.move(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          if(success_arm_plan){
            success_arm_move = (move_group_arm.execute(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          }
          else{
            RCLCPP_INFO(LOGGER, "Planning failed, no execution");
          }



          // Get Current State
          move_group_arm.setStartStateToCurrentState();
          //Set the ready position
          move_group_arm.setNamedTarget("scan_table");
          success_arm_plan = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          //success_arm_plan = (move_group_arm.move(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          if(success_arm_plan){
            success_arm_move = (move_group_arm.execute(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
          }
          else{
            RCLCPP_INFO(LOGGER, "Planning failed, no execution");
          }

          current_pose = move_group_arm.getCurrentPose("arm_link_wr1");
          RCLCPP_INFO(LOGGER, "X pos is %f",current_pose.pose.position.x);
          RCLCPP_INFO(LOGGER, "Y pos is %f",current_pose.pose.position.y);
          RCLCPP_INFO(LOGGER, "Z pos is %f",current_pose.pose.position.z);

          move_group_grip.setNamedTarget("open");
          success_grip = (move_group_grip.plan(my_plan_grip) == moveit::core::MoveItErrorCode::SUCCESS);
          move_group_grip.execute(my_plan_grip);



          geometry_msgs::msg::Pose target_pose;
          target_pose.position.x = current_pose.pose.position.x;
          target_pose.position.y = current_pose.pose.position.y;
          target_pose.position.z = current_pose.pose.position.z;
          std::vector<geometry_msgs::msg::Pose> scan_waypoints;
          double delta=0.1;
          target_pose.position.y -= delta;
          target_pose.position.z -= delta;
          //target_pose.orientation.y = -1.5/2;
          scan_waypoints.push_back(target_pose);
          target_pose.position.z += 2*delta;
          scan_waypoints.push_back(target_pose);
          target_pose.position.y += 2*delta;
          scan_waypoints.push_back(target_pose);
          target_pose.position.z -= 2*delta;
          scan_waypoints.push_back(target_pose);
          moveit_msgs::msg::RobotTrajectory trajectory_approach;
          const double jump_threshold = 0.0;
          const double eef_step = 0.01;
          move_group_arm.setStartStateToCurrentState();
          double fraction = move_group_arm.computeCartesianPath(scan_waypoints, eef_step, jump_threshold, trajectory_approach);
          move_group_arm.execute(trajectory_approach);

          move_group_arm.setStartStateToCurrentState();
          //Set the ready position
          move_group_arm.setNamedTarget("scan_table");
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

          move_group_grip.setNamedTarget("closed");
          success_grip = (move_group_grip.plan(my_plan_grip) == moveit::core::MoveItErrorCode::SUCCESS);
          move_group_grip.execute(my_plan_grip);

          rclcpp::shutdown();
          return 0;
        }