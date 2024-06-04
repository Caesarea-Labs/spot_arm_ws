#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "yolov8_msgs/msg/ws.hpp"
#include "std_msgs/msg/string.hpp"


//using namespace std::placeholders;
using std::placeholders::_1;

using moveit::planning_interface::MoveGroupInterface;

class Chip {
public:
    Chip(){
        detected = false;
        pose_detected = false;
        }
    bool detected;
    bool pose_detected;
    float x, y, ang_x, ang_y, ang_z;
    float key_x[4];
    float key_y[4];
};

class Socket{
    public:
        Socket(){
            detected = false;
            pose_detected = false;
            }
        bool detected;
        bool pose_detected;
        bool state;// True=open Flase=close
        float x, y, ang_x, ang_y, ang_z;
        float key_out_x[4];
        float key_out_y[4];
        float key_in_x[4];
        float key_in_y[4];
};



// Create the custom class
class MoveSpotArm : public rclcpp::Node {
    public:
        MoveSpotArm()
        : Node("move_spot_arm", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {
            cmd_subscription_ = this->create_subscription<std_msgs::msg::String>("cmd_move", 10, std::bind(&MoveSpotArm::move_by_cmd, this, _1));
            WS_est_sub = this->create_subscription<yolov8_msgs::msg::Ws>("Ws_res", 10, std::bind(&MoveSpotArm::ws_est, this, _1));
            RCLCPP_INFO(this->get_logger(), "Subscription created");

        }

//    private:
        void ws_est(const yolov8_msgs::msg::Ws::SharedPtr msg){
            if (msg) {
                if (msg->chip.detected){
                    RCLCPP_INFO(this->get_logger(), "Chip detected");
                    chip.x=msg->chip.c_x;
                    chip.y=msg->chip.c_y;
                    chip.ang_x = msg->chip.angs[0];
                    chip.ang_y = msg->chip.angs[1];
                    }
                if (msg->socket.detected){
                    RCLCPP_INFO(this->get_logger(), "Socket detected");
                    socket.x=msg->chip.c_x;
                    socket.y=msg->chip.c_y;
                    socket.ang_x = msg->socket.angs[0];
                    socket.ang_y = msg->socket.angs[1];
                    }
                else {
                    RCLCPP_WARN(this->get_logger(), "Received empty message");
                }
            }
        }

        void move_by_cmd(const std_msgs::msg::String::SharedPtr msg) {
            if (msg) {
                RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    //            move_group_arm.setStartStateToCurrentState();
    //            move_to_name_pos(group_arm, &arm_plan, "home");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Received empty message");
            }
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
        rclcpp::Subscription<yolov8_msgs::msg::Ws>::SharedPtr WS_est_sub;
        Chip chip;
        Socket socket;
        MoveGroupInterface* group;
};


bool move2pose(MoveGroupInterface* group, std::string pose, rclcpp::Logger logger){
    MoveGroupInterface::Plan plan;
    bool good;
    group->setStartStateToCurrentState();
    group->setNamedTarget(pose);
    good = static_cast<bool>(group->plan(plan));
    if(good)
    {
     good = (group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
     if (good){
       RCLCPP_INFO(logger,"Plan executed");
     }
     else{
        RCLCPP_ERROR(logger,"Failed to execute plan");
     }
     return good;
    }
    else{
    RCLCPP_ERROR(logger,"Failed to create plan");
    return good;
    }
}


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("move_arm",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_arm");
//  rclcpp::executors::SingleThreadedExecutor executor;
//  executor.add_node(node);
//  std::thread([&executor]() { executor.spin(); });
    rclcpp::spin(node);

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  // Defining the move groups interface
  auto m_g_i_arm = MoveGroupInterface(node, "spot_arm");
  auto m_g_i_g_f = MoveGroupInterface(node, "grip_f");
  auto m_g_i_g_l = MoveGroupInterface(node, "grip_l");
  auto m_g_i_g_r = MoveGroupInterface(node, "grip_r");
  // Define boollian vars for plan&exectue check
  // node->group = &m_g_i_arm;
  RCLCPP_INFO(logger, "Planning frame: %s", m_g_i_arm.getPlanningFrame().c_str());


  RCLCPP_INFO(logger, "Planning grops defined setting home positions");

  bool move1 = move2pose(&m_g_i_arm,"home",logger);
  move1 = move2pose(&m_g_i_g_f,"close",logger);
  move1 = move2pose(&m_g_i_g_r,"close",logger);
  move1 = move2pose(&m_g_i_g_l,"close",logger);

  move1 = move2pose(&m_g_i_arm,"scan_high",logger);
  move1 = move2pose(&m_g_i_g_f,"open",logger);
  move1 = move2pose(&m_g_i_g_r,"open",logger);
  move1 = move2pose(&m_g_i_g_l,"open",logger);


  rclcpp::shutdown();



  return 0;
}