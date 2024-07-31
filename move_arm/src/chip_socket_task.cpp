#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/string.hpp"
#include "yolov8_msgs/srv/move.hpp"
#include "yolov8_msgs/msg/ws.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>


using namespace std::placeholders;
using std::placeholders::_1;

using moveit::planning_interface::MoveGroupInterface;

using namespace std::literals::chrono_literals;

class WS_obj {
    public:
    WS_obj(){
    detected=false;
    state=false;
    world_detected=false;
    x_pic=0;
    y_pic=0;
    x_world=0;
    y_world=0;
    z_world=0;
    float theta =0;
    float key_points[2][4]={};
    }
    bool detected,world_detected;
    bool state;
    float x_pic, y_pic;
    float x_world,y_world, z_world, theta;
    float key_points[2][4];
    float joints_pos[9];
    geometry_msgs::msg::Point grip_pos;
    geometry_msgs::msg::Quaternion grip_orientation;
    private:

};


// Create the custom node class
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
            Joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CnS_Task_SpotArm::Log_Joints, this, _1));
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
            service_result_sc_ = false;
            request_sent_time = this->get_clock()->now().seconds();
            request_returned_time = this->get_clock()->now().seconds();
            request_sent_time_sc = this->get_clock()->now().seconds();
            request_returned_time_sc = this->get_clock()->now().seconds();
            request_sent_time_at = this->get_clock()->now().seconds();
            request_returned_time_at = this->get_clock()->now().seconds();
            TOF=0;
            // Creating the client for the suction cups
            sc1_client_ = this->create_client<std_srvs::srv::SetBool>("/custom_switch1", rmw_qos_profile_services_default, client_cb_group_);
            sc2_client_ = this->create_client<std_srvs::srv::SetBool>("/custom_switch2", rmw_qos_profile_services_default, client_cb_group_);
            sc3_client_ = this->create_client<std_srvs::srv::SetBool>("/custom_switch3", rmw_qos_profile_services_default, client_cb_group_);
            sc4_client_ = this->create_client<std_srvs::srv::SetBool>("/custom_switch4", rmw_qos_profile_services_default, client_cb_group_);
            // Creating the clients for link detacher--attacher
            attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK", rmw_qos_profile_services_default, client_cb_group_);
            detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK", rmw_qos_profile_services_default, client_cb_group_);
            // The publisher for the save node
            Img_save_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd_save", 10);
            spot_arm = false;
            grip_f = false;
            grip_l = false;
            grip_r = false;
            Print_status();
            Print_menu();



//            MoveGroupInterface arm_g = MoveGroupInterface(ptr, "spot_arm");
        }
    private:

        void Log_Joints(const sensor_msgs::msg::JointState::SharedPtr data){
            for(int i=0; i<9;i++){
                joints[i]=data->position[i];
            }
        }
        void Estimate(const yolov8_msgs::msg::Ws::SharedPtr data) {
            // This function updates the perception
            if (data->chip.detected){
//                RCLCPP_INFO(this->get_logger(),"Chip detected");
                chip.detected=true;
                chip.x_pic = data->chip.c_x;
                chip.y_pic = data->chip.c_y;
                for(int i=0; i<4 ;i++){
                    chip.key_points[0][i]=data->chip.key_x[i];
                    chip.key_points[1][i]=data->chip.key_y[i];
                }
            } else {
                chip.detected = false;
            }

            if (data->socket.detected){
//                RCLCPP_INFO(this->get_logger(),"Socket detected");
                socket.detected=true;
                socket.state = data->socket.state;
                socket.x_pic = data->socket.c_x;
                socket.y_pic = data->socket.c_y;
                for(int i=0; i<4 ;i++){
                    socket.key_points[0][i]=data->socket.key_x[i];
                    socket.key_points[1][i]=data->socket.key_y[i];
                }
            } else {
                socket.detected = false;
            }

            TOF = data->tof;
            Get_grip_pos();
        }
        void Get_grip_pos(){
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("body", "arm_link_wr1", tf2::TimePointZero);
            grip_pos.x = transform_stamped.transform.translation.x;
            grip_pos.y = transform_stamped.transform.translation.y;
            grip_pos.z = transform_stamped.transform.translation.z;
            grip_orientation = transform_stamped.transform.rotation;
        }
        void Move_cmd(const std_msgs::msg::String::SharedPtr data) {
            int mode = stoi(data->data);
//            RCLCPP_INFO(this->get_logger(),"mode %d selected",mode);
            if(mode==0){
                Print_status();
                Print_menu();
            }
            else if (mode==1){
                RCLCPP_INFO(this->get_logger(),"Starting the Start up process");
                Start_up();
            }
            else if (mode==2){
                RCLCPP_INFO(this->get_logger(),"Scanning the workspace");
                bool chip_d = Chip_scan();
                //Scan for the socket
                bool socket_d = Socket_scan();
                Go_Home();
            }else if(mode==3){
                RCLCPP_INFO(this->get_logger(),"Opening the socket");
                Open_Socket_Stage();
                //Pick
                RCLCPP_INFO(this->get_logger(),"Pick the chip");
                Pick_chip();
                //Move to the Socket
                RCLCPP_INFO(this->get_logger(),"Move to socket");
                Move_2_Pos(2);
                //Release the chip
                RCLCPP_INFO(this->get_logger(),"Setting chip into socket");
                Releas_chip();
                //Closing the socket
                RCLCPP_INFO(this->get_logger(),"Closing the socket");
                Detach_chip();
                Close_Socket_Chip_Stage();
                RCLCPP_INFO(this->get_logger(),"Returning to home position");
                Go_Home();

            }

        }
        void Print_menu(){
            RCLCPP_INFO(this->get_logger(),"0 - Print status an menu");
            RCLCPP_INFO(this->get_logger(),"1 - Start up motion controllers");
            RCLCPP_INFO(this->get_logger(),"2 - Scan workspace");
            RCLCPP_INFO(this->get_logger(),"3 - Load chip to socket");

        }
        void Print_status(){
            //start up
            if (spot_arm & grip_f & grip_l & grip_r){
                RCLCPP_INFO(this->get_logger(),"All controller loaded, ready to start working.");
            }else{
                RCLCPP_INFO(this->get_logger(),"Controller not loaded, start the startup process.");
            }
            //Scan workspace
            if(chip.world_detected & socket.world_detected){
                RCLCPP_INFO(this->get_logger(),"All elements in workspace are detected");
            }else if(!chip.world_detected & socket.world_detected) {
                RCLCPP_INFO(this->get_logger(),"Chip not detected");
            }else if(chip.world_detected & !socket.world_detected) {
                RCLCPP_INFO(this->get_logger(),"Socket not detected");
            }else {
                RCLCPP_INFO(this->get_logger(),"No items detected");
            }
        }
        void Go_Home(){
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->pose_name = "home";
            request->mode =0;
            bool good=false;
            int i=0;
            while(!good){

                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(1000ms);
                }

            }

        }
        void Close_Socket_Stage(){
            if(!socket.state){
                RCLCPP_INFO(this->get_logger(),"Socket is closed, Returning");
                return;
                }
            RCLCPP_INFO(this->get_logger(),"Closing the socket");
            Move_2_element(2,0.5);
            Operate_griper(-0.03,0.03);
            Scan_WS(2);
            Move_2_element(2,0.5);
            Align_orientation(2);
            Save_joints(2);
            double offset = -0.06;
            while(socket.state){
                Close_socket(offset);
                offset+=0.01;
                offset = std::min(offset,-0.04);
                Move_2_element(2,0.5);
                Operate_griper(-0.03,0.03);
                Scan_WS(2);
                Move_2_element(2,0.5);
                Align_orientation(2);
                Save_Pos(2);
                Save_joints(2);
                if(socket.state){
                    RCLCPP_INFO(this->get_logger(),"Socket is open");
                    RCLCPP_INFO(this->get_logger(),"offset distance is %f",offset);
                    }
                else{
                    RCLCPP_INFO(this->get_logger(),"Socket is closed");
                    RCLCPP_INFO(this->get_logger(),"offset distance is %f",offset);
                }
//
            }

        }
        void Close_Socket_Chip_Stage(){
            if(!socket.state){
                RCLCPP_INFO(this->get_logger(),"Socket is closed, Returning");
                return;
                }
            RCLCPP_INFO(this->get_logger(),"Closing the socket");
            Move_2_element(2,0.5);
            Operate_griper(-0.03,0.03);

            double offset = -0.06;
            while(socket.state){
                Close_socket(offset);
                offset+=0.01;
                offset = std::min(offset,-0.04);
                Move_2_element(2,0.5);
                Operate_griper(-0.03,0.03);
                Move_2_element(2,0.5);
            }

        }
        void Open_Socket_Stage(){
            if(socket.state){
                RCLCPP_INFO(this->get_logger(),"Socket is open, Returning");
            }
            Move_2_element(2,0.5);
            Operate_griper(-0.03,0.03);
            Scan_WS(2);
            Move_2_element(2,0.5);
            Align_orientation(2);
            Save_Pos(2);
            Save_joints(2);
            double delta=0.027;
            while(!socket.state){
                RCLCPP_INFO(this->get_logger(),"Attempting to open socket with delta= %f",delta);
                Open_socket(delta);
                delta *=0.9;
                delta = std::max(delta,0.018);
                Operate_griper(-0.03,0.03);
                Move_2_element(2,0.5);
                Scan_WS(2);
                Move_2_element(2,0.5);
                Align_orientation(2);
                Save_Pos(2);
                Save_joints(2);
                }
        }
        bool Chip_scan(){
            // SCANNING FOR CHIP
            RCLCPP_INFO(this->get_logger(),"Scanning for chip");
            chip.world_detected=false;
            Scan_WS(1);
            if(!chip.world_detected){
                RCLCPP_WARN(this->get_logger(),"Chip not detected");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"Move to chip");
            Move_2_element(1,0.4);
            Align_orientation(1);
            Save_joints(1);
            Save_Pos(1);
            return true;
        }
        bool Socket_scan(){
            // SCANNING FOR CHIP
            RCLCPP_INFO(this->get_logger(),"Scanning for socket");
            socket.world_detected=false;
            Scan_WS(2);
            if(!chip.world_detected){
                RCLCPP_WARN(this->get_logger(),"Socket not detected");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"Move to socket");
            Move_2_element(2,0.5);
            Align_orientation(2);
            Save_joints(2);
            Save_Pos(2);
            return true;
        }
        void Generate_images(const std::string str){
            Get_grip_pos();
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->mode =1;
            request->pose_t.position.x = grip_pos.x ;
            request->pose_t.position.y = grip_pos.y;
            request->pose_t.position.z = grip_pos.z; //First number in the correction TOF sensor->end_gripper
            auto grip_orientation_t = grip_orientation;
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::convert(grip_orientation_t, q_orig);
            q_rot.setRPY(0.0, 0.0,-0.5*3.14159);
            q_new = q_rot * q_orig;
            q_new.normalize();
            grip_orientation_t = tf2::toMsg(q_new);

            request->pose_t.orientation = grip_orientation_t;
            RCLCPP_INFO(this->get_logger(),"Sending initial  griper pose");
            bool good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }
            auto message = std_msgs::msg::String();

            //Creating the sequence
            for(int i=1;i<30;i++){
                Get_grip_pos();
                grip_orientation_t = grip_orientation;
                tf2::convert(grip_orientation_t, q_orig);
                q_rot.setRPY(0.0, 0.0,3.1/30);
                q_new = q_rot * q_orig;
                q_new.normalize();
                grip_orientation_t = tf2::toMsg(q_new);
                request->pose_t.orientation = grip_orientation_t;
                RCLCPP_INFO(this->get_logger(),"Sending %d pos",i);
                good=false;
                while (!good){
                    good = Send_request(request);
                    while(request_sent_time>=request_returned_time){
                        rclcpp::sleep_for(500ms);
                    }
                }
                message.data = str + std::to_string(i);
                Img_save_pub_->publish(message);

            }

        }
        void Start_up(){
            //Load spot_arm controller
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->pose_name = "home";
            request->mode =0;
            bool good=false;
            int i=0;
            while(!good){
                i++;
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(1000ms);
                }
                if(good || i>40) break;
            }
            if(good){
                spot_arm=true;
            }
            else{
                RCLCPP_ERROR(this->get_logger(),"spot_arm controller failed to load. It is recommended to relaunch the simulation");
                return;
            }

            //Load grip_f controller

            request->group = "grip_f";
            request->pose_name = "open";
            good=false;
            i=0;
            while (!good){
                i++;
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(1000ms);
                }
                if(good || i>40) break;
            }
            if(good){
                grip_f=true;
            }
            else{
                RCLCPP_ERROR(this->get_logger(),"grip_f controller failed to load. It is recommended to relaunch the simulation");
                return;
            }


            //Load grip_l loaded

            request->group = "grip_l";
            request->pose_name = "open";
            good=false;
            i=0;
            while (!good){
                i++;
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(1000ms);
                }
                if(good || i>40) break;
            }
            if(good){
                grip_l=true;
            }
            else{
                RCLCPP_ERROR(this->get_logger(),"grip_l controller failed to load. It is recommended to relaunch the simulation");
                return;
            }


            //Load grip_r controller

            request->group = "grip_r";
            request->pose_name = "open";
            good=false;
            i=0;
            while (!good){
                i++;
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(1000ms);
                }
                if(good || i>40) break;
            }
            if(good){
                grip_r=true;
            }
            else{
                RCLCPP_ERROR(this->get_logger(),"grip_r controller failed to load. It is recommended to relaunch the simulation");
                return;
            }

            RCLCPP_INFO(this->get_logger(),"All controllers loaded.");


        }
        void Scan_WS(const int element){
            auto base_pose="scan_low";
            // Initial check
            Estimate_world(element);
            if (Check_exit(element))
                return;
            Get_grip_pos();
            //Define the request frame
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            // *****Return to base pose
            request->group = "spot_arm";
            request->mode =0;
            request->pose_name = base_pose;
            // Return to the base position
            bool good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }

            Get_grip_pos();
            auto grip_orientation_t = grip_orientation;
            // Define the quaternion elements
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::convert(grip_orientation_t, q_orig);
            q_rot.setRPY(3.14159/10, 0.0, 0.0);
            q_new = q_rot * q_orig;
            q_new.normalize();
            grip_orientation_t = tf2::toMsg(q_new);

            // *****Moving the gripper left
            request->mode =1;
            request->pose_t.position.x = grip_pos.x;
            request->pose_t.position.y = grip_pos.y;
            request->pose_t.position.z = grip_pos.z;
            request->pose_t.orientation = grip_orientation_t;
//            RCLCPP_INFO(this->get_logger(),"Defined new position -Left");
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            Estimate_world(element);//estimate the chip location
            if (Check_exit(element))
                return;

            //******Return to base pose
//            request->mode =0;
//            request->pose_name = base_pose;
//            good=false;
//            while (!good){
//                good = Send_request(request);
//                while(request_sent_time>=request_returned_time){
//                rclcpp::sleep_for(500ms);
//                }
//            }
//            RCLCPP_INFO(this->get_logger(),"Returned to base pose");

            //***** Left and Up
            Get_grip_pos();
            grip_orientation_t = grip_orientation;
            tf2::convert(grip_orientation_t, q_orig);
//            q_rot.setRPY(3.14159/10, -3.14159/10, 0.0);
            q_rot.setRPY(0, -3.14159/10, 0.0);
            q_new = q_rot * q_orig;
            q_new.normalize();
            grip_orientation_t = tf2::toMsg(q_new);
            request->mode =1;
            request->pose_t.orientation = grip_orientation_t;
//            RCLCPP_INFO(this->get_logger(),"Defined new position -Left Up");
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            Estimate_world(element);
            if (Check_exit(element))
                return;

            // ****Return to base pose
//            request->mode =0;
//            request->pose_name = base_pose;
//            good=false;
//            while (!good){
//                good = Send_request(request);
//                while(request_sent_time>=request_returned_time){
//                rclcpp::sleep_for(500ms);
//                }
//            }
//            RCLCPP_INFO(this->get_logger(),"Returned to base pose");

            // *** Moving Right
            Get_grip_pos();
            grip_orientation_t = grip_orientation;
            tf2::convert(grip_orientation_t, q_orig);
//            q_rot.setRPY(-3.14159/10, 0.0, 0.0);
            q_rot.setRPY(-2*3.14159/10, 0.0, 0.0);
            q_new = q_rot * q_orig;
            q_new.normalize();
            grip_orientation_t = tf2::toMsg(q_new);
            request->mode=1;
            request->pose_t.orientation = grip_orientation_t;
//            RCLCPP_INFO(this->get_logger(),"Defined new position -Right");
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            Estimate_world(element);
            if (Check_exit(element))
                return;

            //***Return to base pose
//            request->mode =0;
//            request->pose_name = base_pose;
//            good=false;
//            while (!good){
//                good = Send_request(request);
//                while(request_sent_time>=request_returned_time){
//                rclcpp::sleep_for(500ms);
//                }
//            }
//            RCLCPP_INFO(this->get_logger(),"Returned to base pose");

            Get_grip_pos();
            grip_orientation_t = grip_orientation;
            tf2::convert(grip_orientation_t, q_orig);
//            q_rot.setRPY(-3.14159/10, -3.14159/10, 0.0);
            q_rot.setRPY(0, 3.14159/10, 0.0);
            q_new = q_rot * q_orig;
            q_new.normalize();
            grip_orientation_t = tf2::toMsg(q_new);
            request->mode=1;
            request->pose_t.orientation = grip_orientation_t;
//            RCLCPP_INFO(this->get_logger(),"Defined new position -Right");
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            Estimate_world(element);
            if (Check_exit(element))
                return;



            request->pose_name=base_pose;
            request->mode=0;
//            RCLCPP_INFO(this->get_logger(),"Defined new position -Base");
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                rclcpp::sleep_for(500ms);
                }
            }
            Get_grip_pos();

        }
        bool Send_request(const auto request){
            using ServiceResponseFuture = rclcpp::Client<yolov8_msgs::srv::Move>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                auto response = future.get();
                request_returned_time = this->get_clock()->now().seconds();
                if (response->success){
//                    RCLCPP_INFO(this->get_logger(), "DONE"); // Change this to your response field
                    }
                service_result_=response->success;
                request_returned_time = this->get_clock()->now().seconds();

            };
            request_sent_time = this->get_clock()->now().seconds();
            auto future = move_client_->async_send_request(request, response_received_callback);

            return service_result_;

        }
        void Estimate_world(const int element){
            tf2::Vector3 v2chip_pic, v2chip_world;
            Get_grip_pos();
            auto grip_orientation_t =grip_orientation;
            float ang_x, ang_y;
            bool good, cali;
            // Setting up the request
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->mode =1;
            request->pose_t.position.x = grip_pos.x;
            request->pose_t.position.y = grip_pos.y;
            request->pose_t.position.z = grip_pos.z;
            request->pose_t.orientation = grip_orientation_t;

            tf2::Quaternion q_orig, q_rot, q_new;
            if(element==1){
                if(!chip.detected) {return;}
                else {

                cali=false;
                while(!cali){

                if(element==1 & chip.detected){
//                    RCLCPP_INFO(this->get_logger(),"Correcting chip x");
                    chip.detected =false;
                    ang_x= (chip.x_pic -960) * 1.089 / 1920;
                    ang_y= (chip.y_pic -540) * 1.089 / 1920;
                    tf2::convert(grip_orientation_t, q_orig);
                    q_rot.setRPY(-ang_x, 0  , 0.0);
                    q_new = q_rot * q_orig;
                    q_new.normalize();
                    grip_orientation_t = tf2::toMsg(q_new);
                    request->pose_t.orientation = grip_orientation_t;
                    good=false;
                    while (!good){
                        good = Send_request(request);
                        while(request_sent_time>=request_returned_time){
                            rclcpp::sleep_for(500ms);
                            }
                        }
                    }
                rclcpp::sleep_for(1000ms);
                Get_grip_pos();
                grip_orientation_t =grip_orientation;
                request->pose_t.position.x = grip_pos.x;
                request->pose_t.position.y = grip_pos.y;
                request->pose_t.position.z = grip_pos.z;
                request->pose_t.orientation = grip_orientation_t;

                if(element==1 & chip.detected){
//                    RCLCPP_INFO(this->get_logger(),"Correcting chip y");
                    chip.detected =false;
                    ang_x= (chip.x_pic -960) * 1.089 / 1920;
                    ang_y= (chip.y_pic -540) * 1.089 / 1920;
                    tf2::convert(grip_orientation_t, q_orig);
                    q_rot.setRPY(0, ang_y  , 0.0);
                    q_new = q_rot * q_orig;
                    q_new.normalize();
                    grip_orientation_t = tf2::toMsg(q_new);
                    request->pose_t.orientation = grip_orientation_t;
                    good=false;
                    while (!good){
                            good = Send_request(request);
                            while(request_sent_time>=request_returned_time){
                                rclcpp::sleep_for(500ms);
                                }
                            }
                    }
                rclcpp::sleep_for(1000ms);
                Get_grip_pos();
                if (chip.detected){
                ang_x= (chip.x_pic -960) * 1.089 / 1920;
                ang_y= (chip.y_pic -540) * 1.089 / 1920;
                if(abs(ang_x)<0.02 & abs(ang_y)<0.02){
                cali=true;
//                RCLCPP_INFO(this->get_logger(),"TOF= %2f",TOF);
                v2chip_pic.setX(TOF+ 0.11778 + 0.151);
                v2chip_pic.setY(0);
                v2chip_pic.setZ(0+0.02);
                tf2::convert(grip_orientation_t, q_orig);
                v2chip_world = quatRotate(q_orig, v2chip_pic);
                chip.x_world = v2chip_world.getX() + grip_pos.x;
                chip.y_world = v2chip_world.getY() + grip_pos.y;
                chip.z_world = v2chip_world.getZ() + grip_pos.z;
                chip.world_detected = true;
//                RCLCPP_INFO(this->get_logger(),"Chip position WRT grip %2f ,%2f ,%2f",v2chip_world.getX(),v2chip_world.getY(),v2chip_world.getZ());
//                RCLCPP_INFO(this->get_logger(),"Chip position %2f ,%2f ,%2f",chip.x_world,chip.y_world,chip.z_world);



                }
                } else {
                return;}
                }
                }
            }

             if(element==2){
                if(!socket.detected) return;
                else {
                cali=false;
                while(!cali){

                if(element==2 & socket.detected){
//                    RCLCPP_INFO(this->get_logger(),"Correcting socket x");
                    socket.detected =false;
                    ang_x= (socket.x_pic -960) * 1.089 / 1920;
                    ang_y= (socket.y_pic -540) * 1.089 / 1920;
                    tf2::convert(grip_orientation_t, q_orig);
                    q_rot.setRPY(-ang_x, 0  , 0.0);
                    q_new = q_rot * q_orig;
                    q_new.normalize();
                    grip_orientation_t = tf2::toMsg(q_new);
                    request->pose_t.orientation = grip_orientation_t;
                    good=false;
                    while (!good){
                        good = Send_request(request);
                        while(request_sent_time>=request_returned_time){
                            rclcpp::sleep_for(500ms);
                            }
                        }
                    }
                rclcpp::sleep_for(1000ms);
                Get_grip_pos();
                grip_orientation_t =grip_orientation;
                request->pose_t.position.x = grip_pos.x;
                request->pose_t.position.y = grip_pos.y;
                request->pose_t.position.z = grip_pos.z;
                request->pose_t.orientation = grip_orientation_t;

                if(element==2 & socket.detected){
//                    RCLCPP_INFO(this->get_logger(),"Correcting socket y");
                    socket.detected =false;
                    ang_x= (socket.x_pic -960) * 1.089 / 1920;
                    ang_y= (socket.y_pic -540) * 1.089 / 1920;
                    tf2::convert(grip_orientation_t, q_orig);
                    q_rot.setRPY(0, ang_y  , 0.0);
                    q_new = q_rot * q_orig;
                    q_new.normalize();
                    grip_orientation_t = tf2::toMsg(q_new);
                    request->pose_t.orientation = grip_orientation_t;
                    good=false;
                    while (!good){
                            good = Send_request(request);
                            while(request_sent_time>=request_returned_time){
                                rclcpp::sleep_for(500ms);
                                }
                            }
                    }
                rclcpp::sleep_for(1000ms);
                Get_grip_pos();
                if (socket.detected){
                ang_x= (socket.x_pic -960) * 1.089 / 1920;
                ang_y= (socket.y_pic -540) * 1.089 / 1920;
                if(abs(ang_x)<0.02 & abs(ang_y)<0.02){
                cali=true;
//                RCLCPP_INFO(this->get_logger(),"TOF= %2f",TOF);
                v2chip_pic.setX(TOF+ 0.11778 + 0.151);
                v2chip_pic.setY(0);
                v2chip_pic.setZ(0+0.02);
                tf2::convert(grip_orientation_t, q_orig);
                v2chip_world = quatRotate(q_orig, v2chip_pic);
                socket.x_world = v2chip_world.getX() + grip_pos.x;
                socket.y_world = v2chip_world.getY() + grip_pos.y;
                socket.z_world = v2chip_world.getZ() + grip_pos.z;
                socket.world_detected = true;
//                RCLCPP_INFO(this->get_logger(),"Socket position WRT grip %2f ,%2f ,%2f",v2chip_world.getX(),v2chip_world.getY(),v2chip_world.getZ());
//                RCLCPP_INFO(this->get_logger(),"Socket position %2f ,%2f ,%2f",chip.x_world,chip.y_world,chip.z_world);



                }
                } else {
                return;}
                }
                }
             }
            }
        void Move_2_element(const int element, float h){
            float x,y,z;
            bool good;
            if (element==1){
                x = chip.x_world;
                y = chip.y_world;
                z = chip.z_world;
            } else if(element==2){
                x = socket.x_world;
                y = socket.y_world;
                z = socket.z_world;
                }
            Get_grip_pos();
            auto grip_orientation_t = grip_orientation;
            tf2::Quaternion q_orig, q_rot, q_new;
//            tf2::convert(grip_orientation_t, q_orig);
            q_rot.setRPY(0.0, 3.14159/2, 0.0);
//            q_new = q_rot * q_orig;
//            q_new.normalize();
            grip_orientation_t = tf2::toMsg(q_rot);

            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->mode =1;
            request->pose_t.position.x = x ;
            request->pose_t.position.y = y;
            request->pose_t.position.z = z + h ;//First number in the correction TOF sensor->end_gripper
            request->pose_t.orientation = grip_orientation_t;
//            RCLCPP_INFO(this->get_logger(),"Aligning griper with element");
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }
            }
        bool Check_exit(const int element){
            if(element==1){
                if(chip.world_detected){
                    return true;
                }
            }
            else if(element==2){
                if(socket.world_detected){
                    return true;
                }
            }
            return false;
        }
        void Align_orientation(const int element){
            double theta=0;
            bool good=false;
            if(element==1){
                theta = atan2(chip.key_points[1][1] - chip.key_points[1][0], chip.key_points[0][1] - chip.key_points[0][0]);
            } else if (element==2){
                theta = atan2(socket.key_points[1][1] - socket.key_points[1][0], socket.key_points[0][1] - socket.key_points[0][0]);
            }
            theta -= 3.14159/2;
            Get_grip_pos();
            auto grip_orientation_t = grip_orientation;
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::convert(grip_orientation_t, q_orig);
//            RCLCPP_INFO(this->get_logger(),"Theta angel is %2f",theta);
            q_rot.setRPY(0.0, 0.0, theta);//Define the needed rotation to allign with element
            q_new = q_rot * q_orig; //Computing the new orientation
            q_new.normalize();
            grip_orientation_t = tf2::toMsg(q_new);//Moving to message format
            //Define the request
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->mode =1;
            request->pose_t.position.x = grip_pos.x ;
            request->pose_t.position.y = grip_pos.y;
            request->pose_t.position.z = grip_pos.z;//First number in the correction TOF sensor->end_gripper
            request->pose_t.orientation = grip_orientation_t;
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }
            rclcpp::sleep_for(1000ms);
//            RCLCPP_INFO(this->get_logger(),"New detection");

             if(element==1){
                theta = atan2(chip.key_points[1][1] - chip.key_points[1][0], chip.key_points[0][1] - chip.key_points[0][0]);
            } else if (element==2){
                theta = atan2(socket.key_points[1][1] - socket.key_points[1][0], socket.key_points[0][1] - socket.key_points[0][0]);
            }
            theta -= 3.14159/2;
//            RCLCPP_INFO(this->get_logger(),"The corrected angle is %f",theta);
            if (abs(theta)>0.03){
                Align_orientation(element);
                }



        }
        void Save_joints(const int element){

            if(element==1){//chip
                for(int i=0; i<9; i++){
                    chip.joints_pos[i]=joints[i];
//                    RCLCPP_INFO(this->get_logger(),"Joint %d pos %f",i,chip.joints_pos[i]);
                }
            }else if(element==2){
                for(int i=0; i<9;i++){
                    socket.joints_pos[i]=joints[i];
                }
                }
//                RCLCPP_INFO(this->get_logger(),"Position saved");
            }
        void Save_Pos(const int element){
            Get_grip_pos();
            if(element==1){//chip
                chip.grip_pos = grip_pos;
                chip.grip_orientation = grip_orientation;
            }else if(element==2){
                socket.grip_pos = grip_pos;
                socket.grip_orientation = grip_orientation;
                }
//                RCLCPP_INFO(this->get_logger(),"Position saved");
            }
        void Move_2_joints(const int element){
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->mode = 3;
            if(element==1){
                for(int i=0; i<6; i++){
                    request->joint[i]=chip.joints_pos[i];
                }
            }else if(element==2){
                for(int i=0; i<6; i++){
                    request->joint[i]=socket.joints_pos[i];
                    }
                }
//            RCLCPP_INFO(this->get_logger(),"Sending Joint request");
            bool good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }
        }//Move_2_Joints
        void Move_2_Pos(const int element){
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->mode = 1;
            if (element==1){
                request->pose_t.position = chip.grip_pos;
                request->pose_t.orientation = chip.grip_orientation;
            }else if (element==2){
                request->pose_t.position = socket.grip_pos;
                request->pose_t.orientation = socket.grip_orientation;
            }
            bool good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }
        }//Move_2_Joints
        void Activate_SC(){
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;
            bool good=false;
            while (!good){
                good = Send_request_SC(request);
                while(request_sent_time_sc>=request_returned_time_sc){
                    rclcpp::sleep_for(500ms);
                    }
                }
        }
        void DeActivate_SC(){
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = false;
            bool good=false;
            while (!good){
                good = Send_request_SC(request);
                while(request_sent_time_sc>=request_returned_time_sc){
                    rclcpp::sleep_for(500ms);
                    }
                }
        }
        bool Send_request_SC(const auto request){
            using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                auto response = future.get();
                request_returned_time_sc = this->get_clock()->now().seconds();
                if (response->success){
//                    RCLCPP_INFO(this->get_logger(), "DONE"); // Change this to your response field
                    }
                service_result_sc_=response->success;
                request_returned_time_sc = this->get_clock()->now().seconds();

            };
            request_sent_time = this->get_clock()->now().seconds();
//            RCLCPP_INFO(this->get_logger(),"Sending SC1");
            auto future1 = sc1_client_->async_send_request(request, response_received_callback);
//            RCLCPP_INFO(this->get_logger(),"Sending SC2");
            auto future2 = sc2_client_->async_send_request(request, response_received_callback);
//            RCLCPP_INFO(this->get_logger(),"Sending SC3");
            auto future3 = sc3_client_->async_send_request(request, response_received_callback);
//            RCLCPP_INFO(this->get_logger(),"Sending SC4");
            auto future4 = sc4_client_->async_send_request(request, response_received_callback);

            return service_result_;

        }
        void Attach(){
            auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
            request->model1_name = "spot";
            request->link1_name = "left_finger_link";
            request->model2_name = "Chip";
            request->link2_name = "link_7";
            bool good = Send_request_AT(request);
//            bool good=false;
//            while (!good){
//                good = Send_request_AT(request);
//                while(request_sent_time_at>=request_returned_time_at){
//                    rclcpp::sleep_for(500ms);
//                    }
//                }
        }
        void Fix_socket(){
            auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
            request->model1_name = "Table";
            request->link1_name = "link";
            request->model2_name = "socket";
            request->link2_name = "pbc";
            bool good=false;
            while (!good){
                good = Send_request_AT(request);
                while(request_sent_time_at>=request_returned_time_at){
                    rclcpp::sleep_for(500ms);
                    }
                }
        }
        bool Send_request_AT(const auto request){
            using ServiceResponseFuture = rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                auto response = future.get();
                request_returned_time_at = this->get_clock()->now().seconds();
                if (response->success){
//                    RCLCPP_INFO(this->get_logger(), "DONE"); // Change this to your response field
                    }
                service_result_at_=response->success;
                request_returned_time_at = this->get_clock()->now().seconds();

            };
            request_sent_time_at = this->get_clock()->now().seconds();
//            RCLCPP_INFO(this->get_logger(),"Sending Attach");
            auto future = attach_client_->async_send_request(request, response_received_callback);


            return service_result_at_;

        }
        void Detach(){
            auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
            request->model1_name = "spot";
            request->link1_name = "left_finger_link";
            request->model2_name = "Chip";
            request->link2_name = "link_7";
            bool good = Send_request_De(request);
//            bool good=false;
//            while (!good){
//                good = Send_request_De(request);
//                while(request_sent_time_at>=request_returned_time_at){
//                    rclcpp::sleep_for(500ms);
//                    }
//                }
        }
        bool Send_request_De(const auto request){
            using ServiceResponseFuture = rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                auto response = future.get();
                request_returned_time_at = this->get_clock()->now().seconds();
                if (response->success){
//                    RCLCPP_INFO(this->get_logger(), "DONE"); // Change this to your response field
                    }
                service_result_at_=response->success;
                request_returned_time_at = this->get_clock()->now().seconds();

            };
            request_sent_time_at = this->get_clock()->now().seconds();
//            RCLCPP_INFO(this->get_logger(),"Sending Detach");
            auto future = detach_client_->async_send_request(request, response_received_callback);


            return service_result_at_;

        }
        void Pick_chip(){
            Move_2_element(1,0.35);//move to element 1 with higth  0.3
            double delta=0.02;
            Align_orientation(1); // Allign orientation with chip
            while(TOF>delta){
                rclcpp::sleep_for(200ms);
                Get_grip_pos();
//                RCLCPP_INFO(this->get_logger(),"TOF = %f",TOF);
                Move_direction(0,0,-0.5*(TOF-delta));
            }
            Activate_SC(); // Activate the suction cups
            rclcpp::sleep_for(500ms);
            Operate_griper(-0.02,0.02);//
            //Sending detach massage
           // Detach();
            //Sending attach massage
            Attach();
        }
        void Releas_chip(){
            Get_grip_pos();
            auto grip_orientation_t = grip_orientation;
            tf2::Vector3 v1, v2;
            v1.setX(0.035);
            v1.setY(0.0);
            v1.setZ(0.005);
            tf2::Quaternion q_orig;
            tf2::convert(grip_orientation_t, q_orig);
            v2 = quatRotate(q_orig, v1);
            //Move griper to front of socket
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
            double delta=0.02;
            while(TOF>delta){
                rclcpp::sleep_for(200ms);
                Get_grip_pos();
//                RCLCPP_INFO(this->get_logger(),"TOF = %f",TOF);
                Move_direction(0,0,-0.5*(TOF-delta));
            }
            Detach();
            Operate_griper(-0.03,0.03);//Open gripper
            DeActivate_SC(); // deActivate the suction cups
            //Sending detach massage
            rclcpp::sleep_for(1000ms);
            Attach_chip();

        }
        void Operate_griper(double l, double r){
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "grip_l";
            request->mode = 3;
            request->joint[0]=l;

            RCLCPP_INFO(this->get_logger(),"Sending Joint request grip_l");
            bool good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }
            request->group = "grip_r";
            request->mode = 3;
            request->joint[0]=r;

            RCLCPP_INFO(this->get_logger(),"Sending Joint request grip_r");
            good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }

        }
        void Open_socket(double delta){
            if (socket.state) return;
//            Move_2_element(2,0.3);
//            Align_orientation(2);
            Get_grip_pos();
            auto grip_orientation_t = grip_orientation;
            tf2::Vector3 v1, v2;
            v1.setX(0);
            v1.setY(0);
            v1.setZ(-0.07);
            tf2::Quaternion q_orig;
            tf2::convert(grip_orientation_t, q_orig);
            v2 = quatRotate(q_orig, v1);
            //Move griper to front of socket
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
            //Move to table lavel
            Get_grip_pos();

            while(TOF>delta){
                rclcpp::sleep_for(200ms);
                Get_grip_pos();
//                RCLCPP_INFO(this->get_logger(),"TOF = %f",TOF);
                Move_direction(0,0,-0.5*(TOF-0.8*delta));
            }

            v1.setX(0.0);
            v1.setY(-0.07);
            v1.setZ(0.);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());

            //open socket
            Operate_griper(0,0);
            v1.setZ(0.023);
            v1.setY(0.000);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//            Open the pressurebar lock
//            v1.setX(0.0);
//            v1.setY(-0.015);
//            v1.setZ(0.);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//
//
            //Move pressurebar up
            v1.setX(-0.05);
            v1.setY(0.0);
            v1.setZ(0.05);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());


//            //reset to prevent locking of pressurbar
//            v1.setX(0.0);
//            v1.setY(0.015);
//            v1.setZ(0.);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
            v1.setX(-0.04);
            v1.setY(0.);
            v1.setZ(0.04);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());

            v1.setX(-0.04);
            v1.setY(0.0);
            v1.setZ(0.0);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//            v1.setX(0.0);
//            v1.setY(0.012);
//            v1.setZ(0.);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//            v1.setX(-0.01);
//            v1.setY(0.);
//            v1.setZ(0.01);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());


        }
        void Move_direction(double x,double y,double z){
//            RCLCPP_INFO(this->get_logger(),"x= %f, y=%f, z=%f",x,y,z);
            Get_grip_pos();
            auto request = std::make_shared<yolov8_msgs::srv::Move::Request>();
            request->group = "spot_arm";
            request->mode = 1;
            request->pose_t.position.x = grip_pos.x +x;
            request->pose_t.position.y = grip_pos.y +y;
            request->pose_t.position.z = grip_pos.z +z;
            request->pose_t.orientation = grip_orientation;
            bool good=false;
            while (!good){
                good = Send_request(request);
                while(request_sent_time>=request_returned_time){
                    rclcpp::sleep_for(500ms);
                    }
                }
        }
        void Close_socket(double offset){

            Get_grip_pos();
            auto grip_orientation_t = grip_orientation;
            tf2::Vector3 v1, v2;

            tf2::Quaternion q_orig;
            tf2::convert(grip_orientation_t, q_orig);


            Operate_griper(0,0);
            v1.setX(0);
            v1.setY(offset);
            v1.setZ(0.15);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
            double delta=0.04;
            while(TOF>delta){
                rclcpp::sleep_for(200ms);
                Get_grip_pos();
//                RCLCPP_INFO(this->get_logger(),"TOF = %f",TOF);
                Move_direction(0,0,-0.5*(TOF-0.8*delta));
            }
            v1.setX(0);
            v1.setY(0);
            v1.setZ(-0.08);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());

            Move_direction(0,0,0.02);
            v1.setX(0);
            v1.setY(0);
            v1.setZ(-0.05);
            v2 = quatRotate(q_orig, v1);
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//
//
//
//
//            v1.setX(0);
//            v1.setY(0);
//            v1.setZ(-0.02);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//            Move_direction(0,0,0.01);
//
//            v1.setX(0);
//            v1.setY(0);
//            v1.setZ(-0.02);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//            Move_direction(0,0,0.01);
//
//            v1.setX(0);
//            v1.setY(0);
//            v1.setZ(-0.03);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//            v1.setX(0);
//            v1.setY(0.02);
//            v1.setZ(0.0);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());
//
//             Move_direction(0,0,-0.01);
//
//            v1.setX(0);
//            v1.setY(0);
//            v1.setZ(-0.03);
//            v2 = quatRotate(q_orig, v1);
//            Move_direction(v2.getX(),v2.getY(),v2.getZ());



        }
        void Attach_chip(){
            auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
            request->model1_name = "socket";
            request->link1_name = "bracket";
            request->model2_name = "Chip";
            request->link2_name = "link_7";
            bool good = Send_request_AT(request);
        }
        void Detach_chip(){
            auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
            request->model1_name = "socket";
            request->link1_name = "bracket";
            request->model2_name = "Chip";
            request->link2_name = "link_7";
            bool good = Send_request_De(request);
        }


        rclcpp::Subscription<yolov8_msgs::msg::Ws>::SharedPtr Perception_sub_;//subscription to the perception node
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joints_sub_;//subscription to the perception node
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Cmd_sub_;// This (Cmd_sub_) is only for testing
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Img_save_pub_;// This (save_sub_) is only for the data collection
        rclcpp::Node::SharedPtr ptr;
        rclcpp::Client<yolov8_msgs::srv::Move>::SharedPtr move_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sc1_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sc2_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sc3_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sc4_client_;
        rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
        rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::Point grip_pos;
        geometry_msgs::msg::Quaternion grip_orientation;
        bool service_result_, service_result_sc_,service_result_at_,service_result_de_;
        double request_sent_time, request_returned_time ;
        double request_sent_time_sc, request_returned_time_sc ;
        double request_sent_time_at, request_returned_time_at ;
        double request_sent_time_de, request_returned_time_de ;
        double TOF;
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr member_cb_group_;
        WS_obj chip, socket;
        double joints[9];
        bool spot_arm, grip_f, grip_l, grip_r;

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

