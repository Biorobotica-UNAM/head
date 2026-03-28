#include <algorithm>
#include <memory>
#include <chrono>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "head_interfaces/srv/move_head.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define MX_PRESENT_POSITION 36
#define MX_GOAL_POSITION    30
#define MX_MOVING_SPEED     32
#define MX_TORQUE_ENABLE    24
#define MX_TORQUE_LIMIT     34

#define TORQUE_LIMIT_BITS 1000

class HeadNode : public rclcpp::Node {
public:

    HeadNode() : Node("head_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing head_node...");

        if(!declare_and_get_parameters() || !initialize_dynamixel())
        {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed");
            rclcpp::shutdown();
            return;
        }

        setup_interfaces();
        initialize_messages();

        timer_ = this->create_wall_timer(
            20ms, std::bind(&HeadNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Head node ready");
    }

    ~HeadNode()
    {
        if(port_handler_ != nullptr)
            port_handler_->closePort();
    }

private:

    // ROS
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_pose_;
    rclcpp::Service<head_interfaces::srv::MoveHead>::SharedPtr srv_move_;

    // Dynamixel
    dynamixel::PortHandler* port_handler_ = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;

    // Parameters
    std::string port_name_;
    int baudrate_;
    bool torque_enable_;
    int max_speed_;
	std::string joint_states_topic_;
	std::string current_pose_topic_;
	std::string service_name_;
    bool simulation_;

    std::vector<int> servo_ids_;
    std::vector<double> servo_bits_per_radian_;
    std::vector<int> servo_zeros_;
    std::vector<int> servo_directions_;
    std::vector<int> servo_min_limits_;
    std::vector<int> servo_max_limits_;

    size_t num_servos_;
    std::vector<std::string> joint_names_;

    // State
    std::vector<int> current_position_bits_;
    std::vector<int> goal_pose_bits_;

    // Messages
    sensor_msgs::msg::JointState joint_states_;
    std_msgs::msg::Float64MultiArray msg_current_pose_;

    uint8_t dxl_error = 0;

    // =========================
    // PARAMETERS
    // =========================
    bool declare_and_get_parameters()
    {
        this->declare_parameter<bool>("simulation", false);

        this->declare_parameter<std::string>("port", "");
        this->declare_parameter<int>("baudrate", 1000000);
        this->declare_parameter<bool>("torque_enable", true);
        this->declare_parameter<int>("max_speed", 20);
        
		this->declare_parameter<std::string>("joint_states_topic", "joint_states");
		this->declare_parameter<std::string>("current_pose_topic", "current_pose");
		this->declare_parameter<std::string>("head_controller_service", "move_head");

        this->declare_parameter<std::vector<int64_t>>("servo_ids", std::vector<int64_t>{});
        this->declare_parameter<std::vector<double>>("servo_bits_per_radian", std::vector<double>{});
        this->declare_parameter<std::vector<int64_t>>("servo_zeros", std::vector<int64_t>{});
        this->declare_parameter<std::vector<int64_t>>("servo_directions", std::vector<int64_t>{});
        this->declare_parameter<std::vector<int64_t>>("servo_min_limits", std::vector<int64_t>{});
        this->declare_parameter<std::vector<int64_t>>("servo_max_limits", std::vector<int64_t>{});
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});

        simulation_ = this->get_parameter("simulation").as_bool();

        port_name_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        torque_enable_ = this->get_parameter("torque_enable").as_bool();
        max_speed_ = this->get_parameter("max_speed").as_int();

        joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
		current_pose_topic_ = this->get_parameter("current_pose_topic").as_string();
		service_name_ = this->get_parameter("head_controller_service").as_string();

        auto ids = this->get_parameter("servo_ids").as_integer_array();
        auto zeros = this->get_parameter("servo_zeros").as_integer_array();
        auto dirs = this->get_parameter("servo_directions").as_integer_array();
        auto min_l = this->get_parameter("servo_min_limits").as_integer_array();
        auto max_l = this->get_parameter("servo_max_limits").as_integer_array();

		joint_names_ = this->get_parameter("joint_names").as_string_array();

        servo_bits_per_radian_ = this->get_parameter("servo_bits_per_radian").as_double_array();

        num_servos_ = ids.size();

        if(num_servos_ == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "No servos configured");
            return false;
        }

        for(auto v : ids) servo_ids_.push_back((int)v);
        for(auto v : zeros) servo_zeros_.push_back((int)v);
        for(auto v : dirs) servo_directions_.push_back((int)v);
        for(auto v : min_l) servo_min_limits_.push_back((int)v);
        for(auto v : max_l) servo_max_limits_.push_back((int)v);

        return true;
    }

    // =========================
    // INIT
    // =========================
    bool initialize_dynamixel()
    {
        if(simulation_)
        {
            RCLCPP_WARN(this->get_logger(), "Running in SIMULATION mode");

            current_position_bits_.resize(num_servos_, 2048);
            goal_pose_bits_.resize(num_servos_, 2048);

            return true;
        }

        if(port_name_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Port not specified");
            return false;
        }

        port_handler_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(1.0);

        if (!port_handler_->openPort())
            return false;

        if (!port_handler_->setBaudRate(baudrate_))
            return false;

        current_position_bits_.resize(num_servos_);
        read_current_positions();
        goal_pose_bits_ = current_position_bits_;

        for(size_t i = 0; i < num_servos_; i++)
        {
            packet_handler_->write2ByteTxOnly(
                port_handler_, servo_ids_[i], MX_MOVING_SPEED, max_speed_);

            packet_handler_->write1ByteTxRx(
                port_handler_, servo_ids_[i], MX_TORQUE_ENABLE, torque_enable_, &dxl_error);

            packet_handler_->write2ByteTxRx(
                port_handler_, servo_ids_[i], MX_TORQUE_LIMIT, TORQUE_LIMIT_BITS, &dxl_error);
        }

        return true;
    }

    // =========================
    // ROS INTERFACES
    // =========================
    void setup_interfaces()
    {
        pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
    	pub_current_pose_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(current_pose_topic_, 10);
    	srv_move_ = this->create_service<head_interfaces::srv::MoveHead>(service_name_, std::bind(&HeadNode::move_service, this, _1, _2));
    }

    void initialize_messages()
    {
        msg_current_pose_.data.resize(num_servos_);
        joint_states_.name = joint_names_;
    }

    // =========================
    // SERVICE
    // =========================
    void move_service(
        const std::shared_ptr<head_interfaces::srv::MoveHead::Request> request,
        std::shared_ptr<head_interfaces::srv::MoveHead::Response> response)
    {
        if(request->positions.size() != num_servos_)
        {
            response->success = false;
            response->message = "Invalid joint size";
            return;
        }

        for(size_t i = 0; i < num_servos_; i++)
        {
            int raw = servo_directions_[i] * request->positions[i] *
                      servo_bits_per_radian_[i] + servo_zeros_[i];

            goal_pose_bits_[i] = std::clamp(raw,
                                           servo_min_limits_[i],
                                           servo_max_limits_[i]);
        }

        write_goal_positions();

        response->success = true;
        response->message = "Command sent";
    }

    // =========================
    // TIMER
    // =========================
    void timer_callback()
    {
        read_current_positions();

        joint_states_.header.stamp = this->now();
        joint_states_.position = bits_to_radians(current_position_bits_);

        pub_joint_state_->publish(joint_states_);

        for(size_t i = 0; i < num_servos_; i++)
            msg_current_pose_.data[i] = joint_states_.position[i];

        pub_current_pose_->publish(msg_current_pose_);
    }

    // =========================
    // HARDWARE
    // =========================
    bool read_current_positions()
    {
        if(simulation_)
            return true;

        for(size_t i = 0; i < num_servos_; i++)
        {
            packet_handler_->read2ByteTxRx(
                port_handler_,
                servo_ids_[i],
                MX_PRESENT_POSITION,
                reinterpret_cast<uint16_t*>(&current_position_bits_[i]));
        }
        return true;
    }

    bool write_goal_positions()
    {
        if(simulation_)
        {
            current_position_bits_ = goal_pose_bits_;
            return true;
        }

        for(size_t i = 0; i < num_servos_; i++)
        {
            packet_handler_->write2ByteTxRx(
                port_handler_,
                servo_ids_[i],
                MX_GOAL_POSITION,
                goal_pose_bits_[i],
                &dxl_error);
        }
        return true;
    }

    std::vector<double> bits_to_radians(const std::vector<int>& bits)
    {
        std::vector<double> radians(bits.size());
        for(size_t i = 0; i < bits.size(); i++)
        {
            radians[i] = servo_directions_[i] *
                         (bits[i] - servo_zeros_[i]) /
                         servo_bits_per_radian_[i];
        }
        return radians;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadNode>());
    rclcpp::shutdown();
    return 0;
}
