#include <algorithm>
#include <atomic>
#include <future>
#include <memory>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
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

        std::vector<double> starting_position = {0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "Moving to starting position...");

        std::string msg;
        if (move_to_position(starting_position, msg))
            RCLCPP_INFO(this->get_logger(), "Starting position reached");
        else
            RCLCPP_WARN(this->get_logger(), "Failed to reach starting position: %s", msg.c_str());

        timer_ = this->create_wall_timer(
            100ms, std::bind(&HeadNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Head node ready");
    }

    ~HeadNode()
    {
        motion_cancelled_.store(true);
        if (motion_future_.valid())
            motion_future_.wait();

        if(port_handler_ != nullptr)
            port_handler_->closePort();
    }

private:
    // =========================
    // ROS
    // =========================
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
    rclcpp::Service<head_interfaces::srv::MoveHead>::SharedPtr srv_move_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_pose_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;

    // =========================
    // DYNAMIXEL
    // =========================
    dynamixel::PortHandler* port_handler_   = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;
    std::mutex hardware_mutex_;

    // =========================
    // PARAMETERS
    // =========================
    std::string port_name_;
    int  baudrate_;
    bool torque_enable_;
    int  max_speed_;
    std::string joint_states_topic_;
    std::string current_pose_topic_;
    std::string service_name_;
    std::string goal_pose_topic_;
    std::string goal_reached_topic_;
    bool simulation_;

    std::vector<int>    servo_ids_;
    std::vector<double> servo_bits_per_radian_;
    std::vector<int>    servo_zeros_;
    std::vector<int>    servo_directions_;
    std::vector<int>    servo_min_limits_;
    std::vector<int>    servo_max_limits_;

    size_t num_servos_;
    std::vector<std::string> joint_names_;

    // =========================
    // STATE
    // =========================
    std::vector<int> current_position_bits_;
    std::vector<int> goal_pose_bits_;

    std::atomic<bool> motors_available_{false};
    uint8_t dxl_error_        = 0;

    rclcpp::Time last_config_attempt_{0, 0, RCL_ROS_TIME};

    std::atomic<bool> motion_in_progress_{false};
    std::atomic<bool> motion_active_{false};
    std::atomic<bool> motion_cancelled_{false};
    std::future<void> motion_future_;

    // =========================
    // MESSAGES
    // =========================
    sensor_msgs::msg::JointState    joint_states_;
    std_msgs::msg::Float64MultiArray msg_current_pose_;

    // ======================================================
    // PARAMETERS
    // ======================================================
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
        this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
        this->declare_parameter<std::string>("goal_reached_topic", "goal_reached");

        this->declare_parameter<std::vector<int64_t>>("servo_ids", std::vector<int64_t>());
        this->declare_parameter<std::vector<double>> ("servo_bits_per_radian", std::vector<double>());
        this->declare_parameter<std::vector<int64_t>>("servo_zeros", std::vector<int64_t>());
        this->declare_parameter<std::vector<int64_t>>("servo_directions", std::vector<int64_t>());
        this->declare_parameter<std::vector<int64_t>>("servo_min_limits", std::vector<int64_t>());
        this->declare_parameter<std::vector<int64_t>>("servo_max_limits", std::vector<int64_t>());
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>());

        simulation_    = this->get_parameter("simulation").as_bool();
        port_name_     = this->get_parameter("port").as_string();
        baudrate_      = this->get_parameter("baudrate").as_int();
        torque_enable_ = this->get_parameter("torque_enable").as_bool();
        max_speed_     = this->get_parameter("max_speed").as_int();

        joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
        current_pose_topic_ = this->get_parameter("current_pose_topic").as_string();
        service_name_       = this->get_parameter("head_controller_service").as_string();
        goal_pose_topic_    = this->get_parameter("goal_pose_topic").as_string();
        goal_reached_topic_ = this->get_parameter("goal_reached_topic").as_string();

        auto ids   = this->get_parameter("servo_ids").as_integer_array();
        auto zeros = this->get_parameter("servo_zeros").as_integer_array();
        auto dirs  = this->get_parameter("servo_directions").as_integer_array();
        auto min_l = this->get_parameter("servo_min_limits").as_integer_array();
        auto max_l = this->get_parameter("servo_max_limits").as_integer_array();

        joint_names_           = this->get_parameter("joint_names").as_string_array();
        servo_bits_per_radian_ = this->get_parameter("servo_bits_per_radian").as_double_array();

        num_servos_ = ids.size();

        if(num_servos_ == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "No servos configured");
            return false;
        }

        for(auto v : ids)   servo_ids_.push_back((int)v);
        for(auto v : zeros) servo_zeros_.push_back((int)v);
        for(auto v : dirs)  servo_directions_.push_back((int)v);
        for(auto v : min_l) servo_min_limits_.push_back((int)v);
        for(auto v : max_l) servo_max_limits_.push_back((int)v);

        return true;
    }

    // ======================================================
    // INIT HARDWARE
    // ======================================================
    bool initialize_dynamixel()
	{
		if (simulation_)
		{
		    RCLCPP_WARN(this->get_logger(), "Running in SIMULATION mode");
		    current_position_bits_.assign(num_servos_, 2048);
		    goal_pose_bits_.assign(num_servos_, 2048);
		    motors_available_.store(true);
		    return true;
		}

		if (port_name_.empty())
		{
		    RCLCPP_ERROR(this->get_logger(), "Port not specified");
		    return false;
		}

		port_handler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
		packet_handler_ = dynamixel::PacketHandler::getPacketHandler(1.0);

		if (!port_handler_->openPort()) 
		{
		    RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", port_name_.c_str());
		    return false;
		}

		if (!port_handler_->setBaudRate(baudrate_)) 
		{
		    RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate to %d", baudrate_);
		    return false;
		}

		current_position_bits_.assign(num_servos_, 0);
		goal_pose_bits_.assign(num_servos_, 0);

		if (read_current_positions())
		{
		    if (configure_servos()) 
		    {
		        goal_pose_bits_ = current_position_bits_;
		        motors_available_.store(true);
		        RCLCPP_INFO(this->get_logger(), "Dynamixel hardware initialized and configured successfully.");
		    }
		    else 
		    {
		        RCLCPP_ERROR(this->get_logger(), "Hardware found but configuration failed.");
		        motors_available_.store(false);
		    }
		}
		else
		{
		    RCLCPP_WARN(this->get_logger(), "Motors not responding at launch. Will retry in background...");
		    motors_available_.store(false);
		}

		return true; 
	}

    // ======================================================
    // ROS INTERFACES
    // ======================================================
    void setup_interfaces()
	{
		service_cb_group_ = this->create_callback_group(
		    rclcpp::CallbackGroupType::MutuallyExclusive);

		pub_joint_state_  = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
		pub_current_pose_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(current_pose_topic_, 10);
		pub_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);

		srv_move_ = this->create_service<head_interfaces::srv::MoveHead>(
		    service_name_,
		    std::bind(&HeadNode::move_service, this, _1, _2),
		    rclcpp::ServicesQoS(),
		    service_cb_group_);

		rclcpp::SubscriptionOptions sub_options;
		sub_options.callback_group = service_cb_group_;

		sub_goal_pose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
		    goal_pose_topic_, 10,
		    std::bind(&HeadNode::goal_pose_callback, this, _1),
		    sub_options);
	}

    void initialize_messages()
    {
        msg_current_pose_.data.resize(num_servos_);
        joint_states_.name     = joint_names_;
        joint_states_.position.resize(num_servos_, 0.0);
        joint_states_.velocity.resize(num_servos_, 0.0);
        joint_states_.effort.resize(num_servos_,   0.0);
    }

    // ======================================================
    // PUBLISH
    // ======================================================
    void publish()
    {
        joint_states_.header.stamp = this->now();
        joint_states_.position     = bits_to_radians(current_position_bits_);

        pub_joint_state_->publish(joint_states_);

        for (size_t i = 0; i < num_servos_; i++)
            msg_current_pose_.data[i] = joint_states_.position[i];

        pub_current_pose_->publish(msg_current_pose_);
    }

    // ======================================================
    // MOVEMENT CONTROL
    // ======================================================
    bool move_to_position(const std::vector<double>& positions, std::string& message)
	{
		if (positions.size() != num_servos_) {
		    message = "Invalid joint size";
		    return false;
		}

		if (!motors_available_.load() && !simulation_) {
		    message = "Hardware unavailable - Reconfiguration in progress";
		    return false;
		}

		if (simulation_) {
		    for (size_t i = 0; i < num_servos_; i++) {
		        goal_pose_bits_[i] = static_cast<int>(
		            servo_directions_[i] * positions[i] * servo_bits_per_radian_[i] + servo_zeros_[i]);
		        current_position_bits_[i] = goal_pose_bits_[i];
		    }
		    publish();
		    message = "Goal reached (simulation)";
		    return true;
		}

		std::vector<int> target_bits(num_servos_);
		for (size_t i = 0; i < num_servos_; i++) {
		    int raw = static_cast<int>(
		        servo_directions_[i] * positions[i] * servo_bits_per_radian_[i] + servo_zeros_[i]);

		    if (raw < servo_min_limits_[i] || raw > servo_max_limits_[i]) {
				int clamped = std::clamp(raw, servo_min_limits_[i], servo_max_limits_[i]);
				RCLCPP_WARN(this->get_logger(),
					"Joint %s out of range (requested %d, clamped to %d)",
					joint_names_[i].c_str(), raw, clamped);
				raw = clamped;
			}
			target_bits[i] = raw;
		}

		const double max_speed_rad_per_sec = (2.28 * 2.0 * M_PI / 60.0) / 1.1;
		const int tolerance_bits = 50;
		const int polling_ms = 50;

		int max_diff = 0;
		for (size_t i = 0; i < num_servos_; i++) {
			max_diff = std::max(max_diff, std::abs(target_bits[i] - current_position_bits_[i]));
		}
		
		double max_dist_rad = static_cast<double>(max_diff) / servo_bits_per_radian_[0]; 
		int timeout_ms = static_cast<int>((max_dist_rad / max_speed_rad_per_sec) * 1000.0) + 1000;

		{
		    std::lock_guard<std::mutex> lock(hardware_mutex_);
		    if (!motors_available_.load()) {
		        message = "Hardware lost before writing goal";
		        return false;
		    }

		    goal_pose_bits_ = target_bits;
		    write_goal_positions(); 
		}

		int elapsed_ms = 0;
		bool goal_reached = false;

		while (elapsed_ms < timeout_ms && !motion_cancelled_.load()) {
		    {
		        std::lock_guard<std::mutex> lock(hardware_mutex_);
		        if (!read_current_positions()) {
		            motors_available_.store(false);
		            message = "Hardware lost during motion";
		            return false;
		        }
		    }

		    publish(); 

		    goal_reached = true;
		    for (size_t i = 0; i < num_servos_; i++) {
    			if (std::abs(current_position_bits_[i] - target_bits[i]) > tolerance_bits) {
		            goal_reached = false;
		            break;
		        }
		    }

		    if (goal_reached) break;

		    std::this_thread::sleep_for(std::chrono::milliseconds(polling_ms));
		    elapsed_ms += polling_ms;
		}

		if (motion_cancelled_.load())
		    message = "Motion cancelled";
		else
		    message = goal_reached ? "Goal reached" : "Timeout";

		return goal_reached;
	}

    // ======================================================
    // TOPIC CALLBACK (goal_pose)
    // ======================================================
    void goal_pose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        motion_cancelled_.store(true);
        if (motion_future_.valid()) motion_future_.wait();
        
        std::vector<double> positions = msg->data;

        motion_cancelled_.store(false);
        motion_active_.store(true);
        motion_in_progress_.store(true);

        motion_future_ = std::async(std::launch::async, [this, positions]()
        {
            std::string result_msg;
            bool reached = move_to_position(positions, result_msg);
            
            std_msgs::msg::Bool out;
            out.data = reached;
            pub_goal_reached_->publish(out);

            motion_in_progress_.store(false);
            motion_active_.store(false);
        });
    }

    // ======================================================
    // SERVICE
    // ======================================================
    void move_service(
        const std::shared_ptr<head_interfaces::srv::MoveHead::Request> request,
        std::shared_ptr<head_interfaces::srv::MoveHead::Response> response)
    {
        motion_cancelled_.store(false);
        motion_active_.store(true);
        motion_in_progress_.store(true);

        response->success = move_to_position(request->positions, response->message);

        motion_in_progress_.store(false);
        motion_active_.store(false);
    }

    // ======================================================
    // TIMER
    // ======================================================
    void timer_callback() 
	{
		std::unique_lock<std::mutex> lock(hardware_mutex_);
		if (simulation_) { publish(); return; }

		if (motors_available_.load()) {
		    if (!read_current_positions()) {
		        RCLCPP_ERROR(this->get_logger(), "Hardware connection lost!");
		        motors_available_.store(false);
		        return;
		    }
		    publish();
		} 
		else {
		    auto now = this->now();
		    if ((now - last_config_attempt_).seconds() > 2.0) {
		        last_config_attempt_ = now;
		        
		        RCLCPP_INFO(this->get_logger(), "Attempting hardware recovery...");
		        
		        if (read_current_positions()) { 
		            if (configure_servos()) {   
		                RCLCPP_INFO(this->get_logger(), "Motors online and configured");
		                goal_pose_bits_ = current_position_bits_;
		                motors_available_.store(true);
		            } else {
		                RCLCPP_WARN(this->get_logger(), "Motors found but configuration failed. Retrying...");
		            }
		        } else {
		            RCLCPP_DEBUG(this->get_logger(), "Motors still not responding.");
		        }
		    }
		}
	}

    // ======================================================
    // HARDWARE
    // ======================================================
    bool configure_servos() 
	{
		if (simulation_) return true;

		for (size_t i = 0; i < num_servos_; i++) {
		    int result;
		    
		    packet_handler_->write1ByteTxRx(port_handler_, servo_ids_[i], MX_TORQUE_ENABLE, 0, &dxl_error_);

		    result = packet_handler_->write2ByteTxRx(port_handler_, servo_ids_[i], MX_MOVING_SPEED, max_speed_, &dxl_error_);
		    if (result != COMM_SUCCESS) return false;

		    result = packet_handler_->write2ByteTxRx(port_handler_, servo_ids_[i], MX_TORQUE_LIMIT, TORQUE_LIMIT_BITS, &dxl_error_);
		    if (result != COMM_SUCCESS) return false;

		    result = packet_handler_->write1ByteTxRx(port_handler_, servo_ids_[i], MX_TORQUE_ENABLE, 1, &dxl_error_);
		    if (result != COMM_SUCCESS) return false;

		    RCLCPP_INFO(this->get_logger(), "Servo %d configurado (Vel: %d, Torque: ON)", servo_ids_[i], max_speed_);
		}
		return true;
	}

    bool read_current_positions()
    {
        if (simulation_) return true;

        for (size_t i = 0; i < num_servos_; i++)
        {
            uint16_t raw = 0;
            int result = packet_handler_->read2ByteTxRx(
                port_handler_, servo_ids_[i], MX_PRESENT_POSITION, &raw, &dxl_error_);

            if (result != COMM_SUCCESS)
            {
                if (motors_available_.load()) {
                    RCLCPP_ERROR(this->get_logger(), "Lost communication with servo %d", servo_ids_[i]);
                    motors_available_.store(false);
                }
                return false;
            }
            current_position_bits_[i] = static_cast<int>(raw);
        }
        
        motors_available_.store(true);
        return true;
    }

    void write_goal_positions()
    {
        if (simulation_) return;
        for (size_t i = 0; i < num_servos_; i++)
        {
            packet_handler_->write2ByteTxRx(port_handler_, servo_ids_[i], MX_GOAL_POSITION, goal_pose_bits_[i], &dxl_error_);
        }
    }

    std::vector<double> bits_to_radians(const std::vector<int>& bits)
    {
        std::vector<double> radians(bits.size());
        for (size_t i = 0; i < bits.size(); i++)
        {
            radians[i] = servo_directions_[i] * (bits[i] - servo_zeros_[i]) / servo_bits_per_radian_[i];
        }
        return radians;
    }
};

// ======================================================
// MAIN
// ======================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeadNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
