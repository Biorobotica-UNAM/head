#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <chrono>
#include <vector>
#include <string>
#include <functional>
#include <cstdint>

#define MX_PRESENT_POSITION 36
#define MX_GOAL_POSITION    30
#define MX_MOVING_SPEED     32
#define MX_TORQUE_ENABLE    24
#define MX_TORQUE_LIMIT     34

class HeadNode : public rclcpp::Node {
public:
    HeadNode() : Node("head_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing head_node...");

        port_ = nullptr;
        packet_ = nullptr;
        port_ready_ = false;

        if (!load_parameters()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load parameters");
            return;
        }

        setup_interfaces();

        // Al iniciar, el objetivo es zero-zero
        for (size_t i = 0; i < ids_.size(); i++) {
            goal_bits_[i] = zeros_[i];
        }

        last_retry_time_ = std::chrono::steady_clock::now();

        // Intento inicial, sin matar el nodo si falla
        init_dxl();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&HeadNode::update_loop, this));

        RCLCPP_INFO(this->get_logger(), "head_node started successfully");
    }

    ~HeadNode()
    {
        if (port_ != nullptr) {
            port_->closePort();
        }
    }

private:

    /* ------------------------------ */
    /*         PARAMETERS             */
    /* ------------------------------ */

    std::string port_name_;
    int baudrate_;
    int max_speed_;

    std::vector<int> ids_;
    std::vector<int> zeros_;
    std::vector<int> directions_;
    std::vector<double> bits_per_rad_;
    std::vector<std::string> joint_names_;

    /* ------------------------------ */
    /*     DYNAMIXEL HANDLES          */
    /* ------------------------------ */

    dynamixel::PortHandler *port_;
    dynamixel::PacketHandler *packet_;
    bool port_ready_;

    /* ------------------------------ */
    /*     ROS INTERFACES             */
    /* ------------------------------ */

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_status_;

    /* ------------------------------ */
    /*            STATE               */
    /* ------------------------------ */

    std::vector<int> curr_bits_;
    std::vector<int> goal_bits_;
    std::vector<bool> motor_connected_;

    std::chrono::steady_clock::time_point last_retry_time_;
    std::chrono::milliseconds retry_period_{1000};

    /* ------------------------------ */
    /*     LOAD PARAMETERS            */
    /* ------------------------------ */

    bool load_parameters()
    {
        this->declare_parameter<std::string>("port", "");
        this->declare_parameter<int>("baudrate", 1000000);
        this->declare_parameter<int>("max_speed", 50);
        this->declare_parameter<std::vector<int64_t>>("servo_head_ids");
        this->declare_parameter<std::vector<double>>("servo_head_bits_per_radian");
        this->declare_parameter<std::vector<int64_t>>("servo_head_zeros");
        this->declare_parameter<std::vector<int64_t>>("servo_head_directions");
        this->declare_parameter<std::vector<std::string>>("joint_names");
        this->declare_parameter<std::string>("goal_topic", "/goal_pose");

        port_name_ = this->get_parameter("port").as_string();
        baudrate_  = this->get_parameter("baudrate").as_int();
        max_speed_ = this->get_parameter("max_speed").as_int();

        auto ids64 = this->get_parameter("servo_head_ids").as_integer_array();
        auto zeros64 = this->get_parameter("servo_head_zeros").as_integer_array();
        auto dirs64 = this->get_parameter("servo_head_directions").as_integer_array();
        bits_per_rad_ = this->get_parameter("servo_head_bits_per_radian").as_double_array();
        joint_names_ = this->get_parameter("joint_names").as_string_array();

        for (auto v : ids64) ids_.push_back((int)v);
        for (auto v : zeros64) zeros_.push_back((int)v);
        for (auto v : dirs64) directions_.push_back((int)v);

        if (ids_.empty()) return false;

        if (ids_.size() != zeros_.size() ||
            ids_.size() != directions_.size() ||
            ids_.size() != bits_per_rad_.size() ||
            ids_.size() != joint_names_.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "Parameter size mismatch in head configuration");
            return false;
        }

        curr_bits_.resize(ids_.size(), 0);
        goal_bits_.resize(ids_.size(), 0);
        motor_connected_.resize(ids_.size(), false);

        return true;
    }

    /* ------------------------------ */
    /*   DXL INITIALIZATION           */
    /* ------------------------------ */

    bool init_dxl()
    {
        if (port_ == nullptr) {
            port_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        }

        if (packet_ == nullptr) {
            packet_ = dynamixel::PacketHandler::getPacketHandler(1.0);
        }

        if (!port_->openPort()) {
            port_ready_ = false;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Failed to open port %s", port_name_.c_str());
            return false;
        }

        if (!port_->setBaudRate(baudrate_)) {
            port_ready_ = false;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Failed to set baudrate %d", baudrate_);
            return false;
        }

        port_ready_ = true;

        for (size_t i = 0; i < ids_.size(); i++) {
            try_initialize_motor(i);
        }

        return true;
    }

    bool try_initialize_motor(size_t i)
    {
        if (!port_ready_) return false;

        const int id = ids_[i];

        uint16_t pos = 0;
        int comm_result = packet_->read2ByteTxRx(port_, id, MX_PRESENT_POSITION, &pos);

        if (comm_result != COMM_SUCCESS) {
            motor_connected_[i] = false;
            return false;
        }

        curr_bits_[i] = pos;

        // Igual que tu nodo original
        packet_->write1ByteTxRx(port_, id, MX_TORQUE_ENABLE, 1);
        packet_->write2ByteTxRx(port_, id, MX_MOVING_SPEED, max_speed_);
        packet_->write2ByteTxRx(port_, id, MX_GOAL_POSITION, goal_bits_[i]);

        bool was_connected = motor_connected_[i];
        motor_connected_[i] = true;

        if (!was_connected) {
            RCLCPP_INFO(
                this->get_logger(),
                "Motor %d detected. Torque enabled. Moving to zero: %d",
                id, goal_bits_[i]);
        }

        return true;
    }

    /* ------------------------------ */
    /*         ROS INTERFACES         */
    /* ------------------------------ */

    void setup_interfaces()
    {
        std::string goal_topic = this->get_parameter("goal_topic").as_string();

        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);

        pub_status_ = this->create_publisher<std_msgs::msg::Int32>(
            "status", 10);

        sub_goal_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            goal_topic,
            10,
            std::bind(&HeadNode::callback_goal, this, std::placeholders::_1)
        );
    }

    /* ------------------------------ */
    /*          CALLBACKS             */
    /* ------------------------------ */

    void callback_goal(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < ids_.size()) return;

        for (size_t i = 0; i < ids_.size(); i++) {
            double q_rad = msg->data[i];
            goal_bits_[i] = static_cast<int>(
                zeros_[i] + directions_[i] * bits_per_rad_[i] * q_rad
            );
        }
    }

    /* ------------------------------ */
    /*            STATUS              */
    /* ------------------------------ */

    bool all_motors_connected() const
    {
        for (bool ok : motor_connected_) {
            if (!ok) return false;
        }
        return true;
    }

    void publish_status_if_needed()
    {
        if (!port_ready_ || !all_motors_connected()) {
            std_msgs::msg::Int32 msg;
            msg.data = 1;
            pub_status_->publish(msg);
        }
    }

    /* ------------------------------ */
    /*          MAIN LOOP             */
    /* ------------------------------ */

    void update_loop()
    {
        // 1) Recuperar puerto si se perdió
        if (!port_ready_) {
            if (!init_dxl()) {
                publish_status_if_needed();
                return;
            }
        }

        // 2) Reintentar detectar motores faltantes cada 1 segundo
        auto now = std::chrono::steady_clock::now();
        if ((now - last_retry_time_) >= retry_period_) {
            for (size_t i = 0; i < ids_.size(); i++) {
                if (!motor_connected_[i]) {
                    try_initialize_motor(i);
                }
            }
            last_retry_time_ = now;
        }

        // 3) Escribir goals solo a motores detectados
        // NO marcar desconectado aquí por un write fallido aislado
        for (size_t i = 0; i < ids_.size(); i++) {
            if (!motor_connected_[i]) continue;

            int comm_result = packet_->write2ByteTxRx(
                port_, ids_[i], MX_GOAL_POSITION, goal_bits_[i]);

            if (comm_result != COMM_SUCCESS) {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Write goal failed for motor %d (comm_result=%d)",
                    ids_[i], comm_result);
            }
        }

        // 4) Leer posiciones: AQUÍ sí decidimos si se perdió el motor
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = joint_names_;
        js.position.resize(ids_.size(), 0.0);

        for (size_t i = 0; i < ids_.size(); i++) {
            if (!motor_connected_[i]) {
                js.position[i] = 0.0;
                continue;
            }

            uint16_t pos = 0;
            int comm_result = packet_->read2ByteTxRx(
                port_, ids_[i], MX_PRESENT_POSITION, &pos);

            if (comm_result != COMM_SUCCESS) {
                bool was_connected = motor_connected_[i];
                motor_connected_[i] = false;
                js.position[i] = 0.0;

                if (was_connected) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Lost motor %d while reading position",
                        ids_[i]);
                }
                continue;
            }

            curr_bits_[i] = pos;
            js.position[i] =
                directions_[i] * (curr_bits_[i] - zeros_[i]) / bits_per_rad_[i];
        }

        pub_joint_states_->publish(js);

        // 5) Publicar status=1 si falta al menos un motor o el puerto
        publish_status_if_needed();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadNode>());
    rclcpp::shutdown();
    return 0;
}