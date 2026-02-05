/*
  Código de control de los dos motores de la cabeza pan y tilt (horizontal, vertical)
  El nodo recibe un vector en bits (pan, tilt) 
  Los rangos para cada motor fueron definidos en el software Dynamixel Wizard:
  [1024->tope derecho,  3091->tope izquierdo] y [3100->tope superior, 4095->tope inferior]
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

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

        if (!load_parameters()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }

        if (!init_dxl()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel");
            rclcpp::shutdown();
            return;
        }

        setup_interfaces();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&HeadNode::update_loop, this));

        RCLCPP_INFO(this->get_logger(), "head_node started successfully");
    }

    ~HeadNode()
    {
        port_->closePort();
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

    /* ------------------------------ */
    /*     DYNAMIXEL HANDLES          */
    /* ------------------------------ */

    dynamixel::PortHandler *port_;
    dynamixel::PacketHandler *packet_;

    /* ------------------------------ */
    /*     ROS INTERFACES             */
    /* ------------------------------ */

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_goal_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

    std::vector<int> curr_bits_;
    std::vector<int> goal_bits_;

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
        this->declare_parameter<std::string>("goal_topic", "/goal_pose");     // evita hardcodear


        port_name_ = this->get_parameter("port").as_string();
        baudrate_  = this->get_parameter("baudrate").as_int();
        max_speed_ = this->get_parameter("max_speed").as_int();

        auto ids64 = this->get_parameter("servo_head_ids").as_integer_array();
        auto zeros64 = this->get_parameter("servo_head_zeros").as_integer_array();
        auto dirs64 = this->get_parameter("servo_head_directions").as_integer_array();
        bits_per_rad_ = this->get_parameter("servo_head_bits_per_radian").as_double_array();

        for (auto v : ids64) ids_.push_back((int)v);
        for (auto v : zeros64) zeros_.push_back((int)v);
        for (auto v : dirs64) directions_.push_back((int)v);

        if (ids_.empty()) return false;

        curr_bits_.resize(ids_.size());
        goal_bits_.resize(ids_.size());

        return true;
    }

    /* ------------------------------ */
    /*   DXL INITIALIZATION           */
    /* ------------------------------ */

    bool init_dxl()
    {
        port_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packet_ = dynamixel::PacketHandler::getPacketHandler(1.0);

        if (!port_->openPort()) return false;
        if (!port_->setBaudRate(baudrate_)) return false;

        // enable torque
        for (int id : ids_) {
            packet_->write1ByteTxRx(port_, id, MX_TORQUE_ENABLE, 1);
            packet_->write2ByteTxRx(port_, id, MX_MOVING_SPEED, max_speed_);
        }

        // read initial positions
        for (size_t i = 0; i < ids_.size(); i++) {
            uint16_t pos;
            packet_->read2ByteTxRx(port_, ids_[i], MX_PRESENT_POSITION, &pos);
            curr_bits_[i] = pos;
            goal_bits_[i] = pos;
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

        // --------- Constantes PAN ---------
        const double PAN_ZERO_BITS = 2057.0;
        const double PAN_DEG_PER_BIT = -180.0 / (3091.0 - 1024.0); // ≈ -0.087
        const double PAN_BIT_PER_DEG = 1.0 / PAN_DEG_PER_BIT;     // ≈ -11.48

        // --------- Constantes TILT ---------
        const double TILT_ZERO_BITS = 3526.0;
        const double TILT_DEG_PER_BIT = 61.0 / (3794.0 - 3100.0); // ≈ 0.0879
        const double TILT_BIT_PER_DEG = 1.0 / TILT_DEG_PER_BIT;  // ≈ 11.37

        double pan_deg  = msg->data[0];
        double tilt_deg = msg->data[1];

        // Conversión grados → bits
        goal_bits_[0] = static_cast<int>(PAN_ZERO_BITS + pan_deg * PAN_BIT_PER_DEG);
        goal_bits_[1] = static_cast<int>(TILT_ZERO_BITS + tilt_deg * TILT_BIT_PER_DEG);
    }

    /* ------------------------------ */
    /*          MAIN LOOP             */
    /* ------------------------------ */

    void update_loop()
    {
        // write goals
        for (size_t i = 0; i < ids_.size(); i++) {
            packet_->write2ByteTxRx(
                port_, ids_[i], MX_GOAL_POSITION, goal_bits_[i]);
        }

        // read current positions
        for (size_t i = 0; i < ids_.size(); i++) {
            uint16_t pos;
            packet_->read2ByteTxRx(port_, ids_[i], MX_PRESENT_POSITION, &pos);
            curr_bits_[i] = pos;

            // ------------------------------
            //   PRINT POSITION IN BITS
            // ------------------------------
            //RCLCPP_INFO(this->get_logger(),
            //    "Motor %d → Position (bits): %d", ids_[i], curr_bits_[i]);
        }

        // publish joint states
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = this->get_parameter("joint_names").as_string_array();
        js.position.resize(ids_.size());

        for (size_t i = 0; i < ids_.size(); i++) {
            js.position[i] =
                directions_[i] * (curr_bits_[i] - zeros_[i]) / bits_per_rad_[i];
        }

        pub_joint_states_->publish(js);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadNode>());
    rclcpp::shutdown();
    return 0;
}