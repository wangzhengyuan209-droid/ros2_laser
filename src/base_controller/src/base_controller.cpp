/******************************************************************
ROS2 Base Controller - Serial Bridge to STM32

Functions:
1. Send velocity commands to STM32 via serial (/cmd_vel)
2. Publish odometry from STM32 (/odom + TF)
3. Laser control via cmd_1 field (/laser_fire)

Serial Protocol:
  Upper → STM32: "ROS:" + 32 bytes data + ">STM\r\n" (42 bytes)
  STM32 → Upper: "STM:" + 32 bytes data + ">ROS\r\n" (42 bytes)
*******************************************************************/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <serial/serial.h>
#include <string>
#include <memory>

#define PI 3.14159265358979323846

using namespace std;

// Serial object
serial::Serial ROS_UART;

// ROS→STM32 data structure (32 bytes)
typedef struct __attribute__((packed))
{
    float cmd_vx;      // m/s
    float cmd_vy;      // m/s
    float cmd_womiga;  // rad/s
    uint32_t cmd_1;    // laser: 1=on, 0=off
    uint32_t cmd_2;
    uint32_t cmd_3;
    uint32_t cmd_4;
    uint32_t cmd_5;
} ROS_STM_TYPEDEF;

// STM32→ROS data structure (32 bytes)
typedef struct __attribute__((packed))
{
    float odom_px;     // m
    float odom_py;     // m
    float odom_ang;    // rad
    float odom_vx;     // m/s
    float odom_vy;     // m/s
    float odom_womiga; // rad/s
    uint32_t state_1;
    uint32_t state_2;
} STM_ROS_TYPEDEF;

ROS_STM_TYPEDEF ROS_STM_DATA;
STM_ROS_TYPEDEF STM_ROS_DATA;

class BaseControllerNode : public rclcpp::Node
{
public:
    BaseControllerNode() : Node("base_controller"), tf_broadcaster_(this)
    {
        // 1. Init serial
        if (User_SerialInit() < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Serial port init failed!");
            rclcpp::shutdown();
            return;
        }

        // 2. Subscribers
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 20, std::bind(&BaseControllerNode::User_CmdVelCallback, this, std::placeholders::_1));
        sub_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 20, std::bind(&BaseControllerNode::User_JoystickCallback, this, std::placeholders::_1));
        sub_laser_ = this->create_subscription<std_msgs::msg::Bool>(
            "laser_fire", 10, std::bind(&BaseControllerNode::User_LaserCallback, this, std::placeholders::_1));

        // 3. Publishers
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
        pub_attack_ = this->create_publisher<std_msgs::msg::Bool>("under_attack", 10);

        // 4. Timer (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&BaseControllerNode::MainLoop, this));

        RCLCPP_INFO(this->get_logger(), "BaseController node started!");
    }

    ~BaseControllerNode()
    {
        ROS_UART.close();
        RCLCPP_INFO(this->get_logger(), "BaseController node stopped!");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joystick_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_laser_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_attack_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    int User_SerialInit(void)
    {
        try
        {
            ROS_UART.setPort("/dev/ttyS1");
            ROS_UART.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(50);
            ROS_UART.setTimeout(to);
            ROS_UART.open();
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port ttyS1!");
            return -1;
        }

        if (ROS_UART.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port ttyS1 opened!");
            return 1;
        }
        return -1;
    }

    void User_CmdVelCallback(const geometry_msgs::msg::Twist& cmd_input)
    {
        ROS_STM_DATA.cmd_vx = cmd_input.linear.x;
        ROS_STM_DATA.cmd_vy = cmd_input.linear.y;
        ROS_STM_DATA.cmd_womiga = cmd_input.angular.z;
    }

    void User_JoystickCallback(const sensor_msgs::msg::Joy& joy)
    {
        ROS_STM_DATA.cmd_vx = joy.axes[0];
        ROS_STM_DATA.cmd_vy = joy.axes[1];
        ROS_STM_DATA.cmd_womiga = joy.axes[2];
        // cmd_1 reserved for laser control (via /laser_fire topic)
        ROS_STM_DATA.cmd_2 = joy.axes[6];
        ROS_STM_DATA.cmd_3 = joy.axes[7];
        ROS_STM_DATA.cmd_4 = joy.buttons[6];
        ROS_STM_DATA.cmd_5 = joy.buttons[7];
    }

    void User_LaserCallback(const std_msgs::msg::Bool& msg)
    {
        ROS_STM_DATA.cmd_1 = msg.data ? 1 : 0;
    }

    void User_RosToStmSend(void)
    {
        uint8_t data_buf[42] = {0};
        data_buf[0] = 'R'; data_buf[1] = 'O'; data_buf[2] = 'S'; data_buf[3] = ':';
        data_buf[36] = '>'; data_buf[37] = 'S'; data_buf[38] = 'T'; data_buf[39] = 'M';
        data_buf[40] = '\r'; data_buf[41] = '\n';
        memcpy(&data_buf[4], &ROS_STM_DATA, 32);
        ROS_UART.write(data_buf, 42);
    }

    uint8_t User_StmToRosParas(void)
    {
        uint8_t rec_buffer[84] = {0};
        int len = ROS_UART.read(rec_buffer, 83);
        ROS_UART.flushInput();

        for (int i = 0; i < len - 41; i++)
        {
            if ((rec_buffer[i+0] == 'S') && (rec_buffer[i+1] == 'T') &&
                (rec_buffer[i+2] == 'M') && (rec_buffer[i+3] == ':') &&
                (rec_buffer[i+36] == '>') && (rec_buffer[i+37] == 'R') &&
                (rec_buffer[i+38] == 'O') && (rec_buffer[i+39] == 'S') &&
                (rec_buffer[i+40] == '\r') && (rec_buffer[i+41] == '\n'))
            {
                memcpy(&STM_ROS_DATA, &rec_buffer[i+4], 32);
                return 1;
            }
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "STM communicate lost...");
        return 0;
    }

    void User_OdomTopicPublish(void)
    {
        nav_msgs::msg::Odometry odom;
        geometry_msgs::msg::TransformStamped odom_trans;

        // TF: odom → base_link
        odom_trans.header.stamp = this->now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = STM_ROS_DATA.odom_px;
        odom_trans.transform.translation.y = STM_ROS_DATA.odom_py;
        odom_trans.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, STM_ROS_DATA.odom_ang);
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(odom_trans);

        // Odometry
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = STM_ROS_DATA.odom_px;
        odom.pose.pose.position.y = STM_ROS_DATA.odom_py;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_trans.transform.rotation;
        odom.twist.twist.linear.x = STM_ROS_DATA.odom_vx;
        odom.twist.twist.linear.y = STM_ROS_DATA.odom_vy;
        odom.twist.twist.angular.z = STM_ROS_DATA.odom_womiga;
        pub_odom_->publish(odom);

        // Publish attack state
        std_msgs::msg::Bool attack_msg;
        attack_msg.data = (STM_ROS_DATA.state_1 == 1);
        pub_attack_->publish(attack_msg);
    }

    void MainLoop(void)
    {
        User_RosToStmSend();
        if (User_StmToRosParas())
        {
            User_OdomTopicPublish();
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
