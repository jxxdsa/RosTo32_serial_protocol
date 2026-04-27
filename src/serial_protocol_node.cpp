#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <mutex>
#include "serial_protocol/serial_port.hpp"

class SerialProtocolNode : public rclcpp::Node {
public:
    // 命令类型定义
    static constexpr uint8_t CMD_HEARTBEAT = 0x00;
    static constexpr uint8_t CMD_ARM       = 0x01;
    static constexpr uint8_t CMD_DISARM    = 0x02;
    static constexpr uint8_t CMD_VELOCITY  = 0x03;
    static constexpr uint8_t CMD_HOLD      = 0x04;   // 保留
    static constexpr uint8_t CMD_LAND      = 0x05;
    static constexpr uint8_t CMD_EMERGENCY = 0x06;

    // 优先级定义（数值越大优先级越高）
    enum Priority : int {
        PRIORITY_HEARTBEAT = 0,
        PRIORITY_VELOCITY  = 1,
        PRIORITY_LAND      = 2,
        PRIORITY_ARM       = 3,
        PRIORITY_DISARM    = 4,
        PRIORITY_EMERGENCY = 5
    };

    // 活跃命令描述
    struct ActiveCommand {
        uint8_t cmd = CMD_HEARTBEAT;
        float data[4] = {0};
        Priority priority = PRIORITY_HEARTBEAT;
        bool is_one_shot = false;   // 一次性命令，发送后自动降级
    };

    SerialProtocolNode()
    : Node("serial_protocol_node")
    {
        // 参数声明
        this->declare_parameter<std::string>("port", "/dev/ttyS3");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<double>("send_rate_hz", 10.0);
        this->declare_parameter<double>("vel_timeout", 0.2);

        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baud").as_int();

        try {
            serial_ = std::make_unique<SerialPort>(port, baud);
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened, baudrate %d", port.c_str(), baud);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Serial init error: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 订阅 /cmd_vel
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialProtocolNode::vel_callback, this, std::placeholders::_1));

        // 创建服务
        arm_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/arm", std::bind(&SerialProtocolNode::handle_arm, this, std::placeholders::_1, std::placeholders::_2));
        disarm_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/disarm", std::bind(&SerialProtocolNode::handle_disarm, this, std::placeholders::_1, std::placeholders::_2));
        land_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/land", std::bind(&SerialProtocolNode::handle_land, this, std::placeholders::_1, std::placeholders::_2));
        emergency_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/emergency", std::bind(&SerialProtocolNode::handle_emergency, this, std::placeholders::_1, std::placeholders::_2));

        // 统一的10Hz发送定时器
        double period = 1.0 / this->get_parameter("send_rate_hz").as_double();
        send_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&SerialProtocolNode::timer_callback, this));

        last_vel_time_ = this->now();
        // 初始化活跃命令为心跳
        active_cmd_ = ActiveCommand{};
    }

private:
    // ---------- 回调函数 ----------

    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        desired_vel_[0] = msg->linear.x;
        desired_vel_[1] = msg->linear.y;
        desired_vel_[2] = msg->linear.z;
        desired_vel_[3] = msg->angular.z;
        last_vel_time_ = this->now();

        // 尝试提升当前命令为 VELOCITY（若未被高优先级命令占据）
        request_command(CMD_VELOCITY, desired_vel_, PRIORITY_VELOCITY, false); // 持续性命令
    }

    void handle_arm(const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        request_command(CMD_ARM, zero, PRIORITY_ARM, true);
        res->success = true;
        res->message = "ARM requested";
    }

    void handle_disarm(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        request_command(CMD_DISARM, zero, PRIORITY_DISARM, true);
        res->success = true;
        res->message = "DISARM requested";
    }

    void handle_land(const std_srvs::srv::Trigger::Request::SharedPtr,
                     std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        request_command(CMD_LAND, zero, PRIORITY_LAND, true);
        res->success = true;
        res->message = "LAND requested";
    }

    void handle_emergency(const std_srvs::srv::Trigger::Request::SharedPtr,
                          std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        request_command(CMD_EMERGENCY, zero, PRIORITY_EMERGENCY, true);
        res->success = true;
        res->message = "EMERGENCY STOP requested";
    }

    // 统一的请求入口：仅当优先级不低于当前活跃命令时覆盖
    void request_command(uint8_t cmd, const float* data, Priority prio, bool one_shot) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (prio >= active_cmd_.priority) {
            active_cmd_.cmd = cmd;
            std::copy(data, data + 4, active_cmd_.data);
            active_cmd_.priority = prio;
            active_cmd_.is_one_shot = one_shot;
        }
    }

    // 定时器回调：10Hz
    void timer_callback() {
        ActiveCommand cmd_to_send;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cmd_to_send = active_cmd_;
        }

        // 发送命令包
        send_command(cmd_to_send.cmd,
                     cmd_to_send.data[0], cmd_to_send.data[1],
                     cmd_to_send.data[2], cmd_to_send.data[3]);

        // 发送后处理：若为一次性命令，执行后降级
        if (cmd_to_send.is_one_shot) {
            std::lock_guard<std::mutex> lock(mutex_);
            // 若定时器周期内没有被更高优先级命令覆盖，则降级
            if (active_cmd_.cmd == cmd_to_send.cmd &&
                active_cmd_.priority == cmd_to_send.priority) {
                recompute_base_command();   // 回落到速度或心跳
            }
        }
    }

    // 在一次命令发送后重新计算持续命令（速度或心跳）
    void recompute_base_command() {
        // 此函数在 mutex_ 锁内调用
        double timeout = this->get_parameter("vel_timeout").as_double();
        bool vel_active = ((this->now() - last_vel_time_).seconds() < timeout);
        if (vel_active) {
            active_cmd_.cmd = CMD_VELOCITY;
            std::copy(desired_vel_, desired_vel_ + 4, active_cmd_.data);
            active_cmd_.priority = PRIORITY_VELOCITY;
            active_cmd_.is_one_shot = false;
        } else {
            active_cmd_.cmd = CMD_HEARTBEAT;
            std::fill(active_cmd_.data, active_cmd_.data + 4, 0.0f);
            active_cmd_.priority = PRIORITY_HEARTBEAT;
            active_cmd_.is_one_shot = false;
        }
    }

    // 打包发送20字节指令包
    void send_command(uint8_t cmd, float v1, float v2, float v3, float v4) {
        if (!serial_) return;

        uint8_t packet[20];
        packet[0] = 0xA5;
        packet[1] = cmd;

        float data[4] = {v1, v2, v3, v4};
        memcpy(packet + 2, data, 16);

        uint8_t checksum = 0;
        for (int i = 0; i < 18; i++) {
            checksum ^= packet[i];
        }
        packet[18] = checksum;
        packet[19] = 0x5B;

        if (!serial_->write(packet, 20)) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed");
        }
    }

    // 成员变量
    std::unique_ptr<SerialPort> serial_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_srv_;

    rclcpp::TimerBase::SharedPtr send_timer_;

    std::mutex mutex_;
    float desired_vel_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    rclcpp::Time last_vel_time_;

    ActiveCommand active_cmd_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialProtocolNode>());
    rclcpp::shutdown();
    return 0;
}