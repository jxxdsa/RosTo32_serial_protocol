#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include "serial_protocol/serial_port.hpp"

class SerialProtocolNode : public rclcpp::Node {
public:
    static constexpr uint8_t CMD_HEARTBEAT = 0x00;
    static constexpr uint8_t CMD_ARM       = 0x01;
    static constexpr uint8_t CMD_DISARM    = 0x02;
    static constexpr uint8_t CMD_VELOCITY  = 0x03;
    static constexpr uint8_t CMD_HOLD      = 0x04;
    static constexpr uint8_t CMD_LAND      = 0x05;
    static constexpr uint8_t CMD_EMERGENCY = 0x06;
    static constexpr uint8_t CMD_DISPLAY   = 0x07;  // չʾģ\BF\E9\C3\FC\C1\EE

    // \D3\C5\CFȼ\B6\CA\FDֵԽ\B4\F3\A3\AC\D3\C5\CFȼ\B6Խ\B8\DF
    enum Priority : int {
        PRIORITY_HEARTBEAT = 0,
        PRIORITY_VELOCITY  = 1,
        PRIORITY_DISPLAY   = 2,   // \B8\DF\D3\DA\CBٶȣ\AC\B5\CD\D3ڽ\B5\C2\E4
        PRIORITY_LAND      = 3,
        PRIORITY_ARM       = 4,
        PRIORITY_DISARM    = 5,
        PRIORITY_EMERGENCY = 6
    };

    struct ActiveCommand {
        uint8_t cmd = CMD_HEARTBEAT;
        float data[4] = {0};
        Priority priority = PRIORITY_HEARTBEAT;
        bool is_one_shot = false;
    };

    SerialProtocolNode()
    : Node("serial_protocol_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyS1");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<double>("send_rate_hz", 20.0);
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

        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialProtocolNode::vel_callback, this, std::placeholders::_1));

        // \D0\C2\D4\F6\A3\BAչʾģ\BF\E9\CA\FD\BEݶ\A9\D4ģ\AC\BB\B0\CC\E2\C3\FB display_data
        display_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "display_data", 10,
            std::bind(&SerialProtocolNode::display_callback, this, std::placeholders::_1));

        arm_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/arm", std::bind(&SerialProtocolNode::handle_arm, this, std::placeholders::_1, std::placeholders::_2));
        disarm_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/disarm", std::bind(&SerialProtocolNode::handle_disarm, this, std::placeholders::_1, std::placeholders::_2));
        land_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/land", std::bind(&SerialProtocolNode::handle_land, this, std::placeholders::_1, std::placeholders::_2));
        emergency_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/emergency", std::bind(&SerialProtocolNode::handle_emergency, this, std::placeholders::_1, std::placeholders::_2));

        double period = 1.0 / this->get_parameter("send_rate_hz").as_double();
        send_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&SerialProtocolNode::timer_callback, this));

        last_vel_time_ = this->now();
        last_display_time_ = this->now();
        active_cmd_ = ActiveCommand{};
        has_display_data_ = false;
    }

    // \CE\F6\B9\B9\BA\AF\CA\FD\A3\BA\B0\B2ȫֹͣ\A3\AC\B7\A2\CB\CD\C1\BD\B4\CE\C1\E3\CBٶ\C8ָ\C1\EE
    ~SerialProtocolNode() {
        if (serial_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down: sending stop commands...");
            send_command(CMD_VELOCITY, 0.0f, 0.0f, 0.0f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            send_command(CMD_VELOCITY, 0.0f, 0.0f, 0.0f, 0.0f);
            RCLCPP_INFO(this->get_logger(), "Stop commands sent.");
        }
    }

private:
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        desired_vel_[0] = msg->linear.x;
        desired_vel_[1] = msg->linear.y;
        desired_vel_[2] = msg->linear.z;
        desired_vel_[3] = msg->angular.z;
        last_vel_time_ = this->now();

        request_command(CMD_VELOCITY, desired_vel_, PRIORITY_VELOCITY, false);
    }

    // \D0\C2\D4\F6\BBص\F7\A3\BA\BD\D3\CA\D5չʾģ\BF\E9\CA\FD\BEݣ\A816\D7ֽڣ\A9\A3\ACӳ\C9\E4Ϊ4\B8\F6float
    void display_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        if (msg->data.size() != 16) {
            RCLCPP_ERROR(this->get_logger(), "Display data must be 16 bytes, got %zu", msg->data.size());
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        // \BD\AB\D7ֽ\DA\CA\FD\BEݸ\B4\D6Ƶ\BD display_data_
        memcpy(display_data_, msg->data.data(), 16);
        last_display_time_ = this->now();
        has_display_data_ = true;
        // \C7\EB\C7\F3\B3\D6\D0\F8\B7\A2\CB\CDչʾ\CA\FD\BEݣ\AC\D3\C5\CFȼ\B6\B8\DF\D3\DAVELOCITY
        request_command(CMD_DISPLAY, display_data_, PRIORITY_DISPLAY, false);
    }

    void handle_arm(const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            request_command(CMD_ARM, zero, PRIORITY_ARM, true);
        }
        res->success = true;
        res->message = "ARM requested";
    }

    void handle_disarm(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            request_command(CMD_DISARM, zero, PRIORITY_DISARM, true);
        }
        res->success = true;
        res->message = "DISARM requested";
    }

    void handle_land(const std_srvs::srv::Trigger::Request::SharedPtr,
                     std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            request_command(CMD_LAND, zero, PRIORITY_LAND, true);
        }
        res->success = true;
        res->message = "LAND requested";
    }

    void handle_emergency(const std_srvs::srv::Trigger::Request::SharedPtr,
                          std_srvs::srv::Trigger::Response::SharedPtr res) {
        float zero[4] = {0};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            request_command(CMD_EMERGENCY, zero, PRIORITY_EMERGENCY, true);
        }
        res->success = true;
        res->message = "EMERGENCY STOP requested";
    }

    // \B5\F7\D3\C3ǰ\B1\D8\D0\EB\D2ѳ\D6\D3\D0 mutex_
    void request_command(uint8_t cmd, const float* data, Priority prio, bool one_shot) {
        if (prio >= active_cmd_.priority) {
            active_cmd_.cmd = cmd;
            std::copy(data, data + 4, active_cmd_.data);
            active_cmd_.priority = prio;
            active_cmd_.is_one_shot = one_shot;
        }
    }

    void timer_callback() {
        ActiveCommand cmd_to_send;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            double timeout = this->get_parameter("vel_timeout").as_double();
            bool vel_active = ((this->now() - last_vel_time_).seconds() < timeout);
            bool display_active = has_display_data_ && ((this->now() - last_display_time_).seconds() < timeout);

            // \B3\D6\D0\F8\B3\A2\CA\D4\D3\C3\D7\EE\D0µ\C4\CF\D4ʾ\CA\FD\BEݸ\FC\D0\C2\C3\FC\C1\D3\C5\CFȼ\B6\B1\A3\BB\A4\A3\A9
            if (display_active) {
                request_command(CMD_DISPLAY, display_data_, PRIORITY_DISPLAY, false);
            }
            if (vel_active) {
                request_command(CMD_VELOCITY, desired_vel_, PRIORITY_VELOCITY, false);
            }

            // \CBٶȳ\ACʱ\BD\B5\BC\B6\B4\A6\C0\ED\A3\A8\B1\A3\C1\F4ԭ\D3\D0\C1\E3\CBٹ\FD\B6\C9\C2߼\AD\A3\A9
            if (active_cmd_.cmd == CMD_VELOCITY && !vel_active) {
                recompute_base_command();
            }
            // չʾ\CA\FD\BEݳ\ACʱֱ\BDӽ\B5\BC\B6Ϊ\D0\C4\CC\F8
            if (active_cmd_.cmd == CMD_DISPLAY && !display_active) {
                // ȷ\B1\A3\B5\B1ǰ\C3\FC\C1\EE\C8\D4\CA\C7 DISPLAY\A3\A8δ\B1\BB\B8\DF\D3\C5\CFȼ\B6\B8\B2\B8ǣ\A9
                if (active_cmd_.cmd == CMD_DISPLAY) {
                    active_cmd_.cmd = CMD_HEARTBEAT;
                    std::fill(active_cmd_.data, active_cmd_.data + 4, 0.0f);
                    active_cmd_.priority = PRIORITY_HEARTBEAT;
                    active_cmd_.is_one_shot = false;
                }
            }

            cmd_to_send = active_cmd_;
        }

        send_command(cmd_to_send.cmd,
                     cmd_to_send.data[0], cmd_to_send.data[1],
                     cmd_to_send.data[2], cmd_to_send.data[3]);

        if (cmd_to_send.is_one_shot) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_cmd_.cmd == cmd_to_send.cmd &&
                active_cmd_.priority == cmd_to_send.priority) {
                recompute_base_command();
            }
        }
    }

    void recompute_base_command() {
        double timeout = this->get_parameter("vel_timeout").as_double();
        bool vel_active = ((this->now() - last_vel_time_).seconds() < timeout);

        if (vel_active) {
            // \CBٶ\C8\CF\FBϢ\C8\D4\D3\D0Ч\A3\AC\D5\FD\B3\A3\B7\A2\CB\CD\C6\DA\CD\FB\CBٶ\C8
            active_cmd_.cmd = CMD_VELOCITY;
            std::copy(desired_vel_, desired_vel_ + 4, active_cmd_.data);
            active_cmd_.priority = PRIORITY_VELOCITY;
            active_cmd_.is_one_shot = false;
        } else {
            // \CBٶ\C8\D2ѳ\ACʱ\A3\AC\BD\B5\BC\B6\B4\A6\C0\ED\A3\BA\CFȷ\A2\C1\E3\CBٰ\FC\D4\D9ת\D0\C4\CC\F8
            // \C5жϵ\B1ǰ\CAǷ\F1\D2Ѿ\AD\B4\A6\D3ڡ\B0һ\B4\CE\D0\D4\C1\E3\CBٹ\FD\B6ɡ\B1״̬
            bool in_zero_transition = (active_cmd_.cmd == CMD_VELOCITY && active_cmd_.is_one_shot);
            if (!in_zero_transition) {
                // \CA״γ\ACʱ\A3\BA\C9\E8\D6\C3һ\B8\F6һ\B4\CE\D0\D4\C1\E3\CB\D9\C3\FC\C1\CF´η\A2\CB\CD
                active_cmd_.cmd = CMD_VELOCITY;
                std::fill(active_cmd_.data, active_cmd_.data + 4, 0.0f);
                active_cmd_.priority = PRIORITY_VELOCITY;
                active_cmd_.is_one_shot = true;
            } else {
                // \D2Ѿ\AD\B7\A2\CB͹\FD\C1\E3\CBٹ\FD\B6ɰ\FC\A3\ACתΪ\D0\C4\CC\F8\B0\FC
                active_cmd_.cmd = CMD_HEARTBEAT;
                std::fill(active_cmd_.data, active_cmd_.data + 4, 0.0f);
                active_cmd_.priority = PRIORITY_HEARTBEAT;
                active_cmd_.is_one_shot = false;
            }
        }
    }

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

        std::ostringstream oss;
        oss << "Sending packet: ";
        for (int i = 0; i < 20; ++i) {
            oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << static_cast<int>(packet[i]) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

        if (!serial_->write(packet, 20)) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed");
        }
    }

    std::unique_ptr<SerialPort> serial_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr display_sub_; // \D0\C2\D4\F6
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_srv_;

    rclcpp::TimerBase::SharedPtr send_timer_;

    std::mutex mutex_;
    float desired_vel_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float display_data_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // չʾ\CA\FD\BE\DD
    rclcpp::Time last_vel_time_;
    rclcpp::Time last_display_time_;                   // \C9ϴ\CE\CAյ\BDչʾ\CA\FD\BEݵ\C4ʱ\BC\E4
    bool has_display_data_ = false;

    ActiveCommand active_cmd_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialProtocolNode>());
    rclcpp::shutdown();
    return 0;
}
