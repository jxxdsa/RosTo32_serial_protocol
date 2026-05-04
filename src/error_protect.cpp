#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>

class LandTriggerNode : public rclcpp::Node
{
public:
    LandTriggerNode()
    : Node("land_trigger_node"), landed_(false), requesting_(false)
    {
        // 创建服务客户端
        client_ = this->create_client<std_srvs::srv::Trigger>("/serial_protocol_node/land");

        // 等待服务可用
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for landing service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for landing service to become available...");
        }

        // 订阅里程计话题
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio2/lio_odom", 10,
            std::bind(&LandTriggerNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Land trigger node started, monitoring odometry.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 已触发或正在请求服务，则不再处理
        if (landed_ || requesting_) {
            return;
        }

        // 提取位置和线速度
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;
        double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

        // 判断触发条件
        if (std::abs(x) > 10.0 || std::abs(y) > 10.0 || std::abs(z) > 10.0 || speed > 0.5) {
            RCLCPP_WARN(this->get_logger(),
                "Trigger condition met: pos(%.2f, %.2f, %.2f), speed=%.2f. Calling landing service.",
                x, y, z, speed);

            // 若服务未就绪，给出警告但不触发
            if (!client_->service_is_ready()) {
                RCLCPP_ERROR(this->get_logger(), "Landing service is not ready.");
                return;
            }

            requesting_ = true;
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

            // 异步调用降落服务
            auto future = client_->async_send_request(
                request,
                [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
                    if (result.get()->success) {
                        RCLCPP_INFO(this->get_logger(), "Landing service succeeded: %s",
                                    result.get()->message.c_str());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Landing service failed: %s",
                                     result.get()->message.c_str());
                    }
                    landed_ = true;
                    requesting_ = false;
                });
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    bool landed_;
    bool requesting_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LandTriggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}