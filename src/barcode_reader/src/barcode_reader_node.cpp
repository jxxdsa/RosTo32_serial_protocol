// barcode_reader_node.cpp
#include "barcode_reader/QRcodeScanner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>

class BarcodeReaderNode : public rclcpp::Node
{
public:
    BarcodeReaderNode() : Node("barcode_reader_node"), scanner(new ZbarBarcodeScanner)
    {
        // 订阅压缩图像话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image", 10,
            std::bind(&BarcodeReaderNode::image_callback, this, std::placeholders::_1));

        // 发布识别后的条码字符串
        barcode_pub_ = this->create_publisher<std_msgs::msg::String>("/barcode", 10);

        RCLCPP_INFO(this->get_logger(), "Barcode Reader Node started, subscribing to /image");
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (msg->data.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received empty image");
            return;
        }

        // 解码压缩图像为 cv::Mat
        cv::Mat raw_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (raw_img.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
            return;
        }

        // 耗时测量（可选）
        auto start = std::chrono::high_resolution_clock::now();
        std::string barcode = scanner->getBarcode(raw_img);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (!barcode.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Barcode detected: %s (took %ld ms)",
                        barcode.c_str(), duration);

            std_msgs::msg::String msg_out;
            msg_out.data = barcode;
            barcode_pub_->publish(msg_out);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No barcode found (took %ld ms)", duration);
        }
    }

    std::unique_ptr<ZbarBarcodeScanner> scanner;   // 可以使用共享指针
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr barcode_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BarcodeReaderNode>());
    rclcpp::shutdown();
    return 0;
}