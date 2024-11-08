#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
    
class ScanUpdateNode : public rclcpp::Node 
{
public:
    ScanUpdateNode() : Node("scan_update")
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("raw_scan", 10, std::bind(&ScanUpdateNode::Callback_scan, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        RCLCPP_INFO(this->get_logger(), "Scan filtering has been started.");
    }
    
private:
    void Callback_scan(const sensor_msgs::msg::LaserScan scan_msg){
        auto filtered_scan_msg = scan_msg;

        for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
            float angle = scan_msg.angle_min + i * scan_msg.angle_increment;

            //ignore support_1 shadow region(215)(+-10 degree)
            if (angle >= 0.4475 && angle <= 0.7975) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }

            //ignore support_2 shadow region(325)(+-10 degree)
            if (angle >= 2.3725 && angle <= 2.7225) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }

            //ignore support_3 shadow region(35)(+-10 degree)
            if (angle >= -2.7025 && angle <= -2.3525) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }
            //ignore support_2 shadow region(145)(+-10 degree)
            if (angle >= -0.7775 && angle <= -0.4275) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }
        }

        // Publish the filtered laser scan
        publisher_->publish(filtered_scan_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanUpdateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}