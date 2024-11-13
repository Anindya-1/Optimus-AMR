#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
    
class ScanUpdateNode : public rclcpp::Node 
{
public:
    ScanUpdateNode() : Node("scan_update")
    {
        rclcpp::QoS qos_profile(10);
        // qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/pf/scan", qos_profile, std::bind(&ScanUpdateNode::Callback_scan, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);
        RCLCPP_INFO(this->get_logger(), "Scan filtering has been started.");
    }
    
private:
    void Callback_scan(const sensor_msgs::msg::LaserScan scan_msg){
        auto filtered_scan_msg = scan_msg;

        for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
            float angle = scan_msg.angle_min + i * scan_msg.angle_increment;

            //ignore support_1 shadow region(90)(+-5 degree)
            if (angle >= (65*0.01745) && angle <= (85*0.01745)) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }

            //ignore support_2 shadow region(325)(+-10 degree)
            if (angle >= (125*0.01745) && angle <= (140*0.01745)) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }

            //ignore support_3 shadow region(35)(+-10 degree)
            if (angle >= (155*0.01745) && angle <= (170*0.01745)) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }

            //ignore support_4 shadow region(145)(+-10 degree)
            if (angle >= (-170*0.01745) && angle <= (-155*0.01745)) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }

            //ignore support_5 shadow region(145)(+-10 degree)
            if (angle >= (-140*0.01745) && angle <= (-125*0.01745)) {
            // Set the filtered range value to a specific value or NaN
                filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
            }

            //ignore support_6 shadow region(-5)(+-5 degree)
            if (angle >= (-90*0.01745) && angle <= (-65*0.01745)) {
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