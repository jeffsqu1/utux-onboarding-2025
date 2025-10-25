#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using std::placeholders::_1;

class SonarSubscriber : public rclcpp::Node
{
  public:
    SonarSubscriber()
    : Node("sonar_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "sonar/data", 10, std::bind(&SonarSubscriber::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "SonarSubscriber node is listening to /sonar/data");
    }

  private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const // Sonar publishes array of 2
    {
        if (msg->data[0] >= 0 && msg->data[1] >= 0 ){
            RCLCPP_INFO(this->get_logger(), "Sonar distance data: [%.3f , %.3f] meters", (msg->data[0])*1.481/2, (msg->data[1])*1.481/2);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Sonar timed out");
        }
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarSubscriber>());
  rclcpp::shutdown();
  return 0;
}