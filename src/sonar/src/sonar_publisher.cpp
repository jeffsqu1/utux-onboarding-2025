#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

/**
 * @brief Sonar data publishing node in milliseconds(ms)
 * 
 * Publishes a 2 element float array to `sonar/data` that represent the time data the front/bottom sonar took
 */
class SonarPublisher : public rclcpp::Node {
  private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr timePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cleanTimePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr distPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mt19937 random_;
    std::uniform_real_distribution<float> distInitial_;
    std::uniform_real_distribution<float> dist_;
    std::uniform_real_distribution<float> step_;
    std::uniform_real_distribution<float> obstructChance_;
    std::uniform_int_distribution<int> noise_;
    std::uniform_int_distribution<int> publishInterval_; // [ ms ]

    float distForward_;
    float distBottom_;

    // ============ Priv Helper ============
    /**
     * @brief Callback function for when timer alarm goes off to publish sonar data
     */
    void timerCallback() {
      updateDistance(distForward_);
      updateDistance(distBottom_);

      // Make fake time data
      float timeForward = distForward_;
      float timeBottom = distBottom_;

      if(obstructChance_(random_) < 0.05f) {
        timeForward =  0.7f + dist_(random_)*0.1f;
      }

      if(obstructChance_(random_) < 0.05f) {
        timeBottom =  0.3f + dist_(random_)*0.1f;
      }

      timeForward = d2t(timeForward);
      timeBottom = d2t(timeBottom);

      float noisyTimeForward = timeForward + noise_(random_);
      float noisyTimeBottom = timeBottom + noise_(random_);

      auto timeMessage = std_msgs::msg::Float32MultiArray();
      auto cleanTimeMessage = std_msgs::msg::Float32MultiArray();
      auto distMessage = std_msgs::msg::Float32MultiArray();
      timeMessage.data = {noisyTimeForward, noisyTimeBottom};
      cleanTimeMessage.data = {timeForward, timeBottom};
      distMessage.data = {distForward_, distBottom_};

      RCLCPP_INFO(this->get_logger(), 
      "time - [ %3.3f, %3.3f ]", noisyTimeForward, noisyTimeBottom);

      timePublisher_->publish(timeMessage);
      cleanTimePublisher_->publish(cleanTimeMessage);
      distPublisher_->publish(distMessage);

      setNextPublish();
    }

    /**
     * @brief Set next publish itime
     */
    void setNextPublish() {
      int delay = publishInterval_(random_);

      timer_ = this->create_wall_timer(std::chrono::milliseconds((delay)), std::bind(&SonarPublisher::timerCallback, this));
    }

    /**
     * @brief Update the UUV distances
     */
    void updateDistance(float &d) {
      d += step_(random_);

      if(d < 0.2f) { d = 0.2f; }
    }

    /**
     * @brief Get time from distance
     * @return Roundtrip time of sonar pulse. -1 if sonar's max range is exceeded
     */
    float d2t(float d) {
      if(d > 23){
        return -1.0f;
      }

      return (2.0f * 1000.0f * d)/(1481.0f);
    }
    
  public:
    // ============ Constructor ============
    /**
     * @brief Default publisher contructor
     */
    SonarPublisher() : Node("sonar_publisher") {
      timePublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sonar/data", 10);
      cleanTimePublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sonar/data_clean", 10);
      distPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sonar/data_distance", 10);
      
      random_.seed(std::random_device{}());

      dist_ = std::uniform_real_distribution<float>(0.5f, 26.0f);  // [ m ]
      distInitial_ = std::uniform_real_distribution<float>(4.5f, 11.3f);  // [ m ]
      step_ = std::uniform_real_distribution<float>(-0.02, 0.08);  // [ m ]

      obstructChance_ = std::uniform_real_distribution<float>(0.0f, 1.0f);
      noise_ = std::uniform_int_distribution<int>(-2.5, 2.5); //  [ ms ]

      publishInterval_ = std::uniform_int_distribution<int>(200, 1000);  // [ ms ]

      distForward_ = distInitial_(random_);
      distBottom_ = distInitial_(random_);

      setNextPublish();
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarPublisher>());
  rclcpp::shutdown();
  return 0;
}