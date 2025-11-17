// obstacle_avoider.cpp (versión adaptada)

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include <string>
#include <algorithm>

class ObstacleAvoider : public rclcpp::Node {
  public:
    ObstacleAvoider() : Node("obstacle_avoider"), 
      sonar_distances_(4, max_range_),
      received_(4, false)
    {
      sonar_topics_ = {"/sonar0", "/sonar1", "/sonar2", "/sonar3"};

      for (size_t i = 0; i < sonar_topics_.size(); ++i) {
        subscriptions_.push_back(
          this->create_subscription<sensor_msgs::msg::Range>(
            sonar_topics_[i], 10,
            [this, i](sensor_msgs::msg::Range::SharedPtr msg) {
              sonar_distances_[i] = msg->range;
              received_[i] = true;
            }
          )
        );
      }

      publisher_ = this->create_publisher<std_msgs::msg::Float64>("/avoidance_curvature", 10);

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObstacleAvoider::evaluate_avoidance, this)
      );
    }

  private:
    void evaluate_avoidance() {

      // Verifica que todos los sensores hayan recibido datos
      if (std::any_of(received_.begin(), received_.end(), [](bool v){ return !v; })) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Esperando datos de todos los sonares...");
        return;
      }

      // Curvatura base = 0 (recto). Rango de salida: [-1, 1]
      double curvature = 0.0;
      double front_right = sonar_distances_[0];  // sonar0
      double back_right = sonar_distances_[1]; // sonar1
      double front_left = sonar_distances_[2];   // sonar2
      double back_left = sonar_distances_[3];  // sonar3

      // Influencias proporcionales a la cercanía (más cerca, más impacto)
      double infl_fr = (front_right < influence_threshold_) ? (1.0 - front_right / max_range_) : 0.0;
      double infl_fl = (front_left  < influence_threshold_) ? (1.0 - front_left  / max_range_) : 0.0;
      double infl_br = (back_right  < influence_threshold_) ? (1.0 - back_right  / max_range_) * 0.5 : 0.0;
      double infl_bl = (back_left   < influence_threshold_) ? (1.0 - back_left   / max_range_) * 0.5 : 0.0;

      // Ajuste de curvatura:
      // - Obstáculo a la derecha → curvar a la izquierda (+)
      // - Obstáculo a la izquierda → curvar a la derecha (-)
      curvature = (infl_fr + infl_br) - (infl_fl + infl_bl);
      curvature = std::clamp(curvature, -1.5, 1.5);

      std_msgs::msg::Float64 msg;
      msg.data = -curvature;
      publisher_->publish(msg);

      if (std::abs(curvature) > 0.1) {
        RCLCPP_INFO(this->get_logger(), "Curvatura de evasión: %.2f", msg.data);
      }
    }

    std::vector<std::string> sonar_topics_;
    std::vector<double> sonar_distances_;
    std::vector<bool> received_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> subscriptions_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    const double max_range_ = 0.2;              // Rango máximo de los sensores
    const double influence_threshold_ = 0.2;    // Distancia a partir de la cual afecta
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoider>());
  rclcpp::shutdown();
  return 0;
}
