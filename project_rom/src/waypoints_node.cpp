// waypoints_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "project_rom/msg/way_point_path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

class WaypointsPublisher : public rclcpp::Node {
    public:
      WaypointsPublisher() : Node("waypoints_publisher"), path_sent_count_(0) {
        std::string path_yaml = this->declare_parameter<std::string>(
            "path_file", 
            ament_index_cpp::get_package_share_directory("project_rom") + "/config/path.yaml"
        );
    
        // Leer el archivo YAML
        try {
          YAML::Node config = YAML::LoadFile(path_yaml);
          if (!config["points"]) {
            RCLCPP_ERROR(this->get_logger(), "No se encontró la clave 'points' en el YAML");
            return;
          }
    
          for (const auto& point : config["points"]) {
            if (!point["x"] || !point["y"]) {
              RCLCPP_WARN(this->get_logger(), "Punto sin 'x' o 'y' en el YAML, se ignora.");
              continue;
            }
            geometry_msgs::msg::Point p;
            p.x = point["x"].as<double>();
            p.y = point["y"].as<double>();
            p.z = 0.0;
            path_msg_.points.push_back(p);
          }

          if (path_msg_.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "El path está vacío, no se publicará nada.");
            return;
          }
    
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Error al leer el archivo YAML: %s", e.what());
          return;
        }
    
        publisher_ = this->create_publisher<project_rom::msg::WayPointPath>("path", 10);
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          std::bind(&WaypointsPublisher::timer_callback, this)
        );
      }    

    private:
      void timer_callback() {
        // Publica el path 5 veces para asegurar que todos los nodos lo reciban
        if (path_sent_count_ < 5) {
            publisher_->publish(path_msg_);
            RCLCPP_INFO(this->get_logger(), "Path publicado (%d/5) con %zu puntos.", path_sent_count_+1, path_msg_.points.size());
            path_sent_count_++;
        }
      }

        rclcpp::Publisher<project_rom::msg::WayPointPath>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        project_rom::msg::WayPointPath path_msg_;
        int path_sent_count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointsPublisher>());
  rclcpp::shutdown();
  return 0;
}
