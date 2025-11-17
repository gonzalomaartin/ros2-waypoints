// pure_pursuit_odom.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "project_rom/msg/way_point_path.hpp"
#include "project_rom/WayPointPathTools.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/float64.hpp"
#include <memory>
#include <cmath>

class PurePursuitController : public rclcpp::Node {
    public:
        PurePursuitController() : Node("pure_pursuit_controller"), 
            x_(0.0), y_(0.0), theta_(0.0),
            x_rob_0_(0.0), y_rob_0_(0.0), theta_rob_0_(0.0),
            initialized_odom_(false), avoidance_curvature_(0.0)
        {
            this->declare_parameter<double>("tolerance", 0.25);
            this->declare_parameter<double>("Kp", 1.1);
            this->declare_parameter<double>("linear_speed", 0.15);

            this->get_parameter("tolerance", tolerance_);
            this->get_parameter("Kp", Kp_);
            this->get_parameter("linear_speed", linear_speed_);

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10,
                std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1)
            );
            subscription_path_ = this->create_subscription<project_rom::msg::WayPointPath>(
                "path", 10,
                std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1)
            );
            subscription_curvature_ = this->create_subscription<std_msgs::msg::Float64>(
                "avoidance_curvature", 10,
                std::bind(&PurePursuitController::curvature_callback, this, std::placeholders::_1)
            );

            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PurePursuitController::control_loop, this)
            );
        }

    private:
        void path_callback(const project_rom::msg::WayPointPath::SharedPtr msg) {
            path_tools_ = std::make_unique<WayPointPathTools>(*msg, tolerance_);
            RCLCPP_INFO(this->get_logger(), "Ruta recibida con %zu puntos.", msg->points.size());
        }
        
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
        
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
        
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
            if (!initialized_odom_) {
                x_rob_0_ = x;
                y_rob_0_ = y;
                theta_rob_0_ = yaw;
                initialized_odom_ = true;
            }
        
            x_ = x;
            y_ = y;
            theta_ = yaw;
        }

        void curvature_callback(const std_msgs::msg::Float64::SharedPtr msg) {
            avoidance_curvature_ = msg->data;
        }

        double normalize_angle(double angle) {
            while (angle > M_PI) angle -= 2 * M_PI;
            while (angle < -M_PI) angle += 2 * M_PI;
            return angle;
        }

        void control_loop() {
            if (!path_tools_) return;

            // Comprobar si hemos llegado al final del recorrido
            if (path_tools_->isFinished()) {
                stop_robot();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ruta completada.");
                return;
            }

            // Obtener siguiente punto
            geometry_msgs::msg::Point goal = path_tools_->getNextPoint(x_, y_);

            RCLCPP_INFO(this->get_logger(), "GOAL: x=%.2f y=%.2f", goal.x, goal.y);
            RCLCPP_INFO(this->get_logger(), "Start: x=%.2f y=%.2f", x_, y_);
            RCLCPP_INFO(this->get_logger(), "Orientation: theta=%.2f", theta_);

            // Control de orientación
            double angle_to_goal = std::atan2(goal.y - y_, goal.x - x_);
            double angle_error = normalize_angle(angle_to_goal - theta_);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = linear_speed_;
            cmd.angular.z = Kp_ * angle_error + avoidance_curvature_;;

            publisher_->publish(cmd);

        }

        void stop_robot() {
            geometry_msgs::msg::Twist stop;
            stop.linear.x = 0.0;
            stop.angular.z = 0.0;
            publisher_->publish(stop);
        }

        // Estado del robot
        double x_, y_, theta_;
        double x_rob_0_, y_rob_0_, theta_rob_0_;
        bool initialized_odom_;
        double avoidance_curvature_;
        
        // Parámetros
        double tolerance_, Kp_, linear_speed_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
        rclcpp::Subscription<project_rom::msg::WayPointPath>::SharedPtr subscription_path_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_curvature_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<WayPointPathTools> path_tools_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
