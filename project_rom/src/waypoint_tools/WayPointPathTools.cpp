// WayPointPathTools.cpp

#include "project_rom/WayPointPathTools.hpp"

WayPointPathTools::WayPointPathTools(project_rom::msg::WayPointPath path_points, double tolerance)
  : path(std::move(path_points)), current_index(0), distance_tolerance(tolerance) {}

	geometry_msgs::msg::Point WayPointPathTools::getNextPoint(double x_rob, double y_rob) {
		if (current_index >= path.points.size()) {
			// Si hemos terminado el path, devolvemos el último punto
			return path.points.back();
		}

		geometry_msgs::msg::Point robot_pos;
			robot_pos.x = x_rob;
			robot_pos.y = y_rob;

	const geometry_msgs::msg::Point& target = path.points[current_index];
		if (distanceTo(robot_pos, target) < distance_tolerance) {
			// Avanzamos al siguiente de forma circular
			current_index = (current_index + 1) % path.points.size();
		}

		return path.points[current_index];
	}

	bool WayPointPathTools::isFinished() const {
		return current_index >= path.points.size() - 1;
	}

	double WayPointPathTools::distanceTo(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const {
		return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
	}
