// WayPointPathTools.hpp

#ifndef WAYPOINTPATHTOOLS_HPP
#define WAYPOINTPATHTOOLS_HPP

#include "project_rom/msg/way_point_path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>

class WayPointPathTools {
    public:
        // Constructor con ruta y tolerancia de distancia
        WayPointPathTools(project_rom::msg::WayPointPath path_points, double tolerance);

        // Devuelve el siguiente punto objetivo en la ruta
        geometry_msgs::msg::Point getNextPoint(double x_rob, double y_rob);

        // Indica si se ha alcanzado el final del recorrido
        bool isFinished() const;

    private:
        project_rom::msg::WayPointPath path;
        size_t current_index;
        double distance_tolerance;

        // Calcula distancia entre dos puntos
        double distanceTo(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const;
};

#endif // WAYPOINTPATHTOOLS_HPP
