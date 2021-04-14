//
// Path Planner
//

#ifndef PATH_PLANNING_PATHPLANNER_HPP
#define PATH_PLANNING_PATHPLANNER_HPP


#include <vector>
#include <set>

class PathPlanner {
public:
  struct Path {
    std::vector<double> px;
    std::vector<double> py;
  };
  struct Car {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
  };
  using SensorFusion = std::vector<std::vector<double>>;
  PathPlanner(std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y, std::vector<double> map_waypoints_s, std::vector<double> map_waypoints_dx, std::vector<double> map_waypoints_dy);

  Path calcNewPath(const Path& previous, double ends, double endd, const Car& car, const SensorFusion& sensorFusion);
private:
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  enum {
    S_KEEP_LANE,
    S_SWITCH_LANE
  } currentState;
  double ref_vel;
  double dst_vel;
  int dst_lane;
};


#endif //PATH_PLANNING_PATHPLANNER_HPP
