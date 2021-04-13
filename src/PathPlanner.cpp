//
// Path Planner
//

#include "PathPlanner.hpp"
#include "helpers.h"
#include "spline.h"

#include <array>
#include <iomanip>
#include <iostream>

using std::array;
using std::cout;
using std::endl;
using std::max;
using std::min;
using std::move;
using std::setw;
using std::vector;

namespace {
constexpr double maxVel = 22.3; //roughly 50mph
constexpr double maxAcc = 9.5; //some buffer to 10.0
constexpr size_t pathLength = 50;
constexpr int numLanes = 3;

double nextVel(double refVel, double dstVel) {
  double acc = dstVel - refVel;
  if (abs(acc)>maxAcc) {
    acc = maxAcc * acc / abs(acc);
  }
  return refVel + acc * 0.02;
}

int d2lane(double d) {
  return min(max(static_cast<int>(floor(d / 4)), 0), numLanes-1);
}

}

PathPlanner::PathPlanner(
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s,
    std::vector<double> map_waypoints_dx,
    std::vector<double> map_waypoints_dy) :
    map_waypoints_x(move(map_waypoints_x)),
    map_waypoints_y(move(map_waypoints_y)),
    map_waypoints_s(move(map_waypoints_s)),
    map_waypoints_dx(move(map_waypoints_dx)),
    map_waypoints_dy(move(map_waypoints_dy)),
    dst_vel(maxVel),
    dst_lane(1) {

  ref_vel = 0;
}

PathPlanner::Path PathPlanner::calcNewPath(const Path& previous, double ends, double endd, const Car& car, const SensorFusion& sensorFusion) {
  Path next;

  vector<double> ptsx;
  vector<double> ptsy;

  size_t path_size = previous.px.size();
  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = deg2rad(car.yaw);
  double ref_s = car.s;
  int ref_lane = d2lane(car.d);
  if (path_size < 2) {
    double prev_car_x = car.x - cos(ref_yaw);
    double prev_car_y = car.y - sin(ref_yaw);
    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);
    ptsx.push_back(car.x);
    ptsy.push_back(car.y);
  } else {
    ref_x = previous.px[path_size-1];
    ref_y = previous.py[path_size-1];
    ref_s = ends;
    ref_lane = d2lane(endd);
    double ref_x_prev = previous.px[path_size-2];
    double ref_y_prev = previous.py[path_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(previous.px[path_size-2]);
    ptsy.push_back(previous.py[path_size-2]);
    ptsx.push_back(previous.px[path_size-1]);
    ptsy.push_back(previous.py[path_size-1]);
  }
  //cout << "ps=" << path_size << ", yaw = " << car.yaw << ", refVel = " << ref_vel << endl;

  array<double,numLanes> maxVels;
  std::fill(maxVels.begin(), maxVels.end(), maxVel);
  array<bool, numLanes> isFree;
  std::fill(isFree.begin(), isFree.end(), true);
  double minDist = 1e9;
  for (auto& item : sensorFusion) {
    int lane = d2lane(item[6]);
    double vx = item[3];
    double vy = item[4];
    double v = sqrt(vx*vx+vy*vy);
    double s = item[5];
    if (s + 4 > car.s && s < car.s + 90) {
      if (maxVels[lane] > v) {
        maxVels[lane] = v;
      }
    }
    if (isFree[lane] && s + 4 > car.s && s < car.s+10) {
      isFree[lane] = false;
    }
    if (lane == dst_lane) {
      double dist = s - car.s;
      if (dist > 0 && dist < minDist) {
        minDist = dist;
      }
    }
  }
  int maxSpeedLane = std::max_element(maxVels.begin(), maxVels.end()) - maxVels.begin();
  if (dst_lane != maxSpeedLane && isFree[maxSpeedLane] && maxVels[maxSpeedLane] > maxVels[dst_lane] + 0.5) {
    dst_lane = maxSpeedLane;
  }
  if (abs(dst_lane - ref_lane) > 1) {
    dst_lane = (dst_lane + ref_lane) / 2;
  }
  if (minDist < 30) {
    if (minDist < 25) {
      dst_vel = maxVels[dst_lane]-2;
    } else {
      dst_vel = maxVels[dst_lane];
    }
  } else {
    dst_vel = maxVel;
  }

  cout << "min dist = " << std::setprecision(4) << setw(7) << minDist
       << ", max vels = " << setw(5) <<  maxVels[0] << "(" << isFree[0]
       << "), " << setw(5) << maxVels[1] << "(" << isFree[0]
       << "), " << setw(5) << maxVels[2] << "(" << isFree[0]
       << "), maxSpeed = " << maxSpeedLane << ", current = " << dst_lane << ", dstVel = " << dst_vel << endl;

  auto wp0 = getXY(ref_s + 30, 2+4*dst_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  auto wp1 = getXY(ref_s + 60, 2+4*dst_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  auto wp2 = getXY(ref_s + 90, 2+4*dst_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(wp0[0]);
  ptsx.push_back(wp1[0]);
  ptsx.push_back(wp2[0]);
  ptsy.push_back(wp0[1]);
  ptsy.push_back(wp1[1]);
  ptsy.push_back(wp2[1]);

  for (size_t i = 0; i < ptsx.size(); ++i) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(-ref_yaw)-shift_y*sin(-ref_yaw));
    ptsy[i] = (shift_x * sin(-ref_yaw)+shift_y*cos(-ref_yaw));
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  double dist_inc = 0.4;

  next.px.resize(previous.px.size());
  next.py.resize(previous.py.size());
  std::copy(previous.px.begin(), previous.px.end(), next.px.begin());
  std::copy(previous.py.begin(), previous.py.end(), next.py.begin());

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x+target_y*target_y);
  double x_add_on = 0.0;

  for (size_t i = path_size; i < pathLength; ++i) {
    ref_vel = nextVel(ref_vel, dst_vel);
    double N = (target_dist/(0.02*ref_vel));
    double x_p = x_add_on+target_x / N;
    double y_p = s(x_p);
    x_add_on = x_p;

    double x_ref = x_p;
    double y_ref = y_p;

    x_p = x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw);
    y_p = x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw);

    x_p += ref_x;
    y_p += ref_y;

    next.px.push_back(x_p);
    next.py.push_back(y_p);
  }

  return next;
}