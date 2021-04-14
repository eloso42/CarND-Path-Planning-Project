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
constexpr double maxVel = 22.3; //a little bit less than 50mph
constexpr double maxAcc = 9.5; //some buffer to max allowed 10.0
constexpr size_t pathLength = 50; //number of path points to generate
constexpr int numLanes = 3; //(max) number of lanes

/**
 * Calculates the next velocity (i.e. distance for the next path point)
 * \param refVel reference (current) velocity
 * \param dstVel destination velocity
 * \return next velocity
 */
double nextVel(double refVel, double dstVel) {
  double acc = dstVel - refVel;
  if (abs(acc)>maxAcc) {
    acc = maxAcc * acc / abs(acc);
  }
  return refVel + acc * 0.02;
}

/**
 * converts a d coordinate to lane number
 *
 * \param d
 * \return lane number
 */
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
    currentState(S_KEEP_LANE),
    ref_vel(0),
    dst_vel(maxVel),
    dst_lane(1) {

}

PathPlanner::Path PathPlanner::calcNewPath(const Path& previous, double ends, double endd, const Car& car, const SensorFusion& sensorFusion) {
  Path next;

  vector<double> ptsx;
  vector<double> ptsy;

  //set reference values and push first spline points to ptsx/y (more or less copied from Q&A video)
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
    ptsx.push_back(previous.px[0]);
    ptsy.push_back(previous.py[0]);
    ptsx.push_back(previous.px[path_size-2]);
    ptsy.push_back(previous.py[path_size-2]);
    ptsx.push_back(previous.px[path_size-1]);
    ptsy.push_back(previous.py[path_size-1]);
  }
  //cout << "ps=" << path_size << ", yaw = " << car.yaw << ", refVel = " << ref_vel << endl;

  // evaluate sensor data
  array<double, numLanes> maxVels;
  std::fill(maxVels.begin(), maxVels.end(), maxVel);
  array<bool, numLanes> isFree;
  std::fill(isFree.begin(), isFree.end(), true);
  double minDist = 1e9;
  double minCars = 1e9;
  int minCarLane = 3;
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
      if (s < minCars) {
        minCars = s;
        minCarLane = lane;
      }
    }
    if (isFree[lane] && ((s + 4) > car.s) && (s < (car.s+20))) {
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

  // behavioral planner state machine
  switch(currentState) {
    case S_KEEP_LANE: {
      //check if it makes sense to switch lanes
      int nextLane = maxSpeedLane;
      if (abs(nextLane - ref_lane) > 1) {
        nextLane = (nextLane + ref_lane) / 2;
      }
      if (dst_lane != nextLane && isFree[nextLane] && isFree[maxSpeedLane] && maxVels[maxSpeedLane] > maxVels[dst_lane] + 0.5 && minDist < 60) {
        dst_lane = nextLane;
        currentState = S_SWITCH_LANE;
      }
      break;
    }
    case S_SWITCH_LANE:
      //wait until lane is switched and switch back to KEEP_LANE then
      if (ref_lane == dst_lane) {
        currentState = S_KEEP_LANE;
      }
      break;
  }

  // determine destination velocity
  if (minDist < 30) {
    if (minDist < 25) {
      dst_vel = maxVels[dst_lane]-2;
    } else {
      dst_vel = maxVels[dst_lane];
    }
  } else {
    dst_vel = maxVel;
  }

  /*
  cout << "min dist = " << std::setprecision(4) << setw(7) << minDist
       << ", max vels = " << setw(5) <<  maxVels[0] << "(" << isFree[0]
       << "), " << setw(5) << maxVels[1] << "(" << isFree[1]
       << "), " << setw(5) << maxVels[2] << "(" << isFree[2]
       << "), maxSpeed = " << maxSpeedLane << ", current = " << dst_lane << ", ref = " << ref_lane << ", dstVel = " << dst_vel
       << ", cars = " << car.s << ", mins = " << minCars << ", l = " << minCarLane << endl;
       */

  //generate control points for spline (again more or less copied from Q&A video)
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