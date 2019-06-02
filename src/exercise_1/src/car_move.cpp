#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <sstream>
#include <map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <fstream>
#include <cassert>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

struct Point {
  Point(double _x = 0, double _y = 0) : x(_x), y(_y) {}
  double mod()const {
    return hypot(x, y);
  }
  double dir()const {
    return atan2(y, x);
  }
  bool operator==(const Point& p) const {
    return x == p.x && y == p.y;
  }
  double x, y;
};

struct GridCell {
  std::string name;
  int value;
  Point point;
  GridCell() {
    name = "";
    value = 0;
  }
};

std::vector<std::string> pathPlan(const std::vector<std::vector<GridCell> >& grid_map, std::string start_point, std::string target_point);

std::vector<geometry_msgs::PoseStamped> genPathDisplay(const std::vector<std::string>& plan_result, const std::vector< std::vector<GridCell> >& grid_map);

bool findIndexWithName(const std::vector< std::vector<GridCell> >& grid_map, const std::string& name, int& cx, int& cy);

std::string int2str(int i);

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_display", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(30);
  std::string start_name = "A0_0";
  std::string end_name = "A9_2"; // "A9_2" "A2_8"
  std::string file_path_name = "/home/handsomexian/tutorials_ws/src/exercise_ec_ws/src/exercise_1/scripts/obsData.txt";
  int x_length = 10;
  int y_length = 10;
  std::vector< std::vector<GridCell> > grid_map(x_length);
  double mapResolution = 0.5;
  for(int i = 0; i < x_length; ++i) {
    for(int j = 0; j < y_length; ++j) {
      GridCell grid_cell;
      grid_cell.name = "A" + int2str(i) + "_" + int2str(j);
      grid_cell.value = 0;
      grid_cell.point.x = i * mapResolution;
      grid_cell.point.y = j * mapResolution;
      grid_map[i].push_back(grid_cell);
    }
  }
  // read obstacle information file
  std::ifstream in;
  in.open(file_path_name.c_str());
  std::string line;
  std::vector<std::string> obsData;
  if (in.is_open()) {
    while (getline (in, line)) {
      obsData.push_back(line);
    }
  } else {
    std::cout << "No such obstacle information file" << std::endl;
  }
  for (int i = 0; i < obsData.size(); i++) {
    int num = atoi(obsData[i].c_str());
    int x_index =  num / x_length;
    int y_index = num % x_length;
    grid_map[x_index][y_index].value = 1;
  }
  std::vector<std::string> plan_result = pathPlan(grid_map, start_name, end_name);
  std::vector<geometry_msgs::PoseStamped> poses_display = genPathDisplay(plan_result, grid_map);

  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = "base_link";

  nav_msgs::Path path_display;
  path_display.header.stamp = ros::Time::now();
  path_display.header.frame_id = "map";
  path_display.poses = poses_display;

  // wait for input
  while(std::getchar() != 's') {
    sleep(1);
  }

  int i = 0;
  while (ros::ok()) {
    // update transform
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = poses_display[i].pose.position.x;
    odom_trans.transform.translation.y = poses_display[i].pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = poses_display[i].pose.orientation;

    // publish transform
    path_pub.publish(path_display);
    broadcaster.sendTransform(odom_trans);
    if (i < poses_display.size() - 1) {
      i += 1;
    } else {
      i += 0;
    }
    // This will adjust as needed per iteration
    loop_rate.sleep();
  }
  return 0;
}

std::vector<geometry_msgs::PoseStamped> genPathDisplay(const std::vector<std::string>& plan_result, const std::vector< std::vector<GridCell> >& grid_map) {
  std::vector<geometry_msgs::PoseStamped> poses_display;
  poses_display.clear();
  if (plan_result.size() > 1) {
    for (int i = 0; i < plan_result.size() - 1; i++) {
      Point current_pose, next_pose;
      int cx, cy;
      findIndexWithName(grid_map, plan_result[i], cx, cy);
      current_pose = grid_map[cx][cy].point;
      findIndexWithName(grid_map, plan_result[i + 1], cx, cy);
      next_pose = grid_map[cx][cy].point;

      double delta_x = next_pose.x - current_pose.x;
      double delta_y = next_pose.y - current_pose.y;

      double angle = atan2(delta_y, delta_x);

      int segment_size = sqrt(delta_x * delta_x + delta_y * delta_y) / 0.1;
      if (segment_size == 0) {
        segment_size = 1;
      }
      for (int i = 0; i < segment_size + 1; i++) {
        geometry_msgs::PoseStamped temp;
        temp.header.stamp = ros::Time::now();
        temp.header.frame_id = "map";
        temp.pose.position.x = current_pose.x + delta_x * i / segment_size;
        temp.pose.position.y = current_pose.y + delta_y * i / segment_size;
        temp.pose.position.z = 0;
        temp.pose.orientation = tf::createQuaternionMsgFromYaw(angle - 1.57);
        poses_display.push_back(temp);
      }
    }
  } else {
    int cx, cy;
    findIndexWithName(grid_map, plan_result[0], cx, cy);
    Point start_pose = grid_map[cx][cy].point;
    geometry_msgs::PoseStamped temp;
    temp.header.stamp = ros::Time::now();
    temp.header.frame_id = "map";
    temp.pose.position.x = start_pose.x;
    temp.pose.position.y =  start_pose.y;
    //ROS_INFO_STREAM("start x: " << start_pose.x);
    //ROS_INFO_STREAM("start y: " << start_pose.y);
    temp.pose.position.z = 0;
    temp.pose.orientation = tf::createQuaternionMsgFromYaw(1.57);
    poses_display.push_back(temp);
  }
  return poses_display;
}

std::vector<std::string> pathPlan(const std::vector< std::vector<GridCell> >& grid_map, std::string start_point, std::string target_point) { // use dijkstra algorithm to plan path
  ROS_INFO_STREAM("start point:" << start_point);
  ROS_INFO_STREAM("target point:" << target_point);
  std::vector<std::string> plan_result;
  int map_size_x = grid_map.size();
  int map_size_y = 0;
  if(map_size_x > 0) {
    map_size_y = grid_map[0].size();
  }
  const int max_value = 10000;
  std::vector<bool> close_list(map_size_x * map_size_y, false);
  std::vector<int> traceback(map_size_x * map_size_y, -1);
  std::vector<int> dist(map_size_x * map_size_y, max_value);
  // init the start point
  int cx, cy;
  findIndexWithName(grid_map, start_point, cx, cy);
  int u = cx * map_size_y + cy;
  dist[u] = 0;
  if(cx - 1 >= 0) {
    dist[(cx - 1) * map_size_y + cy] = 1;
    traceback[(cx - 1) * map_size_y + cy] = u;
  }
  if(cx + 1 < map_size_x) {
    dist[(cx + 1) * map_size_y + cy] = 1;
    traceback[(cx + 1) * map_size_y + cy] = u;
  }
  if(cy - 1 >= 0) {
    dist[cx * map_size_y + (cy - 1)] = 1;
    traceback[cx * map_size_y + (cy - 1)] = u;
  }
  if(cy + 1 < map_size_y) {
    dist[cx * map_size_y + (cy + 1)] = 1;
    traceback[cx * map_size_y + (cy + 1)] = u;
  }
  close_list[u] = true;

  // the main loop of dijkstra
  for(int i = 1; i < map_size_x * map_size_y; ++i) {
    int min_dist = max_value;
    for(int j = 0; j < map_size_x * map_size_y; ++j) {
      if(grid_map[j / map_size_y][j % map_size_y].value == 0 && !close_list[j] && dist[j] < min_dist) {
        min_dist = dist[j];
        u = j;
      }
    }
    cx = u / map_size_y;
    cy = u % map_size_y;
    close_list[u] = true;

    if(cx - 1 >= 0 && !close_list[(cx - 1) * map_size_y + cy]) {
      if(dist[(cx - 1) * map_size_y + cy] > min_dist + 1) {
        dist[(cx - 1) * map_size_y + cy] = min_dist + 1;
        traceback[(cx - 1) * map_size_y + cy] = u;
      }
    }
    if(cx + 1 < map_size_x && !close_list[(cx + 1) * map_size_y + cy]) {
      if(dist[(cx + 1) * map_size_y + cy] > min_dist + 1) {
        dist[(cx + 1) * map_size_y + cy] = min_dist + 1;
        traceback[(cx + 1) * map_size_y + cy] = u;
      }
    }
    if(cy - 1 >= 0 && !close_list[cx * map_size_y + (cy - 1)]) {
      if(dist[cx * map_size_y + (cy - 1)] > min_dist + 1) {
        dist[cx * map_size_y + (cy - 1)] = min_dist + 1;
        traceback[cx * map_size_y + (cy - 1)] = u;
      }
    }
    if(cy + 1 < map_size_y && !close_list[cx * map_size_y + (cy + 1)]) {
      if(dist[cx * map_size_y + (cy + 1)] > min_dist + 1) {
        dist[cx * map_size_y + (cy + 1)] = min_dist + 1;
        traceback[cx * map_size_y + (cy + 1)] = u;
      }
    }
  }

  //traceback the plan path
  findIndexWithName(grid_map, target_point, cx, cy);
  u = cx * map_size_y + cy;
  plan_result.push_back(grid_map[cx][cy].name);
  while(dist[u] != 0) {
    u = traceback[u];
    cx = u / map_size_y;
    cy = u % map_size_y;
    plan_result.push_back(grid_map[cx][cy].name);
  }
  std::reverse(plan_result.begin(), plan_result.end());
  return plan_result;
}

bool findIndexWithName(const std::vector< std::vector<GridCell> >& grid_map, const std::string& name, int& cx, int& cy) {
  for(int i = 0; i < grid_map.size(); ++i) {
    for(int j = 0; j < grid_map[0].size(); ++j) {
      if(name == grid_map[i][j].name) {
        cx = i;
        cy = j;
        return true;
      }
    }
  }
  return false;
}

std::string int2str(int i) {
  std::string s;
  std::stringstream ss(s);
  ss << i;
  return ss.str();
}
