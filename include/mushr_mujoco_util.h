#ifndef _MUSHR_MUJOCO_ROS__MUSHR_MUJOCO_UTIL_H
#define _MUSHR_MUJOCO_ROS__MUSHR_MUJOCO_UTIL_H

#include "geometry_msgs/Pose.h"
#include "mujoco.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

namespace mushr_mujoco_util {

void init_mj(const ros::NodeHandle* nh);
void load_config(
    const YAML::Node& e,
    std::string& body_name,
    std::string& pose_topic,
    std::string& initpose_topic,
    std::string& parent_body_name);

mjtNum mj_name2id_ordie(const mjModel* m, int type, const std::string& name);
mjtNum mj_name2id_ordie(const mjModel* m, int type, const char* name);

void mj2ros_site(
    const mjModel* m,
    mjData* d,
    const char* site_name,
    const char* body_name,
    geometry_msgs::Pose& ros_pose);
void ros2mj_site(
    const mjModel* m,
    mjData* d,
    const char* site_name,
    const char* body_name,
    geometry_msgs::Pose ros_pose);

void mj2ros_body(
    const mjModel* m, mjData* d, const char* name, geometry_msgs::Pose& ros_pose);
void ros2mj_body(
    const mjModel* m, mjData* d, const char* name, const geometry_msgs::Pose& ros_pose);

void reset(mjModel* m, mjData* d);
bool is_paused();

std::string pvt_name(const std::string& body_name, const std::string& topic_name);

} // mushr_mujoco_util

#endif // _MUSHR_MUJOCO_ROS__MUSHR_MUJOCO_UTIL_H
