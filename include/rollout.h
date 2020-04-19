#ifndef _MUSHR_MUJOCO_ROS__ROLLOUT_H_
#define _MUSHR_MUJOCO_ROS__ROLLOUT_H_

#include "mujoco.h"
#include "ros/ros.h"

namespace rollout {

void init(
    ros::NodeHandle* nh,
    std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*>* car_conn,
    std::map<std::string, mushr_mujoco_ros::BodyROSConnector*>* body_conn);
void destroy();

} // rollout

#endif // _MUSHR_MUJOCO_ROS__ROLLOUT_H_
