#ifndef _MUSHR_MUJOCO_ROS__BODY_ROS_CONNECTOR_H_
#define _MUSHR_MUJOCO_ROS__BODY_ROS_CONNECTOR_H_

#include <string>

#include "mujoco.h"
#include "ros/ros.h"
#include "types.h"
#include "yaml-cpp/yaml.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mushr_mujoco_ros/BodyState.h>

namespace mushr_mujoco_ros {

class BodyROSConnector
{
  public:
    BodyROSConnector(ros::NodeHandle* nh, const YAML::Node& e);

    void ros_send_state();
    void set_body_state(mushr_mujoco_ros::BodyState& bs);
    void set_pose(const geometry_msgs::Pose& pose);
    void set_pose(const mushr_mujoco_ros::PoseTuple& pose);
    void get_pose(mjModel* m, mjData* d, mushr_mujoco_ros::PoseTuple& pose);

    const std::string& body_name()
    {
        return body_name_;
    }

  private:
    ros::NodeHandle* nh_;
    ros::Publisher pose_pub_;
    ros::Subscriber initpose_sub_;

    std::string body_name_;
    std::string parent_body_name_;
    int body_id_;

    void initpose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);

    // couldn't get private node handles working, so kludge.
    std::string pvt_name(std::string name);
};

} // namespace mushr_mujoco_ros

#endif // _MUSHR_MUJOCO_ROS__BODY_ROS_CONNECTOR_H_
