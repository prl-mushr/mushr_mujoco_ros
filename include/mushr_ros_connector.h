#ifndef _MUSHR_MUJOCO_ROS__MUSHR_ROS_CONNECTOR_H_
#define _MUSHR_MUJOCO_ROS__MUSHR_ROS_CONNECTOR_H_

#include "mujoco.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "types.h"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mushr_mujoco_ros/BodyState.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

namespace mushr_mujoco_ros {

class MuSHRROSConnector
{
  public:
    MuSHRROSConnector(ros::NodeHandle* nh, const YAML::Node& e);

    void mujoco_controller();
    void apply_control(mjData* d, mjtNum vel, mjtNum steering_angle);
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
    ros::Subscriber control_sub_;
    ros::Publisher imu_pub_;
    ros::Publisher velocity_pub_;
    int accel_idx_, accel_addr_;
    int gyro_idx_, gyro_addr_;
    int vel_idx_, vel_addr_;
    mjtNum accel_noise_, gyro_noise_, vel_noise_;

    std::string parent_body_name_;
    std::string base_link_site_name_;
    std::string body_name_;

    mjtNum velocity_;
    mjtNum steering_angle_;
    int velocity_ctrl_idx_;
    int steering_angle_ctrl_idx_;

    void control_cb(const ackermann_msgs::AckermannDriveStampedConstPtr&);
    void initpose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
    void init_sensors(void);
    void send_sensor_state(void);
    int get_velocimeter(const mjData*, geometry_msgs::Vector3&);
    void get_gyro(const mjData*, sensor_msgs::Imu&);
    void get_accel(const mjData*, sensor_msgs::Imu&);

    // Takes a name, and adds the unique modifier to find it in the Mujoco DOM
    std::string car_ref(std::string);
};

} // namespace mushr_mujoco_ros

#endif // _MUSHR_MUJOCO_ROS__MUSHR_ROS_CONNECTOR_H_
