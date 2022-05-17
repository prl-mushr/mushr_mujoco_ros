#ifndef _MUSHR_MUJOCO_ROS__MUSHR_ROS_CONNECTOR_H_
#define _MUSHR_MUJOCO_ROS__MUSHR_ROS_CONNECTOR_H_

#include "mujoco.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "body_ros_connector.h"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

namespace mushr_mujoco_ros {

class MuSHRROSConnector : public BodyROSConnector
{
  public:
    MuSHRROSConnector(ros::NodeHandle* nh, const YAML::Node& e);

    void mujoco_controller();
    void send_state();
    void set_body_state(mushr_mujoco_ros::BodyState& bs);

  protected:
    ros::Subscriber control_sub_;
    ros::Publisher imu_pub_;
    ros::Publisher velocity_pub_;
    int accel_idx_, accel_addr_;
    int gyro_idx_, gyro_addr_;
    int vel_idx_, vel_addr_;
    mjtNum accel_noise_, gyro_noise_, vel_noise_;

    mjtNum velocity_;
    mjtNum acceleration_;
    mjtNum steering_angle_;
    int velocity_ctrl_idx_;
    int acceleration_ctrl_idx_;
    int steering_angle_ctrl_idx_;
    bool use_accel_control_ = false;

    void control_cb(const ackermann_msgs::AckermannDriveStampedConstPtr&);
    void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
    void init_sensors(void);
    void send_sensor_state(void);
    int get_velocimeter(const mjData*, geometry_msgs::Vector3&);
    void get_gyro(const mjData*, sensor_msgs::Imu&);
    void get_accel(const mjData*, sensor_msgs::Imu&);

    // Takes a name, and adds the unique modifier to find it in the Mujoco DOM
    std::string car_ref(std::string);
}; // class MuSHRROSConnector

} // namespace mushr_mujoco_ros

#endif // _MUSHR_MUJOCO_ROS__MUSHR_ROS_CONNECTOR_H_
