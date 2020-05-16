#include "body_ros_connector.h"
#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "types.h"

namespace mushr_mujoco_ros {

BodyROSConnector::BodyROSConnector(ros::NodeHandle* nh, const YAML::Node& e)
{
    nh_ = nh;

    std::string pose_topic, initpose_topic;
    mushr_mujoco_util::load_config(
        e, body_name_, pose_topic, initpose_topic, parent_body_name_);

    if (nh_)
    {
        pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(
            mushr_mujoco_util::pvt_name(body_name_, pose_topic), 10);
        initpose_sub_ = nh_->subscribe(
            mushr_mujoco_util::pvt_name(body_name_, initpose_topic),
            1,
            &BodyROSConnector::initpose_cb,
            this);
    }
}

void BodyROSConnector::ros_send_state()
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    if (nh_)
    {
        geometry_msgs::PoseStamped ros_pose;

        ros_pose.header.frame_id = parent_body_name_;
        mushr_mujoco_util::mj2ros_body(m, d, body_name_.c_str(), ros_pose.pose);

        pose_pub_.publish(ros_pose);

        mjglobal::mjdata_unlock();
    }
    else
    {
        throw std::runtime_error("No node handle to send state on");
    }
}

void BodyROSConnector::set_body_state(mushr_mujoco_ros::BodyState& bs)
{
    bs.name = body_name_;

    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mushr_mujoco_util::mj2ros_body(m, d, body_name_.c_str(), bs.pose);

    mjglobal::mjdata_unlock();
}

void BodyROSConnector::get_pose(
    mjModel* m, mjData* d, mushr_mujoco_ros::PoseTuple& pose)
{
    mushr_mujoco_util::mj2pose_body(m, d, body_name_.c_str(), pose);
}

void BodyROSConnector::set_pose(const geometry_msgs::Pose& pose)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mushr_mujoco_util::ros2mj_body(m, d, body_name_.c_str(), pose);

    mjglobal::mjdata_unlock();
}

void BodyROSConnector::set_pose(const PoseTuple& pose)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mushr_mujoco_util::pose2mj_body(m, d, body_name_.c_str(), pose);

    mjglobal::mjdata_unlock();
}

void BodyROSConnector::initpose_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    set_pose(msg->pose.pose);
}

} // namespace mushr_mujoco_ros
