#include "mushr_ros_connector.h"
#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "yaml-cpp/yaml.h"

namespace mushr_mujoco_ros {

MuSHRROSConnector::MuSHRROSConnector(ros::NodeHandle* nh, const YAML::Node& e)
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
            &MuSHRROSConnector::initpose_cb,
            this);
    }

    mjModel* m = mjglobal::mjmodel();

    velocity_ = 0.0;
    steering_angle_ = 0.0;

    std::string control_topic = "control";
    if (e["control_topic"])
    {
        control_topic = e["control_topic"].as<std::string>();
    }
    if (nh_)
    {
        control_sub_ = nh_->subscribe(
            mushr_mujoco_util::pvt_name(body_name_, control_topic),
            1,
            &MuSHRROSConnector::control_cb,
            this);
    }

    std::string tv = car_ref("throttle_velocity");
    std::string sp = car_ref("steering_pos");

    velocity_ctrl_idx_ = mushr_mujoco_util::mj_name2id_ordie(m, mjOBJ_ACTUATOR, tv);
    steering_angle_ctrl_idx_
        = mushr_mujoco_util::mj_name2id_ordie(m, mjOBJ_ACTUATOR, sp);

    base_link_site_name_ = car_ref("base_link");

    init_sensors();
}

void MuSHRROSConnector::ros_send_state()
{
    if (nh_)
    {
        mjModel* m = mjglobal::mjmodel();
        mjData* d = mjglobal::mjdata_lock();

        geometry_msgs::PoseStamped ros_pose;

        ros_pose.header.frame_id = parent_body_name_;
        mushr_mujoco_util::mj2ros_site(
            m, d, base_link_site_name_.c_str(), body_name_.c_str(), ros_pose.pose);

        pose_pub_.publish(ros_pose);

        mjglobal::mjdata_unlock();
        send_sensor_state();
    }
    else
    {
        throw std::runtime_error("No node handle to send state on");
    }
}

void MuSHRROSConnector::set_body_state(mushr_mujoco_ros::BodyState& bs)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    bs.name = body_name_;
    mushr_mujoco_util::mj2ros_site(
        m, d, base_link_site_name_.c_str(), body_name_.c_str(), bs.pose);

    bs.ctrl_steering_angle = steering_angle_;
    bs.ctrl_velocity = velocity_;
    get_gyro(d, bs.imu);
    get_velocimeter(d, bs.velocity);

    mjglobal::mjdata_unlock();
}

void MuSHRROSConnector::get_pose(
    mjModel* m, mjData* d, mushr_mujoco_ros::PoseTuple& pose)
{
    mushr_mujoco_util::mj2pose_site(
        m, d, base_link_site_name_.c_str(), body_name_.c_str(), pose);
}

void MuSHRROSConnector::mujoco_controller()
{
    mjData* d = mjglobal::mjdata_lock();
    apply_control(d, velocity_, steering_angle_);
    mjglobal::mjdata_unlock();
}

void MuSHRROSConnector::apply_control(mjData* d, mjtNum vel, mjtNum steering_angle)
{
    d->ctrl[velocity_ctrl_idx_] = vel;
    d->ctrl[steering_angle_ctrl_idx_] = steering_angle;
}

void MuSHRROSConnector::control_cb(
    const ackermann_msgs::AckermannDriveStampedConstPtr& msg)
{
    steering_angle_ = msg->drive.steering_angle;
    velocity_ = msg->drive.speed;
}

void MuSHRROSConnector::set_pose(const geometry_msgs::Pose& pose)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mushr_mujoco_util::ros2mj_site(
        m, d, base_link_site_name_.c_str(), body_name_.c_str(), pose);

    mjglobal::mjdata_unlock();
}

void MuSHRROSConnector::set_pose(const PoseTuple& pose)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mushr_mujoco_util::pose2mj_site(
        m, d, base_link_site_name_.c_str(), body_name_.c_str(), pose);

    mjglobal::mjdata_unlock();
}

void MuSHRROSConnector::initpose_cb(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    set_pose(msg->pose.pose);
}

std::string MuSHRROSConnector::car_ref(const std::string name)
{
    return body_name_ + "_" + name;
}

} // namespace mushr_mujoco_ros
