#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "mushr_ros_connector.h"

namespace mushr_mujoco_ros {

void MuSHRROSConnector::init_sensors()
{
    imu_pub_ = nh_->advertise<sensor_msgs::Imu>(
        mushr_mujoco_util::pvt_name(body_name_, "car_imu"), 10);
    velocity_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>(
        mushr_mujoco_util::pvt_name(body_name_, "velocimeter"), 10);

    mjModel* m = mjglobal::mjmodel();

    std::string iacc = car_ref("accelerometer");
    std::string igyro = car_ref("gyro");
    std::string ivel = car_ref("velocimeter");

    accel_idx_ = mushr_mujoco_util::mj_name2id_ordie(m, mjOBJ_SENSOR, iacc);
    gyro_idx_ = mushr_mujoco_util::mj_name2id_ordie(m, mjOBJ_SENSOR, igyro);
    vel_idx_ = mushr_mujoco_util::mj_name2id_ordie(m, mjOBJ_SENSOR, ivel);

    if (accel_idx_ >= 0)
    {
        accel_noise_ = m->sensor_noise[accel_idx_];
        accel_addr_ = m->sensor_adr[accel_idx_];
    }
    else
    {
        accel_noise_ = -1.0;
    }

    if (gyro_idx_ >= 0)
    {
        gyro_noise_ = m->sensor_noise[gyro_idx_];
        gyro_addr_ = m->sensor_adr[gyro_idx_];
    }
    else
    {
        gyro_noise_ = -1.0;
    }

    if (vel_idx_ >= 0)
    {
        vel_noise_ = m->sensor_noise[vel_idx_];
        vel_addr_ = m->sensor_adr[vel_idx_];
    }

    ROS_INFO("Accelerometer addr = %d, gyroscope addr = %d", accel_addr_, gyro_addr_);
    ROS_INFO(
        "Accelerometer noise = %f, gyroscope noise = %f", accel_noise_, gyro_noise_);
}

void MuSHRROSConnector::send_sensor_state()
{
    if (accel_idx_ < 0 && gyro_idx_ < 0 && vel_idx_ < 0)
        return;

    sensor_msgs::Imu msg;
    geometry_msgs::Vector3Stamped vel_msg;

    msg.orientation_covariance[0] = -1;

    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    get_gyro(d, msg);
    get_accel(d, msg);

    if (get_velocimeter(d, vel_msg.vector) > -1)
    {
        vel_msg.header.stamp = ros::Time::now();
        velocity_pub_.publish(vel_msg);
    }

    mjglobal::mjdata_unlock();

    msg.header.stamp = ros::Time::now();
    imu_pub_.publish(msg);
}

/* Assumes data lock is held */
int MuSHRROSConnector::get_velocimeter(const mjData* d, geometry_msgs::Vector3& vec)
{
    if (vel_idx_ < 0)
        return -1;

    mjtNum* vel = &d->sensordata[vel_addr_];
    vec.x = vel[0];
    vec.y = vel[1];
    vec.z = vel[2];

    return 0;
}

void MuSHRROSConnector::get_gyro(const mjData* d, sensor_msgs::Imu& imu)
{
    if (gyro_idx_ < 0)
    {
        imu.angular_velocity_covariance[0] = -1;
    }
    else
    {
        mjtNum* gyro = &d->sensordata[gyro_addr_];

        imu.angular_velocity.x = gyro[0];
        imu.angular_velocity.y = gyro[1];
        imu.angular_velocity.z = gyro[2];

        imu.angular_velocity_covariance[0] = gyro_noise_; // var(x)
        imu.angular_velocity_covariance[3] = gyro_noise_; // var(y)
        imu.angular_velocity_covariance[6] = gyro_noise_; // var(z)
    }
}

void MuSHRROSConnector::get_accel(const mjData* d, sensor_msgs::Imu& imu)
{
    if (accel_idx_ < 0)
    {
        imu.linear_acceleration_covariance[0] = -1;
    }
    else
    {
        mjtNum* accel = &d->sensordata[accel_addr_];

        imu.linear_acceleration.x = accel[0];
        imu.linear_acceleration.y = accel[1];
        imu.linear_acceleration.z = accel[2];

        imu.linear_acceleration_covariance[0] = accel_noise_; // var(x)
        imu.linear_acceleration_covariance[3] = accel_noise_; // var(y)
        imu.linear_acceleration_covariance[6] = accel_noise_; // var(z)
    }
}

} // namespace mushr_mujoco_ros
