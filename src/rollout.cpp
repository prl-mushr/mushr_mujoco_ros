#include <thread>
#include <omp.h>

#include "geometry_msgs/Pose.h"
#include "mushr_mujoco_ros/AckermannDriveArray.h"
#include "mushr_mujoco_ros/Rollout.h"
#include "ros/advertise_service_options.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"

#include "body_ros_connector.h"
#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "mushr_ros_connector.h"
#include "rollout.h"

namespace rollout {

ros::NodeHandle* nh_;
ros::ServiceServer s_;
ros::CallbackQueue cb_queue_;
ros::AsyncSpinner* spinner_;

std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*>* car_conn_;
std::map<std::string, mushr_mujoco_ros::BodyROSConnector*>* body_conn_;

static bool rollout(
    mushr_mujoco_ros::Rollout::Request& req, mushr_mujoco_ros::Rollout::Response& res);

static void one_rollout(
    mjModel* m,
    int k,
    int T,
    float dt,
    mushr_mujoco_ros::AckermannDriveArray& ctrls,
    geometry_msgs::PoseArray& car_pose,
    geometry_msgs::PoseArray& block_pose,
    mushr_mujoco_ros::MuSHRROSConnector* car,
    mushr_mujoco_ros::BodyROSConnector* block);

void init(
    ros::NodeHandle* nh,
    std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*>* car_conn,
    std::map<std::string, mushr_mujoco_ros::BodyROSConnector*>* body_conn)
{
    nh_ = nh;

    omp_set_dynamic(0);
    omp_set_num_threads(omp_get_num_procs());

    car_conn_ = car_conn;
    body_conn_ = body_conn;

    // uncomment to async handle rollout requests.
    bool async_rollouts = false;
    nh_->getParam("async_rollouts", async_rollouts);
    if (async_rollouts)
    {
        ros::AdvertiseServiceOptions opts
            = ros::AdvertiseServiceOptions::create<mushr_mujoco_ros::Rollout>(
                "rollout", rollout, NULL, &cb_queue_);
        s_ = nh_->advertiseService(opts);
        spinner_ = new ros::AsyncSpinner(0, &cb_queue_);
        spinner_->start();
    }
    else
    {
        s_ = nh_->advertiseService("rollout", rollout);
    }
}

void destroy()
{
    if (spinner_)
    {
        spinner_->stop();
        delete spinner_;
    }
}

bool rollout(
    mushr_mujoco_ros::Rollout::Request& req, mushr_mujoco_ros::Rollout::Response& res)
{
    int k;
    res.car_poses.resize(req.K);
    res.block_poses.resize(req.K);

    double start = omp_get_wtime();
    mjModel* m = mjglobal::mjmodel();

    auto car_it = car_conn_->find(req.car_name);
    if (car_it == car_conn_->end())
    {
        ROS_WARN("Requested rollout for car '%s' doesn't exist", req.car_name.c_str());
        return false;
    }
    mushr_mujoco_ros::MuSHRROSConnector* car = car_it->second;

    auto block_it = body_conn_->find(req.block_name);
    if (block_it == body_conn_->end())
    {
        ROS_WARN(
            "Requested rollout for block '%s' doesn't exist", req.block_name.c_str());
        return false;
    }
    mushr_mujoco_ros::BodyROSConnector* block = block_it->second;

#pragma omp parallel for schedule(static)
    for (k = 0; k < req.K; k++)
    {
        one_rollout(
            m,
            k,
            req.T,
            req.dt,
            req.ctrls[k],
            res.car_poses[k],
            res.block_poses[k],
            car,
            block);
    }

    return true;
}

void one_rollout(
    mjModel* m,
    int k,
    int T,
    float dt,
    mushr_mujoco_ros::AckermannDriveArray& ctrls,
    geometry_msgs::PoseArray& car_pose,
    geometry_msgs::PoseArray& block_pose,
    mushr_mujoco_ros::MuSHRROSConnector* car,
    mushr_mujoco_ros::BodyROSConnector* block)
{
    int t;
    int niter = static_cast<int>(dt / m->opt.timestep);
    int step;

    // Reserve T poses.
    car_pose.poses.resize(T);
    block_pose.poses.resize(T);

    mjData* dcpy = mjglobal::mjdata_copy();

    if (niter == 0)
    {
        ROS_WARN("dt/timestep %f/%f = 0", dt, m->opt.timestep);
    }

    for (t = 0; t < T; t++)
    {
        mjtNum simstart = dcpy->time;
        for (step = 0; step < niter; step++)
        {
            mj_step1(m, dcpy);
            car->apply_control(dcpy, ctrls.data[t].speed, ctrls.data[t].steering_angle);
            mj_step2(m, dcpy);
        }

        mushr_mujoco_util::mj2ros_body(
            m, dcpy, car->body_name().c_str(), car_pose.poses[t]);
        mushr_mujoco_util::mj2ros_body(
            m, dcpy, block->body_name().c_str(), block_pose.poses[t]);
    }

    mj_deleteData(dcpy);
}

} // rollout
