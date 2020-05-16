#include <wordexp.h>

#include <tf2/LinearMath/Quaternion.h>
#include "mjglobal.h"
#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include "mushr_mujoco_util.h"

bool mj_sim_pause_from_viz = false;
bool mj_sim_pause_for_ctrl = false;

namespace mushr_mujoco_util {

void init_mj(const std::string& mj_key_path)
{
    wordexp_t p;

    // expand home
    wordexp(mj_key_path.c_str(), &p, 0);
    if (p.we_wordc != 1)
    {
        throw std::runtime_error("Failed to find single key for param " + mj_key_path);
    }

    mj_activate(p.we_wordv[0]);
    wordfree(&p);
}

void load_config(
    const YAML::Node& e,
    std::string& body_name,
    std::string& pose_topic,
    std::string& initpose_topic,
    std::string& parent_body_name)
{
    if (!e["name"])
    {
        ROS_FATAL("No 'name' for some element");
        exit(1);
    }
    body_name = e["name"].as<std::string>();

    pose_topic = "pose";
    if (e["pose_topic"])
    {
        pose_topic = e["pose_topic"].as<std::string>();
    }

    initpose_topic = "initialpose";
    if (e["initialpose_topic"])
    {
        initpose_topic = e["initialpose_topic"].as<std::string>();
    }

    mjModel* m = mjglobal::mjmodel();
    int body_id = mushr_mujoco_util::mj_name2id_ordie(m, mjOBJ_BODY, body_name);

    int parent_body_id = m->body_parentid[body_id];
    parent_body_name = "map";
    if (parent_body_id != 0)
    {
        std::string parent_body_name(mj_id2name(m, mjOBJ_BODY, parent_body_id));
    }
}

mjtNum mj_name2id_ordie(const mjModel* m, int type, const std::string& name)
{
    return mj_name2id_ordie(m, type, name.c_str());
}

mjtNum mj_name2id_ordie(const mjModel* m, int type, const char* name)
{
    mjtNum idx;
    if ((idx = mj_name2id(m, type, name)) < 0)
    {
        ROS_INFO("'%s' of type mjOBJ_() %d not found", name, type);
        exit(1);
    }
    return idx;
}

void mj2ros_site(
    const mjModel* m,
    mjData* d,
    const char* site_name,
    const char* body_name,
    geometry_msgs::Pose& ros_pose)
{
    mushr_mujoco_ros::PoseTuple pose;
    mj2pose_site(m, d, site_name, body_name, pose);
    ros_pose.position.x = std::get<0>(pose);
    ros_pose.position.y = std::get<1>(pose);
    ros_pose.position.z = std::get<2>(pose);
    ros_pose.orientation.w = std::get<3>(pose);
    ros_pose.orientation.x = std::get<4>(pose);
    ros_pose.orientation.y = std::get<5>(pose);
    ros_pose.orientation.z = std::get<6>(pose);
}

void mj2pose_site(
    const mjModel* m,
    mjData* d,
    const char* site_name,
    const char* body_name,
    mushr_mujoco_ros::PoseTuple& pose)
{
    int siteid;
    mjtNum pos[3], quat[4];

    if ((siteid = mj_name2id(m, mjOBJ_SITE, site_name)) < 0)
    {
        ROS_INFO("Couldn't find site '%s' ID", site_name);
        return;
    }

    mj2pose_body(m, d, body_name, pose);

    mjtNum bodyq[]{
        std::get<3>(pose), std::get<4>(pose), std::get<5>(pose), std::get<6>(pose),
    };
    mju_rotVecQuat(pos, &m->site_pos[3 * siteid], bodyq);
    std::get<0>(pose) += pos[0];
    std::get<1>(pose) += pos[1];
    std::get<2>(pose) += pos[2];

    mju_mulQuat(quat, &m->site_quat[4 * siteid], bodyq);
    std::get<3>(pose) = quat[0];
    std::get<4>(pose) = quat[1];
    std::get<5>(pose) = quat[2];
    std::get<6>(pose) = quat[3];
}

void mj2ros_body(
    const mjModel* m, mjData* d, const char* name, geometry_msgs::Pose& ros_pose)
{
    mushr_mujoco_ros::PoseTuple pose;
    mj2pose_body(m, d, name, pose);
    ros_pose.position.x = std::get<0>(pose);
    ros_pose.position.y = std::get<1>(pose);
    ros_pose.position.z = std::get<2>(pose);
    ros_pose.orientation.w = std::get<3>(pose);
    ros_pose.orientation.x = std::get<4>(pose);
    ros_pose.orientation.y = std::get<5>(pose);
    ros_pose.orientation.z = std::get<6>(pose);
}

void mj2pose_body(
    const mjModel* m, mjData* d, const char* name, mushr_mujoco_ros::PoseTuple& pose)
{
    int bodyid, jntid, jntqposid;

    if ((bodyid = mj_name2id(m, mjOBJ_BODY, name)) < 0)
    {
        ROS_INFO("Couldn't find '%s' ID", name);
        return;
    }

    jntid = m->body_jntadr[bodyid];
    jntqposid = m->jnt_qposadr[jntid];

    std::get<0>(pose) = d->qpos[jntqposid + 0];
    std::get<1>(pose) = d->qpos[jntqposid + 1];
    std::get<2>(pose) = d->qpos[jntqposid + 2];
    std::get<3>(pose) = d->qpos[jntqposid + 3];
    std::get<4>(pose) = d->qpos[jntqposid + 4];
    std::get<5>(pose) = d->qpos[jntqposid + 5];
    std::get<6>(pose) = d->qpos[jntqposid + 6];
}

void ros2mj_site(
    const mjModel* m,
    mjData* d,
    const char* site_name,
    const char* body_name,
    geometry_msgs::Pose ros_pose)
{
    pose2mj_site(
        m,
        d,
        site_name,
        body_name,
        std::make_tuple(
            ros_pose.position.x,
            ros_pose.position.y,
            ros_pose.position.z,
            ros_pose.orientation.w,
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z));
}

void pose2mj_site(
    const mjModel* m,
    mjData* d,
    const char* site_name,
    const char* body_name,
    mushr_mujoco_ros::PoseTuple pose)
{
    int siteid;
    mjtNum pos[3], quat[4];

    if ((siteid = mj_name2id(m, mjOBJ_SITE, site_name)) < 0)
    {
        ROS_INFO("Couldn't find site '%s' ID", site_name);
        return;
    }

    mjtNum bodyq[]{
        std::get<3>(pose), std::get<4>(pose), std::get<5>(pose), std::get<6>(pose)};

    mju_rotVecQuat(pos, &m->site_pos[3 * siteid], bodyq);
    std::get<0>(pose) -= pos[0];
    std::get<1>(pose) -= pos[1];

    // RVIZ publishes in R^2, so we'll have a hard time with Z offsets.
    // std::get<1>(pose) -= pos[2];

    mju_mulQuat(quat, &m->site_quat[4 * siteid], bodyq);
    std::get<3>(pose) = quat[0];
    std::get<4>(pose) = quat[1];
    std::get<5>(pose) = quat[2];
    std::get<6>(pose) = quat[3];

    pose2mj_body(m, d, body_name, pose);
}

void ros2mj_body(
    const mjModel* m, mjData* d, const char* name, const geometry_msgs::Pose& ros_pose)
{
    pose2mj_body(
        m,
        d,
        name,
        std::make_tuple(
            ros_pose.position.x,
            ros_pose.position.y,
            ros_pose.position.z,
            ros_pose.orientation.w,
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z));
}

void pose2mj_body(
    const mjModel* m,
    mjData* d,
    const char* name,
    const mushr_mujoco_ros::PoseTuple& pose)
{
    int bodyid, jntid, jntqposid, jntdofid;

    if ((bodyid = mj_name2id(m, mjOBJ_BODY, name)) < 0)
    {
        ROS_INFO("Couldn't find '%s' ID", name);
        return;
    }

    jntid = m->body_jntadr[bodyid];
    jntqposid = m->jnt_qposadr[jntid];

    d->qpos[jntqposid + 0] = std::get<0>(pose);
    d->qpos[jntqposid + 1] = std::get<1>(pose);
    d->qpos[jntqposid + 2] = std::get<2>(pose);
    d->qpos[jntqposid + 3] = std::get<3>(pose);
    d->qpos[jntqposid + 4] = std::get<4>(pose);
    d->qpos[jntqposid + 5] = std::get<5>(pose);
    d->qpos[jntqposid + 6] = std::get<6>(pose);

    jntdofid = m->jnt_dofadr[jntid];
    mju_zero(&d->qvel[jntdofid], 6);
}

void reset(mjModel* m, mjData* d)
{
    mj_resetData(m, d);
    mj_forward(m, d);
}

bool is_paused()
{
    return mj_sim_pause_from_viz || mj_sim_pause_for_ctrl;
}

std::string pvt_name(const std::string& body_name, const std::string& topic_name)
{
    return body_name + "/" + topic_name;
}

} // mushr_mujoco_util
