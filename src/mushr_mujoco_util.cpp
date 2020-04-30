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

void init_mj(const ros::NodeHandle* nh)
{
    std::string mj_key_path;
    wordexp_t p;

    if (nh->getParam("mj_key", mj_key_path))
    {
        ROS_INFO("Activating MuJoCo...");
        wordexp(mj_key_path.c_str(), &p, 0);
        if (p.we_wordc != 1)
        {
            ROS_FATAL("Failed to find single key for param '%s'", mj_key_path.c_str());
            exit(1);
        }
        mj_activate(p.we_wordv[0]);
        wordfree(&p);
        ROS_INFO("Activated");
    }
    else
    {
        ROS_FATAL("%s not set", nh->resolveName("mj_key").c_str());
        exit(1);
    }
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
    int siteid;
    mjtNum pos[3], quat[4];

    if ((siteid = mj_name2id(m, mjOBJ_SITE, site_name)) < 0)
    {
        ROS_INFO("Couldn't find site '%s' ID", site_name);
        return;
    }

    mj2ros_body(m, d, body_name, ros_pose);

    mjtNum bodyq[]{
        ros_pose.orientation.w,
        ros_pose.orientation.x,
        ros_pose.orientation.y,
        ros_pose.orientation.z,
    };
    mju_rotVecQuat(pos, &m->site_pos[3 * siteid], bodyq);
    ros_pose.position.x += pos[0];
    ros_pose.position.y += pos[1];
    ros_pose.position.z += pos[2];

    mju_mulQuat(quat, &m->site_quat[4 * siteid], bodyq);
    ros_pose.orientation.w = quat[0];
    ros_pose.orientation.x = quat[1];
    ros_pose.orientation.y = quat[2];
    ros_pose.orientation.z = quat[3];
}

void ros2mj_site(
    const mjModel* m,
    mjData* d,
    const char* site_name,
    const char* body_name,
    geometry_msgs::Pose ros_pose)
{
    int siteid;
    mjtNum pos[3], quat[4];

    if ((siteid = mj_name2id(m, mjOBJ_SITE, site_name)) < 0)
    {
        ROS_INFO("Couldn't find site '%s' ID", site_name);
        return;
    }

    mjtNum bodyq[]{
        ros_pose.orientation.w,
        ros_pose.orientation.x,
        ros_pose.orientation.y,
        ros_pose.orientation.z,
    };
    mju_rotVecQuat(pos, &m->site_pos[3 * siteid], bodyq);
    ros_pose.position.x -= pos[0];
    ros_pose.position.y -= pos[1];

    // RVIZ publishes in R^2, so we'll have a hard time with Z offsets.
    // ros_pose.position.z -= pos[2];

    mju_mulQuat(quat, &m->site_quat[4 * siteid], bodyq);
    ros_pose.orientation.w = quat[0];
    ros_pose.orientation.x = quat[1];
    ros_pose.orientation.y = quat[2];
    ros_pose.orientation.z = quat[3];

    ros2mj_body(m, d, body_name, ros_pose);
}

void mj2ros_body(
    const mjModel* m, mjData* d, const char* name, geometry_msgs::Pose& ros_pose)
{
    int bodyid, jntid, jntqposid;

    if ((bodyid = mj_name2id(m, mjOBJ_BODY, name)) < 0)
    {
        ROS_INFO("Couldn't find '%s' ID", name);
        return;
    }

    jntid = m->body_jntadr[bodyid];
    jntqposid = m->jnt_qposadr[jntid];

    ros_pose.position.x = d->qpos[jntqposid + 0];
    ros_pose.position.y = d->qpos[jntqposid + 1];
    ros_pose.position.z = d->qpos[jntqposid + 2];
    ros_pose.orientation.w = d->qpos[jntqposid + 3];
    ros_pose.orientation.x = d->qpos[jntqposid + 4];
    ros_pose.orientation.y = d->qpos[jntqposid + 5];
    ros_pose.orientation.z = d->qpos[jntqposid + 6];
}

void ros2mj_body(
    const mjModel* m, mjData* d, const char* name, const geometry_msgs::Pose& ros_pose)
{
    int bodyid, jntid, jntqposid, jntdofid;

    if ((bodyid = mj_name2id(m, mjOBJ_BODY, name)) < 0)
    {
        ROS_INFO("Couldn't find '%s' ID", name);
        return;
    }

    jntid = m->body_jntadr[bodyid];
    jntqposid = m->jnt_qposadr[jntid];

    d->qpos[jntqposid + 0] = ros_pose.position.x;
    d->qpos[jntqposid + 1] = ros_pose.position.y;
    d->qpos[jntqposid + 2] = ros_pose.position.z;
    d->qpos[jntqposid + 3] = ros_pose.orientation.w;
    d->qpos[jntqposid + 4] = ros_pose.orientation.x;
    d->qpos[jntqposid + 5] = ros_pose.orientation.y;
    d->qpos[jntqposid + 6] = ros_pose.orientation.z;

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
