#include <wordexp.h>

#include <tf2/LinearMath/Quaternion.h>
#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include "mushr_mujoco_util.h"

extern bool mj_sim_pause;

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
    int bodyid, jntid, jntqposid;

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
}

bool is_paused()
{
    return mj_sim_pause;
}

} // mushr_mujoco_util
