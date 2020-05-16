#include "mujoco.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "body_ros_connector.h"
#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "mushr_ros_connector.h"
#include "rollout.h"
#include "sim_record.h"
#include "sim_state.h"

#include "mushr_mujoco_ros/BodyStateArray.h"
#include "mushr_mujoco_ros/GetState.h"
#include "mushr_mujoco_ros/Reset.h"
#include "mushr_mujoco_ros/Step.h"

class SrvResponder
{
  public:
    SrvResponder(mushr_mujoco_ros::SimState& s) : state{s}
    {
    }

    bool step(
        mushr_mujoco_ros::Step::Request& req, mushr_mujoco_ros::Step::Response& res)
    {
        std::map<std::string, mushr_mujoco_ros::CtrlTuple> ctrls;
        ctrls["buddy"]
            = std::make_tuple(req.ctrl.drive.speed, req.ctrl.drive.steering_angle);

        res.body_state.header.stamp = ros::Time::now();
        state.step(ctrls, res.body_state);

        return true;
    }

    bool get_state(
        mushr_mujoco_ros::GetState::Request& req,
        mushr_mujoco_ros::GetState::Response& res)
    {
        res.body_state.header.stamp = ros::Time::now();
        state.get_state(res.body_state);

        return true;
    }

    bool reset(
        mushr_mujoco_ros::Reset::Request& req, mushr_mujoco_ros::Reset::Response& res)
    {
        mjModel* m = mjglobal::mjmodel();
        mjData* d = mjglobal::mjdata_lock();

        ROS_INFO("Reset initiated");
        if (req.body_names.size() != req.init_state.size())
        {
            return false;
        }
        std::map<std::string, mushr_mujoco_ros::PoseTuple> poses;
        for (int i = 0; i < req.body_names.size(); i++)
        {
            auto p = req.init_state[i];
            poses[req.body_names[i]] = std::make_tuple(
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.w,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z);
        }

        res.body_state.header.stamp = ros::Time::now();
        state.reset(poses, res.body_state);

        mjglobal::mjdata_unlock();
        return true;
    }

  private:
    mushr_mujoco_ros::SimState& state;
};

#define PARAM_GET_OR_DIE(nh, key, value)                                               \
    do                                                                                 \
    {                                                                                  \
        if (!nh.getParam(key, value))                                                  \
        {                                                                              \
            ROS_FATAL("%s not set", nh.resolveName(key).c_str());                      \
            exit(1);                                                                   \
        }                                                                              \
        std::cout << key << ", " << value << std::endl;                                \
    } while (0)

int main(int argc, char** argv)
{
    std::string mj_key_path, model_file_path, config_file, record_camera_name,
        record_out_file;
    bool do_viz, do_record;

    ros::init(argc, argv, "mushr_mujoco_ros");
    ros::NodeHandle nh("~");

    PARAM_GET_OR_DIE(nh, "mj_key", mj_key_path);
    PARAM_GET_OR_DIE(nh, "model_file_path", model_file_path);
    PARAM_GET_OR_DIE(nh, "viz", do_viz);
    PARAM_GET_OR_DIE(nh, "record", do_record);
    PARAM_GET_OR_DIE(nh, "config_file_path", config_file);
    PARAM_GET_OR_DIE(nh, "record_camera_name", record_camera_name);
    PARAM_GET_OR_DIE(nh, "record_out_file", record_out_file);

    mushr_mujoco_ros::SimState state(
        mj_key_path,
        model_file_path,
        config_file,
        do_viz,
        do_record,
        record_camera_name,
        record_out_file,
        &nh);
    SrvResponder srv_resp(state);
    ros::ServiceServer reset_srv
        = nh.advertiseService("reset", &SrvResponder::reset, &srv_resp);
    ros::ServiceServer step_srv
        = nh.advertiseService("step", &SrvResponder::step, &srv_resp);
    ros::ServiceServer get_state_srv
        = nh.advertiseService("state", &SrvResponder::get_state, &srv_resp);

    rollout::init(&nh, &state.car_conn, &state.body_conn);

    while (ros::ok())
    {
        state.update_viz();

        ros::spinOnce();
    }
}
