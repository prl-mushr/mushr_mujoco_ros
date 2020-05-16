#ifndef _MUSHR_MUJOCO_ROS__SIM_STATE_H
#define _MUSHR_MUJOCO_ROS__SIM_STATE_H

#include <map>
#include <string>

#include "body_ros_connector.h"
#include "mujoco.h"
#include "mushr_ros_connector.h"
#include "sim_record.h"
#include "types.h"

#include <mushr_mujoco_ros/BodyStateArray.h>

namespace mushr_mujoco_ros {

class SimState
{
  public:
    SimState(
        const std::string& mj_key_path,
        const std::string& model_file_path,
        const std::string& config_file,
        bool viz,
        bool record,
        const std::string& record_camera_name,
        const std::string& record_out_file,
        ros::NodeHandle* nh);
    ~SimState();

    void step(
        const std::map<std::string, mushr_mujoco_ros::CtrlTuple>& ctrls,
        mushr_mujoco_ros::BodyStateArray& body_state);
    void reset(
        const std::map<std::string, mushr_mujoco_ros::PoseTuple>& poses,
        mushr_mujoco_ros::BodyStateArray& body_state);
    void get_state(mushr_mujoco_ros::BodyStateArray& body_state);

    void set_body_state(mjData* d, mushr_mujoco_ros::BodyStateArray& body_state);

    float simtime();
    void update_viz();

    std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*> car_conn;
    std::map<std::string, mushr_mujoco_ros::BodyROSConnector*> body_conn;

    mushr_mujoco_ros::Recorder* recorder = NULL;

    bool viz_;
};
}

#endif // _MUSHR_MUJOCO_ROS__SIM_STATE_H
