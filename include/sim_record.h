#ifndef _MUSHR_MUJOCO_ROS__SIM_RECORDER_H_
#define _MUSHR_MUJOCO_ROS__SIM_RECORDER_H_

#include "mujoco.h"
#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"

namespace mushr_mujoco_ros {

class Recorder
{
  public:
    Recorder(const std::string& record_camera_name, const std::string& record_out_file);
    ~Recorder();

    void save_frame();

  private:
    ros::NodeHandle* nh_;

    mjvScene scn_;
    mjvCamera cam_;
    mjvOption opt_;
    mjrContext con_;
    mjrRect viewport_;

    unsigned char* rgb_;
    float* depth_;
    FILE* fp_;
};

} // namespace mushr_mujoco_ros

#endif // _MUSHR_MUJOCO_ROS__SIM_RECORDER_H_
