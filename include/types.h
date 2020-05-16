#ifndef _MUSHR_MUJOCO_ROS__TYPES_H
#define _MUSHR_MUJOCO_ROS__TYPES_H

namespace mushr_mujoco_ros {

// velocity, steering_angle
typedef std::tuple<float, float> CtrlTuple;

// pose (x, y, z), quat (w, x, y ,z)
typedef std::tuple<float, float, float, float, float, float, float> PoseTuple;
}

#endif // _MUSHR_MUJOCO_ROS__TYPES_H
