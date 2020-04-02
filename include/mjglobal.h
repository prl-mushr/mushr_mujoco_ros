#ifndef _MUSHR_MUJOCO_ROS__MJGLOBAL_H_
#define _MUSHR_MUJOCO_ROS__MJGLOBAL_H_

#include "mujoco.h"

namespace mjglobal {

int init_model(const char* model_file_path, char** error);
void init_data();
void delete_model_and_data();
mjModel* mjmodel();
mjData* mjdata_lock();
void mjdata_unlock();
mjData* mjdata_copy();

} // namespace mjglobal

#endif // _MUSHR_MUJOCO_ROS__MJGLOBAL_H_
