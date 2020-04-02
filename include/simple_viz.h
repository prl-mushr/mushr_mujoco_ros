#ifndef _MUSHR_MUJOCO_ROS__SIMPLE_VIZ_H_
#define _MUSHR_MUJOCO_ROS__SIMPLE_VIZ_H_

#include "glfw3.h"
#include "mujoco.h"
#include "ros/ros.h"

namespace viz {

void init();
void destroy();
void display();

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void scroll(GLFWwindow* window, double xoffset, double yoffset);

void init_local(mjModel* m);
void display_local(mjModel* m, mjData* d);

} // namespace viz

#endif // _MUSHR_MUJOCO_ROS__SIMPLE_VIZ_H_
