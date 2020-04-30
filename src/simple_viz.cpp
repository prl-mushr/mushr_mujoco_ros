#include "simple_viz.h"

#include "mjglobal.h"
#include "mushr_mujoco_util.h"

extern bool mj_sim_pause_from_viz;

namespace viz {

GLFWwindow* window;
mjvCamera cam;  // abstract camera
mjvOption opt;  // visualization options
mjvScene scn;   // abstract scene
mjrContext con; // custom GPU context
mjuiState uistate;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

void init()
{
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjModel* m = mjglobal::mjmodel();
    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
}

void destroy()
{
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

void display()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();
    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    mjglobal::mjdata_unlock();

    if (mj_sim_pause_from_viz)
    {
        mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, viewport, "Paused", NULL, &con);
    }

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void init_local(mjModel* m)
{
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
}

void display_local(mjModel* m, mjData* d)
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    if (mj_sim_pause_from_viz)
    {
        mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, viewport, "Paused", NULL, &con);
    }

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mushr_mujoco_util::reset(m, d);
    }

    mjglobal::mjdata_unlock();

    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE)
    {
        mj_sim_pause_from_viz = !mj_sim_pause_from_viz;
    }
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift
        = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
           || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjModel* m = mjglobal::mjmodel();
    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle
        = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    mjModel* m = mjglobal::mjmodel();

    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

} // namespace viz
