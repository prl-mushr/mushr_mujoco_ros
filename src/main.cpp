#include "mujoco.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "body_ros_connector.h"
#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "mushr_ros_connector.h"
#include "simple_viz.h"

bool mj_sim_pause = false;

int main(int argc, char** argv)
{
    std::string model_file_path;

    bool do_viz;

    ros::init(argc, argv, "mushr_mujoco_ros");
    ros::NodeHandle nh("~");

    mushr_mujoco_util::init_mj(&nh);

    if (!nh.getParam("model_file_path", model_file_path))
    {
        ROS_FATAL("%s not set", nh.resolveName("model_file_path").c_str());
        exit(1);
    }
    if (!nh.getParam("viz", do_viz))
    {
        ROS_FATAL("%s not set", nh.resolveName("viz").c_str());
        exit(1);
    }

    ROS_INFO("Loading model");
    // make and model data
    char* error;
    if (mjglobal::init_model(model_file_path.c_str(), &error))
    {
        ROS_FATAL("Could not load binary model %s", error);
        exit(1);
    }
    ROS_INFO("Loading data");
    mjglobal::init_data();
    ROS_INFO("Loaded model and data");

    std::string config_file;
    if (!nh.getParam("config_file_path", config_file))
    {
        ROS_FATAL("%s not set", nh.resolveName("config_file_path").c_str());
        exit(1);
    }

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (YAML::BadFile e)
    {
        ROS_INFO("Couldn't open file %s", config_file.c_str());
        exit(1);
    }
    catch (std::exception e)
    {
        ROS_INFO("Unknown exception opening config file");
        exit(1);
    }

    std::vector<mushr_mujoco_ros::MuSHRROSConnector*> car_conn;
    std::vector<mushr_mujoco_ros::BodyROSConnector*> body_conn;

    ROS_INFO("Loading car configuration");
    // Load car info
    if (config["cars"])
    {
        YAML::Node car_cfg = config["cars"];
        for (int i = 0; i < car_cfg.size(); i++)
        {
            car_conn.push_back(
                new mushr_mujoco_ros::MuSHRROSConnector(&nh, car_cfg[i]));
        }
    }

    ROS_INFO("Loading bodies configuration");
    // Load body info
    if (config["bodies"])
    {
        YAML::Node bodies_cfg = config["bodies"];
        for (int i = 0; i < bodies_cfg.size(); i++)
        {
            body_conn.push_back(
                new mushr_mujoco_ros::BodyROSConnector(&nh, bodies_cfg[i]));
        }
    }

    if (do_viz)
    {
        ROS_INFO("Starting visualization");
        viz::init();
    }

    mjModel* m = mjglobal::mjmodel();
    mjData* d = NULL;

    double t = 0.0;
    const double dt = m->opt.timestep;

    float maxrate = 60.0;
    ros::Rate simrate(maxrate);
    while (ros::ok())
    {
        m = mjglobal::mjmodel();
        d = mjglobal::mjdata_lock();

        mjtNum simstart = d->time;
        if (!mushr_mujoco_util::is_paused())
        {
            while (d->time - simstart < 1.0 / maxrate)
            {
                mj_step1(m, d);
                for (auto cc : car_conn)
                    cc->mujoco_controller();
                mj_step2(m, d);
            }
        }

        mjglobal::mjdata_unlock();

        for (auto cc : car_conn)
            cc->send_state();
        for (auto bc : body_conn)
            bc->send_state();

        if (do_viz)
            viz::display();

        ros::spinOnce();
        simrate.sleep();
    }

    for (auto cc : car_conn)
        delete cc;
    for (auto bc : body_conn)
        delete bc;

    // free MuJoCo model and data, deactivate
    mjglobal::delete_model_and_data();
    mj_deactivate();

    viz::destroy();

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}
