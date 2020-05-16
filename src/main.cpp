#include "mujoco.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "body_ros_connector.h"
#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "mushr_ros_connector.h"
#include "rollout.h"
#include "simple_viz.h"

#include "mushr_mujoco_ros/BodyStateArray.h"
#include "mushr_mujoco_ros/Step.h"

extern bool mj_sim_pause_for_ctrl;

void step_sim(
    mjModel* m,
    mjData* d,
    float maxrate,
    std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*>& car_conn,
    std::map<std::string, mushr_mujoco_ros::BodyROSConnector*>& body_conn)
{
    mjtNum simstart = d->time;
    if (!mushr_mujoco_util::is_paused())
    {
        while (d->time - simstart < 1.0 / maxrate)
        {
            mj_step1(m, d);
            for (auto cc : car_conn)
                cc.second->mujoco_controller();
            mj_step2(m, d);
        }
        mj_sim_pause_for_ctrl = true;
    }
}

void send_individual_state(
    std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*>& car_conn,
    std::map<std::string, mushr_mujoco_ros::BodyROSConnector*>& body_conn)
{
    for (auto cc : car_conn)
    {
        cc.second->ros_send_state();
    }
    for (auto bc : body_conn)
    {
        bc.second->ros_send_state();
    }
}

void set_body_state(
    mjData* d,
    std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*>& car_conn,
    std::map<std::string, mushr_mujoco_ros::BodyROSConnector*>& body_conn,
    mushr_mujoco_ros::BodyStateArray& body_state)
{
    body_state.simtime = d->time;
    body_state.header.stamp = ros::Time::now();

    for (auto cc : car_conn)
    {
        mushr_mujoco_ros::BodyState bs;
        cc.second->set_body_state(bs);
        body_state.states.push_back(bs);
    }
    for (auto bc : body_conn)
    {
        mushr_mujoco_ros::BodyState bs;
        bc.second->set_body_state(bs);
        body_state.states.push_back(bs);
    }
}

bool reset_sim_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mushr_mujoco_util::reset(m, d);

    mjglobal::mjdata_unlock();
    return true;
}

int main(int argc, char** argv)
{
    std::string mj_key_path, model_file_path;

    bool do_viz, async_sim;

    ros::init(argc, argv, "mushr_mujoco_ros");
    ros::NodeHandle nh("~");

    if (!nh.getParam("mj_key", mj_key_path))
    {
        ROS_FATAL("%s not set", nh.resolveName("mj_key").c_str());
        exit(1);
    }

    mushr_mujoco_util::init_mj(mj_key_path);

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

    std::map<std::string, mushr_mujoco_ros::MuSHRROSConnector*> car_conn;
    std::map<std::string, mushr_mujoco_ros::BodyROSConnector*> body_conn;

    ROS_INFO("Loading car configuration");
    // Load car info
    if (config["cars"])
    {
        YAML::Node car_cfg = config["cars"];
        for (int i = 0; i < car_cfg.size(); i++)
        {
            auto mrc = new mushr_mujoco_ros::MuSHRROSConnector(&nh, car_cfg[i]);
            car_conn[mrc->body_name()] = mrc;
        }
    }

    ROS_INFO("Loading bodies configuration");
    // Load body info
    if (config["bodies"])
    {
        YAML::Node bodies_cfg = config["bodies"];
        for (int i = 0; i < bodies_cfg.size(); i++)
        {
            auto brc = new mushr_mujoco_ros::BodyROSConnector(&nh, bodies_cfg[i]);
            body_conn[brc->body_name()] = brc;
        }
    }

    ros::Publisher body_state_pub
        = nh.advertise<mushr_mujoco_ros::BodyStateArray>("body_state", 10);

    if (do_viz)
    {
        ROS_INFO("Starting visualization");
        viz::init();
    }

    rollout::init(&nh, &car_conn, &body_conn);

    ros::ServiceServer reset_srv = nh.advertiseService("reset", reset_sim_cb);

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

        step_sim(m, d, maxrate, car_conn, body_conn);

        send_individual_state(car_conn, body_conn);

        mushr_mujoco_ros::BodyStateArray body_state;
        set_body_state(d, car_conn, body_conn, body_state);
        body_state_pub.publish(body_state);

        mjglobal::mjdata_unlock();

        if (do_viz)
            viz::display();

        ros::spinOnce();
        simrate.sleep();
    }

    for (auto cc : car_conn)
        delete cc.second;
    for (auto bc : body_conn)
        delete bc.second;

    // free MuJoCo model and data, deactivate
    mjglobal::delete_model_and_data();
    mj_deactivate();

    viz::destroy();

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}
