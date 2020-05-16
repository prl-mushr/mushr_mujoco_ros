#include "sim_state.h"
#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "simple_viz.h"

namespace mushr_mujoco_ros {

SimState::SimState(
    const std::string& mj_key_path,
    const std::string& model_file_path,
    const std::string& config_file,
    bool viz,
    bool record,
    const std::string& record_camera_name,
    const std::string& record_out_file,
    ros::NodeHandle* nh)
{
    char* error;

    mushr_mujoco_util::init_mj(mj_key_path);

    if (mjglobal::init_model(model_file_path.c_str(), &error))
    {
        throw std::runtime_error("Could not load binary model " + std::string(error));
    }
    mjglobal::init_data();

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (YAML::BadFile e)
    {
        throw std::invalid_argument("Couldn't open file: " + config_file);
    }
    catch (std::exception e)
    {
        throw e;
    }

    // Load car info
    if (config["cars"])
    {
        YAML::Node car_cfg = config["cars"];
        for (int i = 0; i < car_cfg.size(); i++)
        {
            auto mrc = new MuSHRROSConnector(nh, car_cfg[i]);
            car_conn[mrc->body_name()] = mrc;
        }
    }

    // Load body info
    if (config["bodies"])
    {
        YAML::Node bodies_cfg = config["bodies"];
        for (int i = 0; i < bodies_cfg.size(); i++)
        {
            auto brc = new BodyROSConnector(nh, bodies_cfg[i]);
            body_conn[brc->body_name()] = brc;
        }
    }

    viz_ = viz;
    if (viz_)
    {
        viz::init();
    }

    if (record)
    {
        recorder = new Recorder(record_camera_name, record_out_file);
    }
}

SimState::~SimState()
{
    delete recorder;

    if (viz_)
    {
        viz::destroy();

        // free MuJoCo model and data, deactivate
        mjglobal::delete_model_and_data();
        mj_deactivate();

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
#endif
    }
}

void SimState::step(
    const std::map<std::string, CtrlTuple>& ctrls, BodyStateArray& body_state)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();
    float dt = 1.0 / 60.0;
    float speed, steering_angle;

    int niter = static_cast<int>(round(dt / m->opt.timestep));

    /* std::cout << "SIM STEP niter " << niter << std::endl; */

    if (!mushr_mujoco_util::is_paused())
    {
        for (int i = 0; i < niter; i++)
        {
            mj_step1(m, d);
            for (auto it = ctrls.begin(); it != ctrls.end(); it++)
            {
                auto name = it->first;
                std::tie(speed, steering_angle) = it->second;
                car_conn[name]->apply_control(d, speed, steering_angle);
            }
            mj_step2(m, d);
        }
    }

    set_body_state(d, body_state);
    if (recorder)
    {
        recorder->save_frame();
    }

    mjglobal::mjdata_unlock();
}

void SimState::reset(
    const std::map<std::string, PoseTuple>& poses, BodyStateArray& body_state)
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();

    mushr_mujoco_util::reset(m, d);

    for (auto it = poses.begin(); it != poses.end(); it++)
    {
        auto car = car_conn.find(it->first);
        if (car != car_conn.end())
        {
            car->second->set_pose(it->second);
        }

        auto body = body_conn.find(it->first);
        if (body != body_conn.end())
        {
            body->second->set_pose(it->second);
        }
    }

    set_body_state(d, body_state);

    mjglobal::mjdata_unlock();
}

void SimState::get_state(BodyStateArray& body_state)
{
    mjData* d = mjglobal::mjdata_lock();
    set_body_state(d, body_state);
    mjglobal::mjdata_unlock();
}

void SimState::set_body_state(mjData* d, BodyStateArray& body_state)
{
    body_state.simtime = d->time;

    for (auto cc : car_conn)
    {
        BodyState bs;
        cc.second->set_body_state(bs);
        body_state.states.push_back(bs);
    }
    for (auto bc : body_conn)
    {
        BodyState bs;
        bc.second->set_body_state(bs);
        body_state.states.push_back(bs);
    }
}

float SimState::simtime()
{
    float simtime;
    mjData* d = mjglobal::mjdata_lock();
    simtime = d->time;
    mjglobal::mjdata_unlock();
    return simtime;
}

void SimState::update_viz()
{
    if (viz_)
    {
        viz::display();
    }
}

} // namespace mushr_mujoco_ros
