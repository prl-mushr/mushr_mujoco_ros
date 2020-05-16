#include <thread>
#include <math.h>
#include <omp.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "mujoco.h"

#include "mjglobal.h"
#include "mushr_mujoco_util.h"
#include "sim_state.h"
#include "simple_viz.h"
#include "types.h"

#include "mushr_mujoco_ros/BodyStateArray.h"

#define quat2yaw(w, x, y, z) atan2(2.0 * (z * w + x * y), -1.0 + 2.0 * (w * w + x * x))

namespace py = pybind11;

int add(int i, int j)
{
    return i + j;
}

class PySimState
{
  public:
    PySimState(
        const std::string& mj_key_path,
        const std::string& model_file_path,
        const std::string& config_file,
        bool viz,
        bool record,
        const std::string& record_camera_name,
        const std::string& record_out_file)
      : state_(
            mj_key_path,
            model_file_path,
            config_file,
            viz,
            record,
            record_camera_name,
            record_out_file,
            NULL)
      , viz_(viz)
    {
        omp_set_dynamic(0);
        omp_set_num_threads(omp_get_num_procs());
    }
    ~PySimState()
    {
    }

    std::tuple<float, std::map<std::string, mushr_mujoco_ros::PoseTuple>> step(
        const std::map<std::string, mushr_mujoco_ros::CtrlTuple>& ctrls)
    {
        mushr_mujoco_ros::BodyStateArray bs;
        state_.step(ctrls, bs);
        std::map<std::string, mushr_mujoco_ros::PoseTuple> poses;
        bs2posetup(bs, poses);
        if (viz_)
            viz::display();
        return std::make_tuple(state_.simtime(), poses);
    }

    std::tuple<float, std::map<std::string, mushr_mujoco_ros::PoseTuple>> reset(
        const std::map<std::string, mushr_mujoco_ros::PoseTuple>& poses)
    {
        mushr_mujoco_ros::BodyStateArray bs;
        state_.reset(poses, bs);
        std::map<std::string, mushr_mujoco_ros::PoseTuple> out_poses;
        bs2posetup(bs, out_poses);
        return std::make_tuple(state_.simtime(), poses);
    }

    std::tuple<float, std::map<std::string, mushr_mujoco_ros::PoseTuple>> get_state()
    {
        mushr_mujoco_ros::BodyStateArray bs;
        state_.get_state(bs);
        std::map<std::string, mushr_mujoco_ros::PoseTuple> poses;
        bs2posetup(bs, poses);
        return std::make_tuple(state_.simtime(), poses);
    }

    void block_car_rollout(
        std::string car_name,
        std::string block_name,
        float dt,
        py::array_t<double> ctrls,
        py::array_t<double> out)
    {
        auto c = ctrls.unchecked<3>();
        auto o = out.mutable_unchecked<3>();

        if (c.shape(0) != o.shape(0))
        {
            throw std::runtime_error("Invalid shape at dim=0");
        }

        if (c.shape(1) != o.shape(1))
        {
            throw std::runtime_error("Invalid shape at dim=1");
        }

        int K = c.shape(0);
        int T = c.shape(1);

        auto car = state_.car_conn[car_name];
        auto block = state_.body_conn[block_name];

        mjModel* m = mjglobal::mjmodel();
        int niter = static_cast<int>(round(dt / m->opt.timestep));

#pragma omp parallel for schedule(static)
        for (int k = 0; k < K; k++)
        {
            mjData* dcpy = mjglobal::mjdata_copy();
            float x, y, z, qw, qx, qy, qz;

            for (int t = 0; t < T; t++)
            {
                for (int step = 0; step < niter; step++)
                {
                    mj_step1(m, dcpy);
                    car->apply_control(dcpy, c(k, t, 0), c(k, t, 1));
                    mj_step2(m, dcpy);
                }

                mushr_mujoco_ros::PoseTuple carp, blockp;
                car->get_pose(m, dcpy, carp);
                block->get_pose(m, dcpy, blockp);

                std::tie(x, y, z, qw, qx, qy, qz) = carp;
                o(k, t, 0) = x;
                o(k, t, 1) = y;
                o(k, t, 2) = quat2yaw(qw, qx, qy, qz);

                std::tie(x, y, z, qw, qx, qy, qz) = blockp;
                o(k, t, 3) = x;
                o(k, t, 4) = y;
                o(k, t, 5) = quat2yaw(qw, qx, qy, qz);
            }

            mj_deleteData(dcpy);
        }
    }

  private:
    mushr_mujoco_ros::SimState state_;
    bool viz_;

    void bs2posetup(
        const mushr_mujoco_ros::BodyStateArray& bs,
        std::map<std::string, mushr_mujoco_ros::PoseTuple>& poses)
    {
        for (auto s : bs.states)
        {
            auto p = s.pose;
            poses[s.name] = std::make_tuple(
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.w,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z);
        }
    }
};

PYBIND11_MODULE(mushr_mujoco_py, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");

    py::class_<PySimState>(m, "PySimState")
        .def(
            py::init<const std::string&,
                     const std::string&,
                     const std::string&,
                     bool,
                     bool,
                     const std::string&,
                     const std::string&>())
        .def("step", &PySimState::step, "Step the simulator forward")
        .def("reset", &PySimState::reset, "Reset the simulator")
        .def("get_state", &PySimState::get_state, "Gives body state")
        .def(
            "block_car_rollout",
            &PySimState::block_car_rollout,
            "Block and car rollout, specific");
}
