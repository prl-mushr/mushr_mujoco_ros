#include "string.h"

#include "mjglobal.h"
#include "sim_record.h"

namespace mushr_mujoco_ros {

Recorder::Recorder(
    const std::string& record_camera_name, const std::string& record_out_file)
{
    // initialize visualization data structures
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    mjModel* m = mjglobal::mjmodel();
    // create scene and context
    mjv_makeScene(m, &scn_, 2000);
    mjr_makeContext(m, &con_, mjFONTSCALE_150);

    int recordcamid;
    if ((recordcamid = mj_name2id(m, mjOBJ_CAMERA, record_camera_name.c_str())) < 0)
    {
        ROS_FATAL("Couldn't find record camera");
        exit(1);
    }

    cam_.type = mjCAMERA_FIXED;
    cam_.fixedcamid = recordcamid;

    // get size of active renderbuffer
    viewport_ = mjr_maxViewport(&con_);

    ROS_WARN("width %d height %d", viewport_.width, viewport_.height);
    rgb_ = new unsigned char[3 * viewport_.width * viewport_.height];
    depth_ = new float[viewport_.width * viewport_.height];

    // create output rgb file
    fp_ = fopen(record_out_file.c_str(), "wb");
    if (!fp_)
    {
        ROS_FATAL("Could not open rgbfile for writing");
        exit(1);
    }
}

Recorder::~Recorder()
{
    fclose(fp_);
    free(rgb_);
    free(depth_);
}

void Recorder::save_frame()
{
    mjModel* m = mjglobal::mjmodel();
    mjData* d = mjglobal::mjdata_lock();
    // update abstract scene
    mjv_updateScene(m, d, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjglobal::mjdata_unlock();

    // add time stamp in upper-left corner
    char stamp[50];
    sprintf(stamp, "Time = %.3f", d->time);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport_, stamp, NULL, &con_);

    // render scene in offscreen buffer
    mjr_render(viewport_, &scn_, &con_);

    // read rgb and depth buffers
    mjr_readPixels(rgb_, depth_, viewport_, &con_);

    // write rgb image to file
    fwrite(rgb_, 3, viewport_.width * viewport_.height, fp_);
}

} // namespace mushr_mujoco_ros
