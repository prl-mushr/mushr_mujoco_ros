#include <mutex>
#include <string.h>

#include "mjglobal.h"
#include "ros/ros.h"

namespace mjglobal {

mjModel* m_global;
mjData* d_global;

static std::recursive_mutex data_lock;
static std::recursive_mutex model_lock;

int init_model(const char* model_file_path, char** error)
{
    std::string mj_key_path;
    mjModel* m = NULL;

    // load and compile model
    *error = new char[1000];
    strcpy(*error, "Could not load binary model");
    size_t model_file_len = strnlen(model_file_path, 1000);
    if (model_file_len > 4 && !strncmp(model_file_path + model_file_len - 4, ".mjb", 4))
        m = mj_loadModel(model_file_path, 0);
    else 
        m = mj_loadXML(model_file_path, 0, *error, 1000);
    if (!m)
        return 1;

    model_lock.lock();
    m_global = m;
    model_lock.unlock();
    return 0;
}

void init_data()
{
    data_lock.lock();
    d_global = mj_makeData(m_global);
    data_lock.unlock();
}

void delete_model_and_data()
{
    model_lock.lock();
    data_lock.lock();
    mj_deleteData(d_global);
    mj_deleteModel(m_global);
    d_global = NULL;
    m_global = NULL;
    data_lock.unlock();
    model_lock.unlock();
}

// Assume the model never gets changed, which should be a fair assumption
mjModel* mjmodel()
{
    model_lock.lock();
    mjModel* m = m_global;
    model_lock.unlock();
    return m;
}

// Grabs lock and returns data
inline mjData* mjdata_lock()
{
    data_lock.lock();
    return d_global;
}

// Unocks, invalidating any updates
inline void mjdata_unlock()
{
    data_lock.unlock();
}

mjData* mjdata_copy()
{
    mjModel* m = mjmodel();
    mjData* d = mjdata_lock();

    mjData* dcpy = mj_makeData(m);

    // copy simulation state
    dcpy->time = d->time;
    mju_copy(dcpy->qpos, d->qpos, m->nq);
    mju_copy(dcpy->qvel, d->qvel, m->nv);
    mju_copy(dcpy->act, d->act, m->na);

    // copy mocap body pose and userdata
    mju_copy(dcpy->mocap_pos, d->mocap_pos, 3 * m->nmocap);
    mju_copy(dcpy->mocap_quat, d->mocap_quat, 4 * m->nmocap);
    mju_copy(dcpy->userdata, d->userdata, m->nuserdata);

    // copy warm-start acceleration
    mju_copy(dcpy->qacc_warmstart, d->qacc_warmstart, m->nv);

    mjdata_unlock();
    return dcpy;
}

} // namespace mjglobal
