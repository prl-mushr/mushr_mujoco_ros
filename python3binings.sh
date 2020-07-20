#!/bin/bash

SOURCES=src/bindings.cpp
        src/body_ros_connector.cpp
        src/mjglobal.cpp
        src/mushr_mujoco_util.cpp
        src/mushr_ros_connector.cpp
        src/rollout.cpp
        src/sensors.cpp
        src/sim_record.cpp
        src/sim_state.cpp
        src/simple_viz.cpp
        include/body_ros_connector.h
        include/mjglobal.h
        include/mushr_mujoco_util.h
        include/mushr_ros_connector.h
        include/rollout.h
        include/sim_record.h
        include/sim_state.h
        include/simple_viz.h
        include/types.h

c++ -O3 -Wall -shared -std=c++11 -fPIC `python3 -m pybind11 --includes` "$SOURCES" -o mushr_mujoco_py`python3-config --extension-suffix`
