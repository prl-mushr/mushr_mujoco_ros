# MuJoCo ROS Simulator for MuSHR

## Requirements
- [The MuSHR Simulator](https://github.com/prl-mushr/mushr_sim) (for the keyboard teleop module).
- [yaml-cpp](https://github.com/jbeder/yaml-cpp). If you've installed the entire MuSHR stack you should already have this installed.
- [MuJoCo 200](http://www.mujoco.org/).
- [ROS](https://www.ros.org/).

## Setup
When building `CMakeLists.txt` assumes the MuJoCo library is located at `$HOME/.mujoco/mujoco200_linux`. You can set the MuJoCo location in the environment variable `MUJOCO_LOCATION`. `USE_GL` can be set to 0 or 1 depending on whether you want to compile with GL.

## Usage
See the following [tutorial](https://mushr.io/tutorials/mujoco/) on setting up and installing the sim. There is another simple tutorial for simple control in the sim [here](https://mushr.io/tutorials/mujoco_figure8/).

### MuJoCo
#### Car models
The car models can be found in `models/cars`. Currently there are two template models. A base car and a car with a "pusher" attachment on the front (a flat plan for manipulating objects). Since mujoco requires elements to have unique names, these template files help with creating multiple cars with identical structures. The file `models/cars/make_car_model.py`, which can be invoked with `python make_car_model.py`, takes the template files and creates MuJoCo XML documents. It has some very rudimentary pattern matching some initial positions for the cars can be set. This functionality should be pretty easily extendible. These cars will be controllable with a target steering angle and target velocity through a ROS topic, as will be discussed below.

#### Bodies
This module will also publish 6DOF poses of free bodies, so make sure to name any bodies you would also like to track. See `launch/block.launch`, `config/block.yaml`, and `models/block.xml` for an example.

#### Top level model
The top level MuJoCo model should be placed in the `models` directory. In order to add a car to the environment, the car needs to be included at the top of the file. See `models/two_cars.xml` for an example.

### Connector configuration
The connector's configuration tells the module what bodies to publish poses for and which bodies are controllable vis the ROS interface. The configuration is a `yaml` file with two main objects:

#### Cars
`cars` is a list of YAML objects, with the following keys:
 - `name`: _Required, the name of the car._ This must match up with one of the cars in the MuJoCo model.
 - `control_topic`: _Optional, default: "controls"._ What topic control messages will be recieved on.
 - `pose_topic`: _Optional, default: "pose"._ What topic pose messages will be recieved on.
 - `initpose_topic`: _Optional, default: "initialpose"._ What topic pose messages will be recieved on.
 - `use_accel_control`: _Optional, default: "false". Whether to use velocity or acceleration to control throttle.

Each of these topics will be relative to the car, i.e. `/{ROS node name}/{car name}/{topic}`. You can specify absolute paths by adding a `/` to the begining of the topic name.

Example:
```
cars:
- name: buddy
  control_topic : controls
  pose_topic: pose
  initpose_topic: initialpose
- name: goose
  control_topic : control
  pose_topic: pose
  initpose_topic: initialpose
```

#### Bodies
`bodies` is a list of YAML objects, with the following keys:
 - `name`: _Required, the name of the body._ This must match up with one of the cars in the MuJoCo model.
 - `pose_topic`: _Optional, default: "pose"._ What topic pose messages will be recieved on.
 - `initpose_topic`: _Optional, default: "initialpose"._ What topic pose messages will be recieved on.

Example:
```
bodies:
- name: block
  pose_topic: pose
  initpose_topic: initpose
- name: ball
  pose_topic: pose
  initpose_topic: initpose
```

Similar to the cars, these topics will be relative to the body name.

## Launch

Once you have an adequate model and configuration file, a launch file can be made. Using `two_cars.launch` as an examples:

Arguments:
 - `map_server`: If you want to run a map server, you can supply a map associated with the environment.
 - `map_file`: Path to ROS map `yaml` file.
 - `environment`: The name of the MuJoCo model xml and config `yaml` file. Ex: If you're model is called `two_cars.xml` and the configuration `two_cars.yaml`, the environment is just `two_cars`.

Node parameters:
- `mj_key`: Path to your mujoco key.
- `model_file_path`: Path to model file, you shouldn't need to change this if you structured your files as described above.
- `config_file_path`: Path to configuration file, you shouldn't need to change this if you structured your files as described above.
- `viz`: When true, the MuJoCo visualization window will appear. _Note: You won't be able to stop the node by closing the window, you must shut down the node._

### Teleoperating cars.

You'll notice that a window pops up which can be used to drive the car using WASD. Topic it sends commands on is `/mux/ackermann_cmd_mux/input/teleop`. If you want to control your car using this window  you can use the `topic_tools` package to bridge the two topics:
```
$ rosrun topic_tools relay /mux/ackermann_cmd_mux/input/teleop /mushr_mujoco_ros/{car_name}/control
```
