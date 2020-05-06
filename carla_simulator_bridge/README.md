# Autoware in Carla
Integration of the CARLA simulator

![Autoware Runtime Manager Settings](docs/images/autoware-rviz-carla-town01-running.png)

## Requirements

- ROS Melodic
- CARLA 0.9.8
- CARLA ROS Bridge

[System requirements](https://carla.readthedocs.io/en/latest/start_quickstart/#requirements) for running CARLA.

## Opens

- no compliance with traffic rules (due to missing vector map)

## Setup


### CARLA

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 304F9BC29914A77D &&
    sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-0.9.8/ all main"

    sudo apt install carla-simulator

#### Point cloud maps of CARLA towns

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BA0F9B7406F60E23
    sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-hdmaps/ bionic main"

    sudo apt install carla-hdmaps

### CARLA ROS Bridge

The [CARLA ROS bridge](https://github.com/carla-simulator/ros-bridge.git) package aims at providing a simple ROS bridge for CARLA simulator.

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 81061A1A042F527D &&
    sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-ros-bridge-melodic/ bionic main"

    sudo apt install carla-ros-bridge-melodic


## Run

To run Autoware within CARLA please use the following execution order:

1. CARLA Server
2. Autoware (including carla-ros-bridge and additional nodes)

You need two terminals:

    #Terminal 1

    cd /opt/carla/bin
    ./CarlaUE4.sh

For details, please refer to the [CARLA documentation](https://carla.readthedocs.io/en/latest/).

    #Terminal 2

    export CARLA_MAPS_PATH=/opt/carla/HDMaps/

    source /opt/carla-ros-bridge/$ROS_DISTRO/setup.bash
    source ~/autoware.ai/install/setup.bash
    roslaunch carla_autoware_bridge carla_autoware_bridge_with_manual_control.launch


### Multi machine setup

You can run Autoware and CARLA on different machines.
To let the CARLA Autoware bridge connect to a remote CARLA Server, execute roslaunch with the following parameters

    roslaunch host:=<hostname> port:=<port number> carla_autoware_bridge carla_autoware_bridge_with_manual_control.launch

## Customization

### Ego Vehicle

The setup of the sensors is defined within [sensors.json](carla_autoware_bridge/config/sensors.json).

[carla_ego_vehicle](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ego_vehicle) reads the file and spawn the ego vehicle and the sensors. The spawn point can be specified by a launch file argument, otherwise a random one is used.

#### Manual Control

The ROS node [carla_manual_control](https://github.com/carla-simulator/ros-bridge/tree/master/carla_manual_control) is used to visualize the simulation (from an ego perspective). It is completely optional.

By pressing `B` it is possible to override the steering that is received by Autoware.

### Route Creation

Routes are provided by [carla_waypoint_publisher](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher). It is possible to specify a goal with RVIZ or by publishing to a topic.

## Development support

### Use CARLA Ground Truth Objects

You can skip the Autoware perception by using the ground truth objects from CARLA.
Therefore disable all relevant Autware perception nodes and execute:

```
rosrun carla_autoware_bridge carla_to_autoware_detected_objects
```

The objects get then published to `/tracked_objects`.

### Additional Functionality

There are additional nodes available in the CARLA ROS bridge repo. Please have a look [here](https://github.com/carla-simulator/ros-bridge/blob/master/README.md).

## Design

Beside several ROS-only nodes, the bridge contains three CARLA Clients.

1. ROS Bridge - Monitors existing actors in CARLA, publishes changes on ROS Topics (e.g. new sensor data)
2. Ego Vehicle - Instantiation of the ego vehicle with its sensor setup.
3. Waypoint Calculation - Uses the CARLA Python API to calculate a route.

![Design Overview](docs/images/design.png)

## Scenario Execution

It is possible to use CARLA scenario runner in conjunction with Autoware: [Documentation](docs/use_scenario_runner.md).

## Further Documentation and Support

[CARLA Documentation](https://carla.readthedocs.io/en/latest/)

[CARLA ROS Bridge Documentation](https://github.com/carla-simulator/ros-bridge/blob/master/README.md)

[CARLA Scenario Runner](https://github.com/carla-simulator/scenario_runner/blob/master/README.md)

[CARLA Discord](https://discord.gg/8kqACuC)

