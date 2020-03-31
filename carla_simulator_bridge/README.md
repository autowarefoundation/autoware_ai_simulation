# Autoware in Carla
Integration of the CARLA simulator

![Autoware Runtime Manager Settings](docs/images/autoware-rviz-carla-town01-running.png)

## Requirements

- ROS Melodic
- CARLA latest
- CARLA ROS Bridge

[System requirements](https://carla.readthedocs.io/en/latest/start_quickstart/#requirements) for running CARLA.

## Opens

- object detection (especially traffic lights)
- no compliance with traffic rules (due to missing vector map)

## Setup


### Carla

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 304F9BC29914A77D &&
    sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-0.9.8/ all main"

    sudo apt install carla

#### Point cloud maps of Carla towns

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BA0F9B7406F60E23
    sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-hdmaps/ bionic main"

    sudo apt install carla-hdmaps

### Carla ROS Bridge

The [Carla ROS bridge](https://github.com/carla-simulator/ros-bridge.git) package aims at providing a simple ROS bridge for CARLA simulator.

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 81061A1A042F527D &&
    sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-ros-bridge-melodic/ bionic main"

    sudo apt install carla-ros-bridge-melodic


## Run

To run Autoware within Carla please use the following execution order:

1. Carla Server
2. Autoware (including carla-ros-bridge and additional nodes)

You need two terminals:

    #Terminal 1
    #execute Carla
    cd /opt/carla/bin
    ./CarlaUE4.sh

For details, please refer to the [CARLA documentation](https://carla.readthedocs.io/en/latest/).

    #Terminal 2

    export CARLA_MAPS_PATH=/opt/carla/HDMaps/
    source /opt/carla-ros-bridge/<ros-distro>/setup.bash
    source ~/autoware.ai/install/setup.bash
    roslaunch carla_autoware_bridge carla_autoware_bridge_with_manual_control.launch


### Multi machine setup

You can run Autoware and Carla on different machines.
To let the carla autoware bridge connect to a remote Carla Server, execute roslaunch with the following parameters

    roslaunch host:=<hostname> port:=<port number> carla_autoware_bridge carla_autoware_bridge_with_manual_control.launch

## Ego Vehicle

The setup of the sensors is defined within [sensors.json](carla_autoware_bridge/config/sensors.json).

[carla_ego_vehicle](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ego_vehicle) reads the file and spawn the ego vehicle and the sensors.


## Development support

### Set Start/End of Route

When starting the carla_autoware_bridge a random spawn point and a fixed goal is used to calculate the route.

To override this, you can use RVIZ.

![Autoware Runtime Manager Settings](docs/images/rviz_set_start_goal.png)

- selecting a Pose with '2D Pose Estimate' will delete the current ego_vehicle and respawn it at the specified position.
- selecting a Pose with '2D Nav Goal' will set a new goal within `carla_waypoint_publisher`.

#### Manual steering

Press `B` to be able to steer the ego vehicle within ROS manual control.

Internally, this is done by stopping the conversion from the Autoware control message to AckermannDrive within the node `vehiclecmd_to_ackermanndrive`. The relevant ros-topic is `/vehicle_control_manual_override`.

#### Use Carla Ground Truth Objects

You can skip the Autoware perception by using the ground truth objects from CARLA.
Therefore disable all relevant Autware perception nodes and execute:

```
rosrun carla_autoware_bridge carla_to_autoware_detected_objects
```

The objects get then published to `/tracked_objects`.

## Design

The bridge contains three Carla Clients.

1. ROS Bridge - Monitors existing actors in Carla, publishes changes on ROS Topics (e.g. new sensor data)
2. Ego Vehicle - Instantiation of the ego vehicle with its sensor setup.
3. Waypoint Calculation - Uses the Carla Python API to calculate a route.

![Design Overview](docs/images/design.png)

## Scenario Execution

It is possible to use CARLA scenario runner in conjunction with autoware: [Documentation](docs/use_scenario_runner.md).