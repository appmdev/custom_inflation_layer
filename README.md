# Custom Inflation Layer ROS2



## Introduction

Custom inflation layer ros2 implementation 

## Dependencies

- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Humble

## How to Use

### Create Folder

```bash
mkdir -p ~/custom_inflation_layer
cd ~/custom_inflation_layer
```

### Clone Repository

```bash
git clone  https://github.com/appmdev/custom_inflation_layer.git
cd ~/custom_inflation_layer/custom_inflation_layer
```
### Launch Simulation 

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Launch Navigation Stack

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=map.yaml
```
### Save map

```bash
python3 save_map.py
```


**Copy map.yaml file which represents loaded map in same folder**

It needs as it has configurations of the map  as height, width, resolution and so on. Example of map.yaml file is located in same folder, please replace with your actual map configuration keeping the same file name, otherwise change in code.


### Run to publish appedned gridmap of submaps

```bash
python3 srv_merged_maps.py
```

### Run to publish segmented maps

```bash
python3 occupancy_grid_objects_v3.py
```
**It will require 2 parameters:**

**Insert number of segmented object (submap)**

**Insert custom integer number for virtual inflation of that object (0 means no virtual inflation)**

### Create Folder to store temporary map

```bash
mkdir -p ~/map
```
### Run to publish saved map after each appended gridmap of submaps

**Temporary map will be saved in folder "map"**


```bash
python3 python3 load_map.py
```

**Gridmap to be published on following topic**
```bash
/global_costmap/costmap
```