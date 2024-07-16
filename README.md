# Custom Inflation Layer ROS2



## Introduction

Custom inflation layer ros2 Plugin implementation 

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

### Run to publish segmented maps

```bash
python3 inflation_pub.py
```

**In RViz add map and subscribe to topic**
```bash
/occupancy_grid_v1
```

### Run to publish custom inflation

```bash
python3 custom_inflation.py
```

**In RViz add map and subscribe to topic inflation**
```bash
/merged_global_costmap
```
**In terminal where inflation_pub.py script is running change number of map**