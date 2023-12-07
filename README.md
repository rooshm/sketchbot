# sketchbot
Course project for rob514

## Bringup ur5
```bash
ros2 launch drawbot drawbot_ur_bringup.launch.py 
```

## Starting Drawbot
```bash
ros2 launch drawbot drawbot.launch.py
```

## Invoke service to start draw pipeline
```bash
ros2 service call /drawbot sketchbot_interfaces/srv/Drawbot
```


## Starting Moveit with custom config
```bash
ros2 launch drawbot_moveit_config ur_moveit.launch.py ur_type:=ur5e
```

## Starting Gazebo w Controllers + Moveit
```bash
# has a 1 minute wait time in ur_sim_control.launch.py to ensure gazebo finishes spawning
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5e use_sim_time:=True
```

![image](./data/success.jpg)
