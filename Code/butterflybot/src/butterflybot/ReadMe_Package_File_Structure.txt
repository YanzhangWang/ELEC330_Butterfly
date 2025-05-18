~/butterflybot/         <-- workspace root
├── src/
│   ├── butterflybot/   <-- The ROS 2 package
│   │   ├── butterflybot/
│   │   │   ├── __pycache__
│   │   │   ├── __init__.py
│   │   │   ├── joint_controller.py
│   │   │   ├── object_detection_node.py
│   │   │   ├── transform_broadcaster.py
│   │   │   └── map_save_panel.py
│   │   ├── config/
│   │   │   └──butterflybot_2d.lua
│   │   ├── launch/
│   │   │   ├── display.launch
│   │   │   ├── gazebo.launch
│   │   │   ├── launch.py
│   │   │   ├── object_detection.launch.py
│   │   │   └── cartographer.launch.py
│   │   ├── meshes/
│   │   │   ├── base_link.stl
│   │   │   ├── link_front_left.stl
│   │   │   ├── link_front_right.stl
│   │   │   ├── link_rear_left.stl
│   │   │   ├── link_rear_right.stl
│   │   │   └── (other STLs, if any)
│   │   ├── resources/
│   │   │   └── butterflybot
│   │   ├── rviz/
│   │   │   └── butterflybot.rviz
│   │   ├── textures/
│   │   ├── urdf/
│   │   │   ├── butterflybot.sdf
│   │   │   ├── butterflybot.urdf
│   │   │   ├── Butterflybot.csv
│   │   │   └── env_ws.sdf
│   │   ├── package.xml
│   │   └── setup.py
│   └── (other packages, if any)
