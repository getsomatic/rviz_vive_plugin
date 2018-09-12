# rviz plugin for HTC Vive

![Example](https://raw.githubusercontent.com/getsomatic/rviz_vive_plugin/master/media/poc.gif)

## Installation
1. Install OpenVR SDK following officical instructions [here](https://github.com/ValveSoftware/openvr).
2. Clone repository into catkin workspace and add `OPENVR` variable to the build:
    ```bash
    cd catkin_ws/src
    git clone git@github.com:getsomatic/rviz_vive_plugin.git
    cd ../..
    catkin build -DOPENVR="..."
    ```