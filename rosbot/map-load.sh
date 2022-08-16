#!/bin/bash

docker exec -it floor-heat-mapper-localization-mapping-1 bash -c "source /opt/ros/humble/setup.bash &&  ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \"{map_url: /maps/map.yaml}\""