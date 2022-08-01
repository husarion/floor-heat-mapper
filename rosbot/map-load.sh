#!/bin/bash

docker exec -it nav2-stack bash -c "source /opt/ros/galactic/setup.bash &&  ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \"{map_url: /maps/map.yaml}\""