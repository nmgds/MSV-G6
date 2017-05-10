#!/bin/bash

gnome-terminal -e "sh odsupercomponent.sh"
sleep 2
gnome-terminal -e "sh odsimvehicle.sh"
gnome-terminal -e "sh odsimirus.sh"
gnome-terminal -e "sh odcockpit.sh"
sleep 2
gnome-terminal -e "sh sidewaysparker.sh"
#gnome-terminal -e "sh odsimcamera.sh"
#gnome-terminal -e "sh proxy-camera.sh"
#sleep 2
#gnome-terminal -e "sh lanefollow.sh"
#sleep 1
#gnome-terminal -e "sh ardu-proxy.sh"
