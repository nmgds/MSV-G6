#! /bin/sh
docker exec -ti $(docker ps -q --latest) /opt/opendlv.scaledcars/bin/miniature/sidewaysparker --cid=111 --freq=10 --verbose=1
