#! /bin/sh
docker exec -ti $(docker ps -q --latest) /opt/opendlv.scaledcars/bin/miniature/lanefollower --cid=111 --freq=20 --verbose=1
