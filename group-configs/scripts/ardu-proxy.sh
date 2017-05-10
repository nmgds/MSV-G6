#! /bin/sh
docker run -ti --rm --net=host --device=/dev/ttyACM0 seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/opendlv.scaledcars/bin/miniature/ardu-proxy --cid=111 --verbose=1 --freq=20
