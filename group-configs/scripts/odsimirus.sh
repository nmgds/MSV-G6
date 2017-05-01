#! /bin/sh
docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/od4/bin/odsimirus --cid=111 --freq=10
