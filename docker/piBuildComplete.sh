#!/bin/bash

make IMAGE=opendlv-on-opendlv-core-on-opendavinci-on-base-armhf buildComplete
make IMAGE=opendlv-on-opendlv-core-on-opendavinci-on-base-armhf createDockerImage

docker tag seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-armhf navysealswithac/scaledcars-rpi
docker login -u navysealswithac -p 123abc
docker push navysealswithac/scaledcars-rpi
docker rmi navysealswithac/scaledcars-rpi
