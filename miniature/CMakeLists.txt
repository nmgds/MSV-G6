cmake_minimum_required(VERSION 3.6)
project(miniature)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        boxparker/apps/boxparker.cpp
        boxparker/build/BoxParkerTestSuite.h-TestSuite.cpp
        boxparker/include/BoxParker.h
        boxparker/src/BoxParker.cpp
        boxparker/testsuites/BoxParkerTestSuite.h
        driver/apps/driver.cpp
        driver/include/Driver.h
        driver/src/Driver.cpp
        driver/testsuites/DriverTestSuite.h
        lanedetector/apps/lanedetector.cpp
        lanedetector/build/LaneDetectorTestSuite.h-TestSuite.cpp
        lanedetector/include/LaneDetector.h
        lanedetector/src/LaneDetector.cpp
        lanedetector/testsuites/LaneDetectorTestSuite.h
        lanefollower/apps/lanefollower.cpp
        lanefollower/include/LaneFollower.h
        lanefollower/src/LaneFollower.cpp
        lanefollower/testsuites/LaneFollowerTestSuite.h
        overtaker/apps/overtaker.cpp
        overtaker/include/Overtaker.h
        overtaker/src/Overtaker.cpp
        overtaker/testsuites/OvertakerTestSuite.h
        proxy/apps/proxy.cpp
        proxy/include/Camera.h
        proxy/include/OpenCVCamera.h
        proxy/include/Proxy.h
        proxy/include/uEyeCamera.h
        proxy/src/Camera.cpp
        proxy/src/OpenCVCamera.cpp
        proxy/src/Proxy.cpp
        proxy/src/uEyeCamera.cpp
        proxy/testsuites/ProxyTestSuite.h
        sidewaysparker/apps/sidewaysparker.cpp
        sidewaysparker/include/SidewaysParker.h
        sidewaysparker/src/SidewaysParker.cpp
        sidewaysparker/testsuites/SidewaysParkerTestSuite.h
        simpledriver/apps/simpledriver.cpp
        simpledriver/include/SimpleDriver.h
        simpledriver/src/SimpleDriver.cpp
        simpledriver/testsuites/SimpleDriverTestSuite.h
        studies/example-boxparker/apps/example-boxparker.cpp
        studies/example-boxparker/include/ExampleDriverSimulation.h
        studies/example-boxparker/src/ExampleDriverSimulation.cpp
        studies/example-boxparker/testsuites/ExampleDriverSimulationTestSuite.h
        VCR/apps/VCR.cpp
        VCR/include/VCR.h
        VCR/src/VCR.cpp
        VCR/testsuites/VCRTestSuite.h)

add_executable(miniature ${SOURCE_FILES})