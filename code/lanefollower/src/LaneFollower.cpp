/**
 * lanefollower - Sample application for following lane markings.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/base/Lock.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"

#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/CarStatus.h"

#include "LaneFollower.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace automotive::miniature;

		using namespace cv;

        LaneFollower::LaneFollower(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "lanefollower"),
            m_hasAttachedToSharedImageMemory(false),
            m_sharedImageMemory(),
            m_image(NULL),
            gray(NULL),
            thresh(NULL),
            blur(NULL),
            m_debug(false),
            m_font(),
            m_previousTime(),
            m_eSum(0),
            m_eOld(0),
            m_vehicleControl() {}

        LaneFollower::~LaneFollower() {}

        void LaneFollower::setUp() {
	        // This method will be call automatically _before_ running body().
	        if (m_debug) {
		        // Create an OpenCV-window.
		        cvNamedWindow("WindowShowImage", CV_WINDOW_AUTOSIZE);
		        cvMoveWindow("WindowShowImage", 300, 100);
	        }
        }

        void LaneFollower::tearDown() {
	        // This method will be call automatically _after_ return from body().
	        if (gray != NULL) {
		        cvReleaseImage(&gray);
	        }

	        if (m_debug) {
		        cvDestroyWindow("WindowShowImage");
	        }
        }

        bool LaneFollower::readSharedImage(Container &c) {
	        bool retVal = false;

	        if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
		        SharedImage si = c.getData<SharedImage> ();

		        // Check if we have already attached to the shared memory.
		        if (!m_hasAttachedToSharedImageMemory) {
			        m_sharedImageMemory
					        = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
							        si.getName());
		        }

		        // Check if we could successfully attach to the shared memory.
		        if (m_sharedImageMemory->isValid()) {
			        // Lock the memory region to gain exclusive access using a scoped lock.
                    Lock l(m_sharedImageMemory);
			        const uint32_t numberOfChannels = 3;
			        // For example, simply show the image.
			        if (m_image == NULL) {
				        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, numberOfChannels);
				        // Creating IplImages for image processing (grayscale and thresholding) 
				        gray = cvCreateImage(cvSize(m_image->width, m_image->height), IPL_DEPTH_8U, 1);
				        blur = cvCreateImage(cvSize(gray->width, gray->height), IPL_DEPTH_8U, 1);
				        thresh = cvCreateImage(cvSize(gray->width, gray->height), IPL_DEPTH_8U, 1);

			        }

			        // Copying the image data is very expensive...
			        
			        if (m_image != NULL) {
				        memcpy(m_image->imageData,
						       m_sharedImageMemory->getSharedMemory(),
						       si.getWidth() * si.getHeight() * numberOfChannels);
						       
						       if (m_image != NULL && gray != NULL && thresh != NULL && blur != NULL){
								   
							    // Converting m_image to gray scale
						       cvCvtColor(m_image, gray, CV_RGB2GRAY);
						       
						       // Gaussian threshold on gray
						       //cvAdaptiveThreshold(gray, thresh, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV,51,15);
						       //cvThreshold(gray, thresh, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
						       cvSmooth(gray, blur, CV_GAUSSIAN, 9, 9, 10);
						       cvAdaptiveThreshold(blur, thresh, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV,51,15);

						   }
			        }

			        cvFlip(thresh, 0, -1);
			        cvFlip(m_image, 0, -1);


			        retVal = true;
		        }
	        }
	        return retVal;
        }

        void LaneFollower::processImage() {
            static bool useRightLaneMarking = true;
            double e = 0;

            const int32_t CONTROL_SCANLINE = 462; // 462 calibrated length to right: 280px
            const int32_t distance = 100;

            TimeStamp beforeImageProcessing;
            for(int32_t y = thresh->height - 8; y > thresh->height * .65; y -= 10) {
                
                // Search from middle to the right:
                CvScalar pixelRight;
                CvPoint right;
                right.y = y;
                right.x = -1;
                for(int x = thresh->width/2; x < thresh->width; x++) {
		            pixelRight = cvGet2D(thresh, y, x);
		            if (pixelRight.val[0] >= 200) {
                        right.x = x;
                        break;
                    }
                }
                
                // Search from middle to the left:
                CvScalar pixelLeft;
                CvPoint left;
                left.y = y;
                left.x = -1;
                for(int x = thresh->width/2; x > 0; x--) {
		            pixelLeft = cvGet2D(thresh, y, x);
		            if (pixelLeft.val[0] >= 200) {
                        left.x = x;
                        break;
                    }
                }


                if (m_debug) {
                    if (left.x > 0) {
                    	CvScalar green = CV_RGB(0, 255, 0);
                    	cvLine(m_image, cvPoint(m_image->width/2, y), left, green, 1, 8);

                        stringstream sstr;
                        sstr << (m_image->width/2 - left.x);
                    	cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 - 100, y - 2), &m_font, green);
                    }
                    if (right.x > 0) {
                    	CvScalar red = CV_RGB(255, 0, 0);
                    	cvLine(m_image, cvPoint(m_image->width/2, y), right, red, 1, 8);

                        stringstream sstr;
                        sstr << (right.x - m_image->width/2);
                    	cvPutText(m_image, sstr.str().c_str(), cvPoint(m_image->width/2 + 100, y - 2), &m_font, red);
                    }
                }

                if (y == CONTROL_SCANLINE) {
                    // Calculate the deviation error.
                    
                    if (right.x > 0) {
                        if (!useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                        
                        e = ((right.x - thresh->width/2.0) - distance)/distance;

                        useRightLaneMarking = true;
                    }
                    else if (left.x > 0) {
                        if (useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                        
                        e = (distance - (thresh->width/2.0 - left.x))/distance;

                        useRightLaneMarking = false;
                    }
                    else {
                        // If no measurements are available, reset PID controller.
                        m_eSum = 0;
                        m_eOld = 0;
                    }
                }
            }

            TimeStamp afterImageProcessing;
            cerr << "Processing time: " << (afterImageProcessing.toMicroseconds() - beforeImageProcessing.toMicroseconds())/1000.0 << "ms." << endl;

            // Show resulting features.
            if (m_debug) {
                if (thresh != NULL && m_image != NULL) {
                    cvShowImage("WindowShowImage", thresh);
                    cvShowImage("WindowMImage", m_image);
                    cvWaitKey(10);
                }
            }

            TimeStamp currentTime;
            double timeStep = (currentTime.toMicroseconds() - m_previousTime.toMicroseconds()) / (1000.0 * 1000.0);
            m_previousTime = currentTime;

            if (fabs(e) < 1e-2) {
                m_eSum = 0;
            }
            else {
                m_eSum += e;
            }
            //const double Kp = 2.5;
            //const double Ki = 8.5;
//           const double Kd = 0;

            // The following values have been determined by Twiddle algorithm.
			/*
            const double Kp = 0.4482626884328734;
            const double Ki = 3.103197570937628;
            const double Kd = 0.030450210485408566;
            */
          //  const double Kp = 1.193434;
          //  const double Ki = 0.090000;
          //  const double Kd = 0.160000;
          
           // const double Kp = 0.440;
           // const double Ki = 3.11;
           // const double Kd = 0.003;
		   
		    //const double Kp = 1.009;
            //const double Ki = 0.0123123;
			//const double Kd = 0.00;

		  // Get PID data from configuration file
            KeyValueConfiguration kva = getKeyValueConfiguration(); 
            
            const double Kp = kva.getValue<int32_t> ("lanefollower.Kp");
            const double Ki = kva.getValue<int32_t> ("lanefollower.Ki");
            const double Kd = kva.getValue<int32_t> ("lanefollower.Kd");

            const double p = Kp * e;
            const double i = Ki * timeStep * m_eSum;
            const double d = Kd * (e - m_eOld)/timeStep;
            m_eOld = e;

            const double y = p + i + d;
            double desiredSteering = 0;
            if (fabs(e) > 1e-2) {
                desiredSteering = y;
				
				//right turning
                if (desiredSteering > 25) {
                   desiredSteering = 25;
                }
              
                //left turning
                if (desiredSteering < -25) {
                   desiredSteering = -25;
                }
            }
            cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering << ", y = " << y << endl;


            // Go forward.
            m_vehicleControl.setSpeed(5);
            m_vehicleControl.setSteeringWheelAngle(desiredSteering);
        }

        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {
	        // Get configuration data.
	        KeyValueConfiguration kv = getKeyValueConfiguration();
	        m_debug = kv.getValue<int32_t> ("lanefollower.debug") == 1;

            // Initialize fonts.
            const double hscale = 0.4;
            const double vscale = 0.3;
            const double shear = 0.2;
            const int thickness = 1;
            const int lineType = 6;

            cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);

            // Parameters for overtaking.
            //const int32_t ULTRASONIC_FRONT_CENTER = 3;
            //const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            //const int32_t INFRARED_FRONT_RIGHT = 0;
            //const int32_t INFRARED_REAR_RIGHT = 2;

            //const double OVERTAKING_DISTANCE = 5.5;
            //const double HEADING_PARALLEL = 0.04;

            // Overall state machines for moving and measuring.
            enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURN, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };
            enum StateMachineMeasuring { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT, FIND_OBJECT_PLAUSIBLE, HAVE_BOTH_IR, HAVE_BOTH_IR_SAME_DISTANCE, END_OF_OBJECT };
            enum carStatus { LANE_FOLLOWING = 0, OVERTAKING = 1,PARKING = 2};

            //StateMachineMoving stageMoving = FORWARD;
            //StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

            // State counter for dynamically moving back to right lane.
            //int32_t stageToRightLaneRightTurn = 0;
            //int32_t stageToRightLaneLeftTurn = 0;

            // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
            //double distanceToObstacle = 0;
            //double distanceToObstacleOld = 0;
 
            chalmersrevere::scaledcars::CarStatus cs;
            //cs.setStatus(4);
            // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
            //double distanceToObstacle = 0;
            //double distanceToObstacleOld = 0;

            bool isActive = true;

            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                bool has_next_frame = false;

                Container cont = getKeyValueDataStore().get(chalmersrevere::scaledcars::CarStatus::ID());
                if (cont.getDataType() == chalmersrevere::scaledcars::CarStatus::ID()) {
                    cs = cont.getData<chalmersrevere::scaledcars::CarStatus>();
                    int carStatus = cs.getStatus();
                    //cerr << "Reading new value " << readValue << endl;
                    if (carStatus == LANE_FOLLOWING){
                        isActive = true;
                        cerr << "Active " << endl;
                    }else{
                        isActive = false;
                        cerr << "Inactive " << endl;
                    }
                }

                // Only exectute lanefollow algorithm if car state is set to LANE_FOLLOWING
                if (isActive){
                    // Get the most recent available container for a SharedImage.
                    Container c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());

                    if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                        // Example for processing the received container.
                        has_next_frame = readSharedImage(c);
                    }

                    // Process the read image and calculate regular lane following set values for control algorithm.
                    if (true == has_next_frame) {
                        processImage();
                    }

                    // Create container for finally sending the set values for the control algorithm.
                    Container c2(m_vehicleControl);
                    // Send container.
                    getConference().send(c2);

                }            
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature





