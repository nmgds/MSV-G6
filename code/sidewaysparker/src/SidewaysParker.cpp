/**
 * sidewaysparker - Sample application for realizing a sideways parking car.
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

#include <cstdio>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "odvdscaledcarsdatamodel/generated/chalmersrevere/scaledcars/CarStatus.h"

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "SidewaysParker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        SidewaysParker::SidewaysParker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "SidewaysParker") {
        }

        SidewaysParker::~SidewaysParker() {}

        void SidewaysParker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void SidewaysParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SidewaysParker::body() {

            // Sensors
            const double IR_RIGHT_FRONT = 2;
           	const double IR_RIGHT_BACK = 3;
            const double WHEEL_ENCODER = 5;
			
            // Save IR value for comparison later
            double distanceOld = 0;
            // Starting value of the gap
            double absPathStart = 0;
            // End value of the gap
            double absPathEnd = 0;
			
			
            // Load values form the configuration
			KeyValueConfiguration kv = getKeyValueConfiguration();
			// Default speed for moving forward
			const int speedForward = kv.getValue<int32_t>("sidewaysparker.speedForward");
			// Default speed for moving backward
			const int speedBackward = kv.getValue<int32_t>("sidewaysparker.speedBack");
			// Pause after the gap was found and the car has moved a certain amount forward
			const int COUNTER1_MAX = kv.getValue<int32_t>("sidewaysparker.timer1");
			// Pause between turns while backing up
			const int COUNTER2_MAX = kv.getValue<int32_t>("sidewaysparker.timer2");
			// Minimal gap size
			const int GAP_SIZE = kv.getValue<int32_t>("sidewaysparker.gapSize");
			// Distance to move while backing up and turning right
			const int FIRST_TURN = kv.getValue<int32_t>("sidewaysparker.firstTurn");
			// Distance to move while backing up and turning left
			const int SECOND_TURN = kv.getValue<int32_t>("sidewaysparker.secondTurn");
			// Distance to move forward after the end of the gap
			const int READY_TO_PARK_DISTANCE = kv.getValue<int32_t>("sidewaysparker.readydistance");
			// Speed value for stopping the car when moving backward
			const int STOP_BACKWARD = kv.getValue<int32_t>("sidewaysparker.stopBackward");
			// Speed value for stopping the car when moving forward
			const int STOP_FORWARD = kv.getValue<int32_t>("sidewaysparker.stopForward");

			
			// For counting the lenght of the pause
			int counter = 0;
			// Current value of the encoder
			int encoderVal = 0;
			// Saved value of the encoder to be used when counting the distance travelled
			int encoderFixed = 0;

			// States for movement
			enum ParkingState {START, GO_FORWARD, PREPARE_TO_PARK, READY_TO_PARK, TURN_RIGHT, WAITING, TURN_LEFT, STOP};
			// States for measuring
			enum MeasuringState {START_MEASURING, GAP_BEGIN, GAP_END};
			// Global status of the car
			enum carStatus { LANE_FOLLOWING = 0, OVERTAKING = 1,PARKING = 2};

			// Setting initial states
            ParkingState stageMoving = GO_FORWARD;
			MeasuringState stageMeasuring = START_MEASURING;
			
				

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
				
                // Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();
				
                // Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();


                // Printouts for debugging 
				// cout << "0: " << sbd.getValueForKey_MapOfDistances(0) << endl;
				// cout << "1: " << sbd.getValueForKey_MapOfDistances(1) << endl;
				// cout << "2: " << sbd.getValueForKey_MapOfDistances(2) << endl;
				// cout << "3: " << sbd.getValueForKey_MapOfDistances(3) << endl;
				// cout << "4: " << sbd.getValueForKey_MapOfDistances(4) << endl;
				// cout << "5: " << sbd.getValueForKey_MapOfDistances(5) << endl;
				// cout << "State: " << stageMoving << endl;
				// cout << "SpeedFwd: " << speedForward << endl;


                // Create vehicle control data.
                VehicleControl vc;
                // Create vehicle status
                chalmersrevere::scaledcars::CarStatus cs;
          
				// Get current absolute path travelled
				encoderVal = sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);

                // Moving state machine.
                	
				// Get the car moving and give control to the lanefollower
				if(stageMoving == START){
					vc.setSteeringWheelAngle(0);
					vc.setSpeed(speedForward+1);
					cs.setStatus(LANE_FOLLOWING);
					if(encoderVal > 2){
					stageMoving = GO_FORWARD;
					}
				}
				
				if(stageMoving == GO_FORWARD){

					 vc.setSteeringWheelAngle(0);
					 vc.setSpeed(speedForward);
				}

				// When proper gap is detected and side back IR detects the end of the gap go to next stage
				if(stageMoving == PREPARE_TO_PARK){
					vc.setSpeed(speedForward);
					if(sbd.getValueForKey_MapOfDistances(IR_RIGHT_BACK)>0){
						encoderFixed = encoderVal;
						stageMoving = READY_TO_PARK;
					}
				
				}
				
				// Car goes forwared for set amount of distance to be a little bit ahead of the gap
				// Get back control form the lanefollower
				// Prepare the wheels for backing up by turning them right
				// Pause for a set amount and switch to next stage
				if(stageMoving == READY_TO_PARK){
					vc.setSpeed(speedForward);
					if(encoderVal - encoderFixed >= READY_TO_PARK_DISTANCE){
						cs.setStatus(PARKING);
						vc.setSpeed(STOP_FORWARD); //STOP
						vc.setSteeringWheelAngle(30); //turn your wheels before going backwards (for the encoder)
						counter++;
						if(counter > COUNTER1_MAX){
							stageMoving = TURN_RIGHT;
							counter = 0;
							encoderFixed = encoderVal;
						}
					}
				}
				
				// Go backward for a set distance while wheels are turned left
				if(stageMoving == TURN_RIGHT){
					vc.setSteeringWheelAngle(30);
					vc.setSpeed(speedBackward);
					if(encoderVal - encoderFixed > FIRST_TURN){
						stageMoving = WAITING;
						encoderFixed = encoderVal; //Restart position
						counter = 0;
					}
				}

				// Pause for a set amount and prepare wheels turning left
				if(stageMoving == WAITING){
					counter++;
					vc.setSpeed(STOP_BACKWARD); //STOP
					vc.setSteeringWheelAngle(-30);
					if(counter > COUNTER2_MAX){
						vc.setSpeed(0);
						counter = 0;
						encoderFixed = encoderVal; //Restart position
						stageMoving = TURN_LEFT;
					}
				}

				// Go backward for a set istance while wheels are turned left
				if(stageMoving == TURN_LEFT){
					vc.setSteeringWheelAngle(-30);
					vc.setSpeed(speedBackward);					
					
					if(encoderVal - encoderFixed > SECOND_TURN){
						vc.setSpeed(STOP_BACKWARD); //STOP
						stageMoving = STOP;
						counter = 0;
					}
				}
				
				// Stop the car
				if(stageMoving == STOP){
					counter++;
					vc.setSpeed(STOP_BACKWARD); //STOP
					if(counter > COUNTER1_MAX){
						vc.setSteeringWheelAngle(0);
						vc.setSpeed(0);
					}
				}
				

				//TODO: do we leave this? I would say delete it

				/*
				if(sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT) < 0 && stageMoving == GO_FORWARD){
					freeSpaceCounter =  freeSpaceCounter + (encoderVal - lastEncoderValue); 
				}else{
					freeSpaceCounter = 0;
				}
				
				if(freeSpaceCounter > GAP_SIZE && stageMoving == GO_FORWARD){
					counter = 0;
					encoderFixed = encoderVal;
					stageMoving = READY_TO_PARK;
					cs.setStatus(PARKING);
				}
				
				
				lastEncoderValue = encoderVal;
				*/
				
				
				
				
                // Measuring state machine.
                switch (stageMeasuring) {
                    case START_MEASURING:
                        {
                            // Initialize measurement.
                            distanceOld = sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT);
                            stageMeasuring = GAP_BEGIN;
                        }
                    break;
                    case GAP_BEGIN:
                        {	
                        	// Keep updating front right IR values
                        	// When car passes the object andd sees a free space, go to next stage to
                        	// start counting the gap size
                            if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT) < 0)) {
                                stageMeasuring = GAP_END;
                                absPathStart = sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT);
                        }
                    break;
                    case GAP_END:
                        {
                            // Keep counting free space
                            // If an object detected - measure length of free space
                            if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT) > 0)) {
                                
                                stageMeasuring = GAP_BEGIN;
                                absPathEnd = sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);

                                const double GAP = (absPathEnd - absPathStart);

                                cerr << "Size = " << GAP << endl;

                                // If gap size matches minimal set gap size and car is GO_FORWARD state
                                // set moving stage to prepare for parking procedure 
                                if (stageMoving == GO_FORWARD && (GAP >= GAP_SIZE)) {
                                    stageMoving = PREPARE_TO_PARK;
									counter = 0;
                               }
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT);
                        }
                    break;
                }
				
				// TODO: check if this is really ok
				if(stageMoving!=GO_FORWARD){
					// Create container for finally sending the data.
					Container c(vc);
					// Send container.
					getConference().send(c);
				}

                // TODO: check if this is right, I think we check these things above in states
                // 			also I think we should just leave those two lines after else 
				if(stageMoving == GO_FORWARD){
					cs.setStatus(LANE_FOLLOWING);
				}
				else if(stageMoving == READY_TO_PARK){
					//do nothing
				}
				else{
					cs.setStatus(PARKING);
				}
				// These
                Container cont(cs);
                getConference().send(cont);


            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature

