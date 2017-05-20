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

            const double IR_RIGHT_FRONT = 2;
           //const double IR_RIGHT_BACK = 3;
			//const double IR_BACK = 4;
            const double WHEEL_ENCODER = 5;
			
            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;
			
			

			KeyValueConfiguration kv = getKeyValueConfiguration();
			const int speedForward = kv.getValue<int32_t>("sidewaysparker.speedForward");
			const int speedBackward = kv.getValue<int32_t>("sidewaysparker.speedBack");
			const int COUNTER_MAX = kv.getValue<int32_t>("sidewaysparker.timer");
			const int GAP_SIZE = kv.getValue<int32_t>("sidewaysparker.gapSize");
			const int FIRST_TURN = kv.getValue<int32_t>("sidewaysparker.firstTurn");
			const int SECOND_TURN = kv.getValue<int32_t>("sidewaysparker.secondTurn");
			const int TRANSITION_VALUE = kv.getValue<int32_t>("sidewaysparker.transition");
			//const int STARTING_STAGE = kv.getValue<int32_t>("sidewaysparker.startstage");
			const int READY_TO_PARK_DISTANCE = kv.getValue<int32_t>("sidewaysparker.readydistance");
			
			
			int counter = 0;
			int startParking = 0;
			
			
			
			//int lastEncoderValue = 0;
			//int freeSpaceCounter = 0;
			
			enum ParkingState {START, GO_FORWARD, READY_TO_PARK, TURN_RIGHT, WAITING_ONE, TRANSITION, WAITING_TWO, TURN_LEFT, STOP};
			enum MeasuringState {START_MEASURING, GAP_BEGIN, GAP_END};
			enum carStatus { LANE_FOLLOWING = 0, OVERTAKING = 1,PARKING = 2};
			
            ParkingState stageMoving = START;
			MeasuringState stageMeasuring = START_MEASURING;
			
			
			int encoderVal = 0;
			int readyToPark = 0;
			
			

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
				
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();
				//encoderVal = vd.getAbsTraveledPath();
				
                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                cout << "0: " << sbd.getValueForKey_MapOfDistances(0) << endl;
                cout << "1: " << sbd.getValueForKey_MapOfDistances(1) << endl;
                cout << "2: " << sbd.getValueForKey_MapOfDistances(2) << endl;
                cout << "3: " << sbd.getValueForKey_MapOfDistances(3) << endl;
                cout << "4: " << sbd.getValueForKey_MapOfDistances(4) << endl;
                cout << "5: " << sbd.getValueForKey_MapOfDistances(5) << endl;
				cout << "State: " << stageMoving << endl;
				cout << "SpeedFwd: " << speedForward << endl;
                // Create vehicle control data.
                VehicleControl vc;

                // Create vehicle status
                chalmersrevere::scaledcars::CarStatus cs;


          
				
				encoderVal = sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);

                //int start = 0;

                // Moving state machine.
                
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
				
				if(stageMoving == READY_TO_PARK){
					cout<<"Encoder val: "<<encoderVal<<endl;
					cout<<"Ready to park: "<<readyToPark<<endl;
					vc.setSpeed(speedForward);
					if(sbd.getValueForKey_MapOfDistances(3) > 0){
						if(encoderVal - readyToPark > READY_TO_PARK_DISTANCE){
							cs.setStatus(PARKING);
							vc.setSpeed(speedBackward + 2);
							vc.setSteeringWheelAngle(30);
							counter++;
							if(counter > COUNTER_MAX){
								stageMoving = TURN_RIGHT;
								counter = 0;
								startParking = encoderVal;
							}
						}
					}
				}
				
				if(stageMoving == TURN_RIGHT){
					vc.setSteeringWheelAngle(30);
					vc.setSpeed(speedBackward);
					counter++;
					if(encoderVal - startParking > FIRST_TURN){
					//if(counter > COUNTER_MAX){
						stageMoving = WAITING_ONE;
						startParking = encoderVal;
						counter = 0;
					}
				}
				if(stageMoving == WAITING_ONE){
					counter++;
					vc.setSpeed(speedForward); //Fix this
					startParking = encoderVal;
					if(counter > COUNTER_MAX){
						vc.setSpeed(0);
						counter = 0;
						stageMoving = TURN_LEFT;
					}
				}
				if(stageMoving == TRANSITION){
					vc.setSpeed(speedBackward);
					vc.setSteeringWheelAngle(0);
					if(encoderVal - startParking > TRANSITION_VALUE){
						startParking = encoderVal;
						counter = 0;
						stageMoving = WAITING_TWO;
					}
				}
				if(stageMoving == WAITING_TWO){
					counter++;
					vc.setSpeed(speedForward);
					startParking = encoderVal;
					if(counter > COUNTER_MAX){
						vc.setSpeed(0);
						counter = 0;
						stageMoving = TURN_LEFT;
					}
				}
				if(stageMoving == TURN_LEFT){
					vc.setSteeringWheelAngle(-30);
					vc.setSpeed(speedBackward);
					//counter++;
					if(encoderVal - startParking > SECOND_TURN){
					  //if(counter > 40){
						stageMoving = STOP;
					}
					
					
			//		if((sbd.getValueForKey_MapOfDistances(IR_BACK) > 0 && sbd.getValueForKey_MapOfDistances(IR_BACK) < 15) ){
//						vc.setSteeringWheelAngle(0);
//						vc.setSpeed(5);
//						startParking = sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);
				//		stageMoving = STOP;
					//}
					
					
				}
				if(stageMoving == STOP){
					vc.setSteeringWheelAngle(0);
					vc.setSpeed(0);
				}
				


				/*
				if(sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT) < 0 && stageMoving == GO_FORWARD){
					freeSpaceCounter =  freeSpaceCounter + (encoderVal - lastEncoderValue); 
				}else{
					freeSpaceCounter = 0;
				}
				
				if(freeSpaceCounter > GAP_SIZE && stageMoving == GO_FORWARD){
					counter = 0;
					readyToPark = encoderVal;
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
                            if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT) < 0)) {
                                stageMeasuring = GAP_END;
                                absPathStart = sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT);
                        }
                    break;
                    case GAP_END:
                        {
                            // Checking for sequence -, +.
                            if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT) > 0)) {
                                // Found sequence -, +.
                                stageMeasuring = GAP_BEGIN;
                                absPathEnd = sbd.getValueForKey_MapOfDistances(WHEEL_ENCODER);

                                const double GAP = (absPathEnd - absPathStart);

                                cerr << "Size = " << GAP << endl;

                                if (stageMoving == GO_FORWARD && (GAP >= GAP_SIZE)) {
									readyToPark = encoderVal;
                                    stageMoving = READY_TO_PARK;
                               }
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(IR_RIGHT_FRONT);
                        }
                    break;
                }
				
				
				if(stageMoving!=GO_FORWARD){
					// Create container for finally sending the data.
					Container c(vc);
					// Send container.
					getConference().send(c);
				}

                // Try to send car status
				if(stageMoving == GO_FORWARD){
					cs.setStatus(LANE_FOLLOWING);
				}
				else if(stageMoving == READY_TO_PARK){
					//do nothing
				}
				else{
					cs.setStatus(PARKING);
				}
                Container cont(cs);
                getConference().send(cont);


            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature
