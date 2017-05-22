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
           	const double IR_RIGHT_BACK = 3;
			//const double IR_BACK = 4;
            const double WHEEL_ENCODER = 5;
			
            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;
			
			

			KeyValueConfiguration kv = getKeyValueConfiguration();
			const int speedForward = kv.getValue<int32_t>("sidewaysparker.speedForward");
			const int speedBackward = kv.getValue<int32_t>("sidewaysparker.speedBack");
			const int COUNTER1_MAX = kv.getValue<int32_t>("sidewaysparker.timer1");
			const int COUNTER2_MAX = kv.getValue<int32_t>("sidewaysparker.timer2");
			const int GAP_SIZE = kv.getValue<int32_t>("sidewaysparker.gapSize");
			const int FIRST_TURN = kv.getValue<int32_t>("sidewaysparker.firstTurn");
			const int SECOND_TURN = kv.getValue<int32_t>("sidewaysparker.secondTurn");
			const int READY_TO_PARK_DISTANCE = kv.getValue<int32_t>("sidewaysparker.readydistance");
			const int STOP_BACKWARD = kv.getValue<int32_t>("sidewaysparker.stopBackward");
			const int STOP_FORWARD = kv.getValue<int32_t>("sidewaysparker.stopForward");

			
			
			int counter = 0;
			int startParking = 0;

			enum ParkingState {START, GO_FORWARD, PREPARE_TO_PARK, READY_TO_PARK, TURN_RIGHT, WAITING, TURN_LEFT, STOP};
			enum MeasuringState {START_MEASURING, GAP_BEGIN, GAP_END};
			enum carStatus { LANE_FOLLOWING = 0, OVERTAKING = 1,PARKING = 2};
			
            ParkingState stageMoving = GO_FORWARD;
			MeasuringState stageMeasuring = START_MEASURING;
			
			
			int encoderVal = 0;
			int encoderFixed = 0;			

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

				if(stageMoving == PREPARE_TO_PARK){
					vc.setSpeed(speedForward);
					if(sbd.getValueForKey_MapOfDistances(IR_RIGHT_BACK)>0){
						encoderFixed = encoderVal;
						stageMoving = READY_TO_PARK;
					}
				
				}
				
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
							startParking = encoderVal;
						}
					}
				}
				
				if(stageMoving == TURN_RIGHT){
					vc.setSteeringWheelAngle(30);
					vc.setSpeed(speedBackward);
					if(encoderVal - startParking > FIRST_TURN){
						stageMoving = WAITING;
						startParking = encoderVal; //Restart position
						counter = 0;
					}
				}
				if(stageMoving == WAITING){
					counter++;
					vc.setSpeed(STOP_BACKWARD); //STOP
					vc.setSteeringWheelAngle(-30);
					if(counter > COUNTER2_MAX){
						vc.setSpeed(0);
						counter = 0;
						startParking = encoderVal; //Restart position
						stageMoving = TURN_LEFT;
					}
				}
				if(stageMoving == TURN_LEFT){
					vc.setSteeringWheelAngle(-30);
					vc.setSpeed(speedBackward);					
					
					if(encoderVal - startParking > SECOND_TURN){
						vc.setSpeed(STOP_BACKWARD); //STOP
						stageMoving = STOP;
						counter = 0;
					}
				}
				
				if(stageMoving == STOP){
					counter++;
					vc.setSpeed(STOP_BACKWARD); //STOP
					if(counter > COUNTER1_MAX){
						vc.setSteeringWheelAngle(0);
						vc.setSpeed(0);
					}
				}
				


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
                                    stageMoving = PREPARE_TO_PARK;
									counter = 0;
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
