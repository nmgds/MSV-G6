/**
 * overtaker - Sample application for overtaking obstacles.
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

#include "Overtaker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        Overtaker::Overtaker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "overtaker") {
        }

        Overtaker::~Overtaker() {}

        void Overtaker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void Overtaker::tearDown() {
            // This method will be call automatically _after_ return from body().

        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Overtaker::body() {
            const int32_t ULTRASONIC_FRONT_CENTER = 3;
           	const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            const int32_t INFRARED_FRONT_RIGHT = 0;
            const int32_t INFRARED_REAR_RIGHT = 2;

            const double OVERTAKING_DISTANCE = 9.0;
            //const double HEADING_PARALLEL = 0.04;


            // Overall state machines for moving and measuring.
            enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURN, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };
            enum StateMachineMeasuring { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT, FIND_OBJECT_PLAUSIBLE, HAVE_BOTH_IR, HAVE_BOTH_IR_SAME_DISTANCE, USFR_FIND, USFR_CLEAR,  END_OF_OBJECT };
            enum carStatus { LANE_FOLLOWING = 0, OVERTAKING = 1,PARKING = 2};
           
            StateMachineMoving stageMoving = FORWARD;
            StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

            // State counter for dynamically moving back to right lane. 
            int32_t stageToRightLaneRightTurn = 0;
            int32_t stageToRightLaneLeftTurn = 0;

            // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
            double distanceToObstacle = 0;
            double distanceToObstacleOld = 0;
            

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
	            // 1. Get most recent vehicle data:
	            Container containerVehicleData = getKeyValueDataStore().get(VehicleData::ID());
	            VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.
                VehicleControl vc;

                // Create vehicle status
                chalmersrevere::scaledcars::CarStatus cs;


                // Moving state machine.
                if (stageMoving == FORWARD) {
                    cs.setStatus(LANE_FOLLOWING);
                    // Go forward.
                   
                    stageToRightLaneLeftTurn = 0;
                    stageToRightLaneRightTurn = 0;

                }
                else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                	cerr << "Changing stage to Left lane Left turn" << endl;
                    // Move to the left lane: Turn left part until both IRs see something.
                    cs.setStatus(OVERTAKING);
                    vc.setSpeed(0.5);
                    vc.setSteeringWheelAngle(-0.4);

                    // State machine measuring: Both IRs need to see something before leaving this moving state.
                    if (stageMeasuring != USFR_CLEAR){
                        stageMeasuring = USFR_FIND;
                    }
                    

                    stageToRightLaneRightTurn++;
                }
                else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                	cerr << "Changing stage to Left lane Right turn" << endl;
                    // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                    cs.setStatus(OVERTAKING);
                    vc.setSpeed(0.6);
                    vc.setSteeringWheelAngle(1);

                    // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                    //stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE; 
                    // State machine measuring: Leave state when front right ultrasonic sensor shows no obstacle
                    stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE; 

                    stageToRightLaneLeftTurn++; // How long were we in this state, i.e. How long we will have to turn left before we go straight again.
                }
                else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                	cerr << "Changing stage to continue on left" << endl;
                    // Move to the left lane: Passing stage.
                    cs.setStatus(LANE_FOLLOWING);
                   
                    // Find end of object.
                    stageMeasuring = END_OF_OBJECT;
                }
                else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                	cerr << "Changing stage to right lane right turn" << endl;
                    // Move to the right lane: Turn right part.
                    cs.setStatus(OVERTAKING);
                    vc.setSpeed(0.5);
                    vc.setSteeringWheelAngle(1);

                    //stageToRightLaneRightTurn--;
                    stageToRightLaneRightTurn = stageToRightLaneRightTurn -2; 
                    cerr <<"RLRT--"<< stageToRightLaneRightTurn <<endl;

                    if (stageToRightLaneRightTurn < 7) { //need only some right turns to come back to Lane follower
                        stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                    }
                }
                else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                	cerr << "Changing stage to right lane Left turn" << endl;
                    // Move to the left lane: Turn left part.
                    cs.setStatus(OVERTAKING);
                    vc.setSpeed(0.5);
                    vc.setSteeringWheelAngle(-0.8);

                    //stageToRightLaneLeftTurn = stageToRightLaneLeftTurn -2;
                    stageToRightLaneLeftTurn--;
                    if (stageToRightLaneLeftTurn < 10) {
                        // Start over.
                        stageMoving = FORWARD;
                        stageMeasuring = FIND_OBJECT_INIT;

                        distanceToObstacle = 0;
                        distanceToObstacleOld = 0;

                    }
                }

                // Measuring state machine.
                if (stageMeasuring == FIND_OBJECT_INIT) { 
                             distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                    stageMeasuring = FIND_OBJECT;
                    distanceToObstacleOld = distanceToObstacle;
                }
                else if (stageMeasuring == FIND_OBJECT) {
                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);

                    // Approaching an obstacle (stationary or driving slower than us).
                    if ( (distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0))) {
                        // Check if overtaking shall be started.
                        stageMeasuring = FIND_OBJECT_PLAUSIBLE;

                    }

                    distanceToObstacleOld = distanceToObstacle;
                }
                else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) { // check if we have to overtake this object.
                    if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE || (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT) > 0)) {
                        stageMoving = TO_LEFT_LANE_LEFT_TURN;
                
                        // Disable measuring until requested from moving state machine again.
                        stageMeasuring = DISABLE;
                    }
                    else {
                        stageMeasuring = FIND_OBJECT;
                    }
                }
                else if (stageMeasuring == HAVE_BOTH_IR) {
                    // Remain in this stage until both IRs see something.
                    if ( (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0) ) {
                        // Turn to right.
                        stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                    }
                }
                else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                    // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
                    // and the driven parts of the turn are plausible.
                    const double IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    const double IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                     cerr <<"BOTH SENSORS RLRT--"<< stageToRightLaneRightTurn <<endl;
                     cerr <<"BOTH SENSORS RLFT--"<< stageToRightLaneLeftTurn <<endl;
                     cerr <<"SENSOR FR--"<< IR_FR <<endl;
                     cerr <<"SENSOR RR--"<< IR_RR <<endl;

                    //if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) || (fabs(stageToRightLaneRightTurn - stageToRightLaneLeftTurn +1) < 1)) { //May have to add fabs on turns and add a max limit to difference
                    //if ((fabs(stageToRightLaneRightTurn - stageToRightLaneLeftTurn) < 1)) { //May have to add fabs on turns and add a max limit to difference
                     //  if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) && (fabs(stageToRightLaneLeftTurn - stageToRightLaneRightTurn) > 0)){
                    //if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) || (fabs(stageToRightLaneRightTurn - stageToRightLaneLeftTurn) < 1)){ 
                    if (((stageToRightLaneRightTurn - stageToRightLaneLeftTurn) < 1)){ 
                        // Straight forward again.
                        stageMoving = CONTINUE_ON_LEFT_LANE;
                    }
                }
                 else if (stageMeasuring == USFR_FIND) {
                    // Remain in this stage until front ultrasonic sensor finds obstacle
                    const double US_FR = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                    
                    if (US_FR > -1 && US_FR < OVERTAKING_DISTANCE) {
                        // Straight forward again.
                        stageMeasuring = USFR_CLEAR;
                    }
                }
                else if (stageMeasuring == USFR_CLEAR) {
                    // Remain in this stage until front ultrasonic sensor shows no obstacle
                    const double US_FR = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                    
                    if (US_FR < 0 || US_FR > 6) {
                        // Straight forward again.
                        stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                    }
                }
                else if (stageMeasuring == END_OF_OBJECT) {
                    // Find end of object.
                    double distFront = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    distanceToObstacle = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT); // When rear sensor sees nothing we are clear to move back

                    if ((distanceToObstacle < 0) && (distFront < 0))  {
                        // Move to right lane again.
                        stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

                        // Disable measuring until requested from moving state machine again.
                        stageMeasuring = DISABLE;
                    }
                }

                cerr << "State: " << stageMoving << endl;
                cerr << "StateMeasuring: " << stageMeasuring << endl;

                // Only send vehicle data if controlling the vehicle from overtaker
                if (cs.getStatus() == OVERTAKING){
                    // Create container for finally sending the data.
                    Container c(vc);
                    // Send container.
                    getConference().send(c);
                }
                

                // Try to send cat status
                Container cont(cs);
                getConference().send(cont);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

