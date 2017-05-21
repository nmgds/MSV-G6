

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
            bool simulation = false;

            Overtaker::Overtaker(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "overtaker") {}

            Overtaker::~Overtaker() {}

            void Overtaker::setUp() {
            // This method will be call automatically _before_ running body().
                KeyValueConfiguration kv = getKeyValueConfiguration();
                simulation = kv.getValue<uint32_t>("global.simulation") == 1;
            }

            void Overtaker::tearDown() {
            // This method will be call automatically _after_ return from body().
            }

        // This method will do the main data processing job.
            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Overtaker::body() {


                int32_t ULTRASONIC_FRONT_CENTER;
                int32_t ULTRASONIC_FRONT_RIGHT;
                int32_t INFRARED_FRONT_RIGHT;
                int32_t INFRARED_REAR_RIGHT;
                int32_t WHEELENCODER;
                double OVERTAKING_DISTANCE;


            //const double HEADING_PARALLEL = 0.04;

                if(simulation == 0) { //todo: fine tune the different angles on the wheel and distances for overtake
                  ULTRASONIC_FRONT_RIGHT = 0;
                  INFRARED_FRONT_RIGHT = 2;
                  INFRARED_REAR_RIGHT = 3;
                  ULTRASONIC_FRONT_CENTER = 1;
                  WHEELENCODER = 5;
                  OVERTAKING_DISTANCE = 40.0; 
              }
              else {
                  ULTRASONIC_FRONT_RIGHT = 0;
                  INFRARED_FRONT_RIGHT = 2;
                  INFRARED_REAR_RIGHT = 3;
                  ULTRASONIC_FRONT_CENTER = 1;
                  WHEELENCODER = 5;
                  OVERTAKING_DISTANCE = 40.0;
              }
              // Overall state machines for moving and measuring.
              enum CarStage { FORWARD, LEFT, LEFT_RIGHT, CONTINUE_ON_LEFT_LANE, RIGHT, RIGHT_LEFT };
              enum ObjectStatus { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT ,FOUND_OBJECT , FIND_OBJECT_SIDE, END_OF_THE_OBJECT};
              enum carStatus { LANE_FOLLOWING = 0, OVERTAKING = 1,PARKING = 2};


              CarStage movingStage = FORWARD;
              ObjectStatus objectStatus = FIND_OBJECT_INIT; 

        //Declaration of sensors and variables
              double distanceToObstacle = 0;
              double distanceToObstacleOld = 0;
              double usFront = 0;
              double usRight = 0;
              double irFront = 0;
              double irRear  = 0;
              double wheelEncoder = 0;



              double turnStart = 0;

              while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                usFront = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                usRight = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                irFront = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                irRear = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                wheelEncoder = sbd.getValueForKey_MapOfDistances(WHEELENCODER);

                cout << "usFront: " << usFront << endl;
                cout << "usRight: " << usRight << endl;

                cout << "0: " << sbd.getValueForKey_MapOfDistances(0) << endl;
                cout << "1: " << sbd.getValueForKey_MapOfDistances(1) << endl;
                cout << "2: " << sbd.getValueForKey_MapOfDistances(2) << endl;
                cout << "3: " << sbd.getValueForKey_MapOfDistances(3) << endl;
                cout << "4: " << sbd.getValueForKey_MapOfDistances(4) << endl;
                cout << "5: " << sbd.getValueForKey_MapOfDistances(5) << endl;

        // Create vehicle control data.
                VehicleControl vc;

        // Create vehicle status
                chalmersrevere::scaledcars::CarStatus cs;


        // Moving state machine.
                if (movingStage == FORWARD) {

                    cs.setStatus(OVERTAKING);
                    vc.setSteeringWheelAngle(0);

                }else if (movingStage == LEFT) {
                    cs.setStatus(OVERTAKING);
                //    vc.setSpeed(3);
                    vc.setSteeringWheelAngle(-0.5);
                    cerr <<"LEFT:Angle steering -1" << endl;
                    turnStart = wheelEncoder + 5;
 
                    if (irFront > 0 && irRear > 0 ){ 
                        movingStage = LEFT_RIGHT;        
                    }

                }else if(movingStage == LEFT_RIGHT){
                    cs.setStatus(OVERTAKING);  
                    vc.setSpeed(3);
                    vc.setSteeringWheelAngle(0.5);

                    if(wheelEncoder > turnStart){
                        movingStage = CONTINUE_ON_LEFT_LANE;
                    }
                        
                }else if(movingStage == CONTINUE_ON_LEFT_LANE){
                    cs.setStatus(OVERTAKING);
                    vc.setSteeringWheelAngle(0);
                    vc.setSpeed(3);
                    turnStart = wheelEncoder + 5;

                    if(irRear > 0 && irFront < 0) {
                        movingStage = RIGHT;
                    }
           
                }else if(movingStage == RIGHT){
                    cs.setStatus(OVERTAKING);
                    vc.setSpeed(3);
                    vc.setSteeringWheelAngle(1);
                    cerr <<"RIGHT:Angle steering to right" << endl;

                    if(wheelEncoder > turnStart){
                        turnStart = wheelEncoder + 2;
                        movingStage = RIGHT_LEFT;
                        }       
       
                }else if(movingStage == RIGHT_LEFT){
                    cs.setStatus(OVERTAKING);
                    vc.setSpeed(3);
                    vc.setSteeringWheelAngle(-1);

                    if(wheelEncoder > turnStart){
   
                        cerr << "Again at the beginning" << endl;
                        objectStatus = FIND_OBJECT_INIT;
                        movingStage = FORWARD;
                    }

                }

                // Measuring state machine.
                if (objectStatus == FIND_OBJECT_INIT) {
                    //cs.setStatus(LANE_FOLLOWING); 
                    distanceToObstacle = usFront;
                    cerr << "Object is this far: " << distanceToObstacle <<endl;
                    objectStatus = FIND_OBJECT;
                    distanceToObstacleOld = distanceToObstacle;

                }else if (objectStatus == FIND_OBJECT) {
                    distanceToObstacle = usFront;
                    cerr << "Obstacle is far away: " << distanceToObstacle <<endl;
                    // Approaching an obstacle (stationary or driving slower than us).
                    if ( (distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0))) {
                    // Check if overtaking shall be started.
                        objectStatus = FOUND_OBJECT;
                    }
                    distanceToObstacleOld = distanceToObstacle;

                }else if (objectStatus == FOUND_OBJECT) { // check if we have to overtake this object.
                    if (usFront <= OVERTAKING_DISTANCE || usRight > 0) {
                        movingStage = LEFT;
                        // Disable measuring until requested from moving state machine again.
                        objectStatus = DISABLE;
                  
                    } else {
                        objectStatus = FIND_OBJECT;
                    }

                }else if (objectStatus == FIND_OBJECT_SIDE) { // check if we have to overtake this object.
                    if(usRight > 0 && irRear > 0 && irFront > 0){
                        objectStatus = END_OF_THE_OBJECT;
                    }
                }

                cout << "State: " << movingStage << endl;
                cout << "StateMeasuring: " << objectStatus << endl;
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

