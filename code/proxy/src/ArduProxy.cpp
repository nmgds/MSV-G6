/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
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
#include <sstream>
#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <memory>


#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include "automotivedata/generated/automotive/VehicleData.h"

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

#include "ArduProxy.h"
#include "SerialReceiveBytes.hpp"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;
		using namespace odcore::wrapper;

		int sensorValues[6];

		enum sensors{ULTRASOUND_SIDE, ULTRASOUND_FRONT INFRARED_SIDE_1, INFRARED_SIDE_2, INFRARED_BACK, WHEEL_ENCODER};
		
        ArduProxy::ArduProxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "ardu-proxy"),
            	sensorBoardData()
        {}

        ArduProxy::~ArduProxy() {
        }

        void ArduProxy::setUp() {
            // This method will be call automatically _before_ running body().

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();           
        }

        void ArduProxy::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

	void SerialReceiveBytes::nextString(const std::string &s){
		if(s.length()==4){
			uint8_t mask = s.at(0);
			uint8_t number = s.at(1);
			switch(mask){
				case 0x01:
					if(number <= 0 || number >= 55){
						sensorValues[0] = -1;
					}else{
						sensorValues[0] = number;
					}
					break;
				case 0x02:
					if(number <= 0 || number >= 55){
						sensorValues[1] = -1;
					}else{
						sensorValues[1] = number;
					}
					break;
				case 0x04:
					if(number <= 0 || number >= 20){
						sensorValues[2] = -1;
					}else{
						sensorValues[2] = number;
					}
					break;
				case 0x08:
					if(number <= 0 || number >= 20){
						sensorValues[3] = -1;
					}else{
						sensorValues[3] = number;
					}
					break;
				case 0x10:
					if(number <= 0 || number >= 20){
						sensorValues[4] = -1;
					}else{
						sensorValues[4] = number;
					}
					break;
				case 0x20:
						sensorValues[5] = number;
					break;
				default:
					break;
			}
		}
	}
		
		string ArduProxy::makeSteeringCommand(int steering){
			uint8_t payload = 0x0;
			uint8_t steerMask = 0x80;
			uint8_t positiveMask = 0x20;
			string convertedPayload;

			//set the first bit because it is a steering command
			payload = payload | steerMask;
			//set bit 5 because it is a positive number
			if(steering>0){
				payload = payload | positiveMask;
			}
			else{
				//number is negative and needs to be adjuested
				steering = 0 - steering;
			}

			//set the last couple of bits with the steering angle
			payload = payload | steering;
			
			//convert to a string to send through the serial
			convertedPayload = string(1, payload);
			return convertedPayload;
			
			
		}
		string ArduProxy::makeMovingCommand(int moving){
			uint8_t payload = 0x0;
			uint8_t positiveMask = 0x20;
			string convertedPayload;

			//set bit 5 because it is a positive number
			if(moving>0){
				payload = payload | positiveMask;
			}
			else{
				//number is negative and needs to be adjuested
				moving = 0 - moving;
			}

			//set the last couple of bits with the moving speed
			payload = payload | moving;
			
			//convert to a string to send through the serial
			convertedPayload = string(1, payload);
			return convertedPayload;
		}


        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode ArduProxy::body() {
           

			KeyValueDataStore &kvs = getKeyValueDataStore();
			KeyValueConfiguration kv = getKeyValueConfiguration();
			const string SEND_SERIAL_PORT = kv.getValue<string>("ardu-proxy.serialPort");
        	const uint32_t BAUD_RATE = kv.getValue<uint32_t>("ardu-proxy.serialSpeed");

			int steeringValue = 0;
			int speedValue = 0;
			int lastSteering = 0;
			
            try {			
                std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SEND_SERIAL_PORT, BAUD_RATE));

				SerialReceiveBytes handler;
				serial->setStringListener(&handler);

				// Start receiving bytes.
				serial->start();
				cout<<"Serial port created."<<endl;
				
				//delay for the serial to connect
				const uint32_t ONE_SECOND = 1000 * 1000;
				odcore::base::Thread::usleepFor(1*ONE_SECOND);
					

				while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
					
					//get and send sensors
					sensorBoardData.putTo_MapOfDistances(INFRARED_SIDE_1, sensorValues[INFRARED_SIDE_1]);
					sensorBoardData.putTo_MapOfDistances(INFRARED_SIDE_2, sensorValues[INFRARED_SIDE_2]);			
					sensorBoardData.putTo_MapOfDistances(INFRARED_BACK, sensorValues[INFRARED_BACK]);
					sensorBoardData.putTo_MapOfDistances(ULTRASOUND_FRONT, sensorValues[ULTRASOUND_FRONT]);
					sensorBoardData.putTo_MapOfDistances(ULTRASOUND_SIDE, sensorValues[ULTRASOUND_SIDE]);
					sensorBoardData.putTo_MapOfDistances(WHEEL_ENCODER, sensorValues[WHEEL_ENCODER]);

					Container cc(sensorBoardData);
					getConference().send(cc);
					
					Container c = kvs.get(automotive::VehicleControl::ID());
					automotive::VehicleControl vc = c.getData<VehicleControl>();
					double readSteering;
					readSteering = vc.getSteeringWheelAngle();
					steeringValue= (int)floor(readSteering*(180/3.14)*-1);
					speedValue = vc.getSpeed();

					cout<<"Steering:" << steeringValue<<endl;
					cout<<"Speed:"<<speedValue<<endl;

					
					serial->send(makeMovingCommand(speedValue));	
					
					if(steeringValue != lastSteering){
						//adjust steering
						if(steeringValue > 30){
								steeringValue = 30;
							}
						if(steeringValue < -30){
							steeringValue = -30;
						}
						serial->send(makeSteeringCommand(steeringValue));
						lastSteering = steeringValue;
					}
					
				}

			//stop the car and adjust the wheels
			serial->send(makeMovingCommand(0));		
			serial->send(makeSteeringCommand(0));	
				
				}
            catch(string &exception) {
                cerr << "Serial port could not be created: " << exception << endl;
				odcore::base::Thread::usleepFor(1000 * 1000);
					
				body();
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature
