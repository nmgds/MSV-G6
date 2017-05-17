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
#include <bitset>

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
		
		const uint8_t sensor = 0x0;
		
		const string SEND_SERIAL_PORT = "/dev/ttyACM0"; 
        const string RECEIVE_SERIAL_PORT = "/dev/ttyACM0";
        const uint32_t BAUD_RATE = 9600;
		
		int desired_steering = 0;
		int desired_speed = 0;
		
		int counter = 0;
		int average_steering = 0;

		int sensorValues[6];
		const uint8_t sensorMask = 0x80;
		const uint8_t ultrasoundMask = 0x40;
		const uint8_t infraredMask = 0x60;

		const uint8_t ultrasoundClearMask = 0x3F;
		const uint8_t infraredClearMask = 0x1F;

		enum sensors{INFRARED_SIDE_1, INFRARED_BACK, INFRARED_SIDE_2, ULTRASOUND_FRONT, ULTRASOUND_SIDE, WHEEL_ENCODER};
		
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
			
			serial->send(makeMovingCommand(desired_speed));
			odcore::base::Thread::usleepFor(1*ONE_SECOND);			

		while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
			
			sensorBoardData.putTo_MapOfDistances(INFRARED_SIDE_1, sensorValues[INFRARED_SIDE_1]);
			sensorBoardData.putTo_MapOfDistances(INFRARED_SIDE_2, sensorValues[INFRARED_SIDE_2]);			
			sensorBoardData.putTo_MapOfDistances(INFRARED_BACK, sensorValues[INFRARED_BACK]);
			sensorBoardData.putTo_MapOfDistances(ULTRASOUND_FRONT, sensorValues[ULTRASOUND_FRONT]);
			sensorBoardData.putTo_MapOfDistances(ULTRASOUND_SIDE, sensorValues[ULTRASOUND_SIDE]);
			sensorBoardData.putTo_MapOfDistances(WHEEL_ENCODER, sensorValues[WHEEL_ENCODER]);


			Container cc(sensorBoardData);
			getConference().send(cc);



			Container c = kvs.get(automotive::VehicleControl::ID());
			double steeringValue;
			automotive::VehicleControl vc = c.getData<VehicleControl>();
			
			steeringValue = vc.getSteeringWheelAngle();
			desired_steering= (int)floor(steeringValue*(180/3.14)*-1);
			desired_speed = vc.getSpeed();

			cout<<"Steering:" << desired_steering<<endl;
			cout<<"Speed:"<<desired_speed<<endl;

			serial->send(makeMovingCommand(desired_speed));	
			
			if(counter == 3){
				average_steering = average_steering / counter;
				//send data to the serial
				if(average_steering > 30){
					average_steering = 30;
				}
				if(average_steering < -30){
					average_steering = -30;
				}
				serial->send(makeSteeringCommand(average_steering));
				//cout<<"     :Average steering:"<<average_steering<<" Steering value received:"<<steeringValue<<endl;
				average_steering = 0;
				counter = 0;
			}
			else{
				counter++;
				average_steering = average_steering + desired_steering;
			}
		}

		serial->send(makeMovingCommand(0));			
			
            }
            catch(string &exception) {
                cerr << "Serial port could not be created: " << exception << endl;
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature