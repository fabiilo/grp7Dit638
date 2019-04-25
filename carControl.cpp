/*
    Author: Hakim El Amri (@Jorelsin) 

*/
#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include "cluon-complete.hpp"
#include "messages.hpp"

float calculatePedel(float distance, float currentVelocity);

int32_t main(int32_t argc, char **argv){

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    std::cout << commandlineArguments["cid"] << "\n";
    cluon::OD4Session od4Distance{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};   
    cluon::OD4Session od4Speed{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Turn{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    float baseSpeed = std::stof(commandlineArguments["s"]);
    float turnAngle = std::stof(commandlineArguments["a"]);
    float speed{0.0};
    

    float tempDistReading{0.0};
    auto onDistanceReading{[&speed, &baseSpeed, &tempDistReading](cluon::data::Envelope &&envelope)
            // &<variables> will be captured by reference (instead of value only)
            // Local variables are not available outside the lambda function
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); 
                if(senderStamp == 0)
                {
                    tempDistReading = msg.distance();
                    speed = calculatePedel(tempDistReading, baseSpeed);
                    std::cout << "The speed of the car is: " << speed << std::endl;
                }
            }
    };

    od4Distance.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
   
    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringRequest steerReq;
    const int16_t delay{2000};
   
    pedalReq.position(speed);
    steerReq.groundSteering(turnAngle);
    od4Speed.send(pedalReq);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    pedalReq.position(0.0);
    steerReq.groundSteering(0.0);
    
    return 0;
}
//checks a objects ditance from the car, panic stops if distance too small
float calculatePedel(float distance, float currentVelocity){
    // 0.10 minimum
    // 0.15 - 0.20 it needs to slowdown
    // 0.20 - 0.30 good distance
    // above 0.30 needs to speed up
    float panicStop{0.0};
    float minDistance{0.15};
   
   if(distance > minDistance){
       return currentVelocity;
   }else {
       currentVelocity = panicStop;
   } 
    return currentVelocity;
}