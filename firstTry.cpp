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

    float baseSpeed = std::stof(commandlineArguments["speed"]);
    float speed{0.0};
    std::cout << "The speed is set to: " << baseSpeed << std::endl;
 
    float tempDistReading{0.0};
    auto onDistanceReading{[&speed, &baseSpeed, &tempDistReading](cluon::data::Envelope &&envelope)
            // &<variables> will be captured by reference (instead of value only)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); // Local variables are not available outside the lambda function
                if(senderStamp == 0)
                {
                    tempDistReading = msg.distance(); // Corresponds to odvd message set
                    speed = calculatePedel(tempDistReading, baseSpeed);
                    std::cout << "The speed of the car is: " << speed << std::endl;
                }
            }
    };

    od4Distance.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
   
    opendlv::proxy::PedalPositionRequest pedalReq;
    const int16_t delay{50};
    // bool loopCheck = true; 
    // loopCheck = std::thread checkLoop(breakLoop());

    while(od4Speed.isRunning()){

        pedalReq.position(speed);
        od4Speed.send(pedalReq);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));

    }

    pedalReq.position(0.0);
    od4Speed.send(pedalReq);
    return 0;
}

float calculatePedel(float distance, float currentVelocity){
    // 0.10 minimum
    // 0.15 - 0.20 it needs to slowdown
    // 0.20 - 0.30 good distance
    // above 0.30 needs to speed up
    float panicStop{0.0};
    float minDistance{0.15};
    float avgDistance{0.25};
    float maxDistance{0.35};
   
   if(distance > maxDistance){
       currentVelocity = currentVelocity;
   }else if(distance <= avgDistance && distance >= minDistance){
       currentVelocity = currentVelocity;
   }else if(distance > avgDistance && distance <= maxDistance){
       return currentVelocity;
   }else if(distance < minDistance){
       currentVelocity = panicStop;
   } 
    return currentVelocity;
}