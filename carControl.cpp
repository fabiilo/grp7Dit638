/*
    Author: Hakim El Amri (@Jorelsin) 
*/
#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <list>
#include "cluon-complete.hpp"
#include "messages.hpp"

using namespace std;

float calculatePedel(float distance, float currentVelocity);
int16_t stateView(uint16_t CID, int16_t delay, bool VERBOSE);

class carObj{
    std::string objID;
    uint32_t height, Xpos, Ypos;

    public:
    carObj(std::string,uint32_t,uint32_t,uint32_t);
    carObj(opendlv::proxy::CarReading);
    carObj(opendlv::proxy::SignReading);

    void print(){
        std::cout << objID <<": "<< height << ", " << Xpos << " ," << Ypos << std::endl;
    }
};

carObj::carObj(std::string _objID,uint32_t _height,uint32_t _Xpos,uint32_t _Ypos){
    objID = _objID;
    height = _height;
    Xpos = _Xpos;
    Ypos = _Ypos;
}

carObj::carObj(opendlv::proxy::CarReading car){
    objID = car.objID;
    height = car.height;
    Xpos = car.Xpos;
    Ypos = car.Ypos;
}

carObj::carObj(opendlv::proxy::SignReading sign){
    objID = sign.type;
    height = sign.height;
    Xpos = sign.Xpos;
    Ypos = sign.Ypos;
}


int32_t main(int32_t argc, char **argv){

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t CID{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Distance{CID};   
    cluon::OD4Session od4Speed{CID};
    cluon::OD4Session od4Turn{CID};
    cluon::UDPSender UDPsender{"255.0.0.112", 1239};
   
    float baseSpeed = std::stof(commandlineArguments["s"]);
    // float turnAngle = std::stof(commandlineArguments["a"]);
    std::string message = "";
    float speed{0.0};
    float sonicDistReading{0.0};
    // float objectDistReading{0.0};

    auto onDistanceReading{[VERBOSE, &speed, &baseSpeed, &sonicDistReading](cluon::data::Envelope &&envelope)
            // &<variables> will be captured by reference (instead of value only)
            // Local variables are not available outside the lambda function
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); 
                if(senderStamp == 0)
                {
                    sonicDistReading = msg.distance();
                    speed = calculatePedel(sonicDistReading, baseSpeed);
                    if(VERBOSE == 1){
                    std::cout << "The speed of the car is: " << speed << std::endl;
                    }
                }
            }
    };

    // recives commands from the Car Command Software
    cluon::UDPReceiver reciverCar("225.0.0.111", 1238,[VERBOSE, &message](std::string &&data, std::string &&sender,  std::chrono::system_clock::time_point &&/*timepoint*/) noexcept {
            message = data;
            if(VERBOSE == 1){
                std::cout << data << " was sent by: " << sender << std::endl;
            }
        });

    od4Distance.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringRequest steerReq;
    
    const int16_t systemDelay{50};
    const int16_t delay{500};
    const int16_t turnDelay{2000};
    bool running = 1;

    // Main loop where the diffrent actions will be in place
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    while(running != 0){

        if(message == "FORWARD"){
            pedalReq.position(speed);
            od4Speed.send(pedalReq);
            
            if(VERBOSE == 1){
                UDPsender.send("The car is moving FORWARD");
            }

        }else if (message == "RIGHT"){
            pedalReq.position(0.12);
            od4Speed.send(pedalReq);
            steerReq.groundSteering(-0.28);
            od4Turn.send(steerReq);
            
            if(VERBOSE == 1){
                UDPsender.send("The car is turning RIGHT");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(turnDelay));
            steerReq.groundSteering(0.0);
            od4Turn.send(steerReq);
            message = "FORWARD";

        }else if (message == "LEFT"){
            pedalReq.position(0.12);
            od4Speed.send(pedalReq);
            steerReq.groundSteering(0.42);
            od4Turn.send(steerReq);

            if(VERBOSE == 1){
                UDPsender.send("The car is turning LEFT");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(turnDelay));
            steerReq.groundSteering(0.0);
            od4Turn.send(steerReq);
            message = "FORWARD";

        }else if(message == "STOP"){
            pedalReq.position(0.0);
            steerReq.groundSteering(0.0);
            od4Speed.send(pedalReq);
            od4Turn.send(steerReq);

            // std::string text ="{\"1\":{\"name\":\"MIKE\",\"surname\":\"TAYLOR\"},\"2\":{\"name\":\"TOM\",\"surname\":\"JERRY\"}}";
            // UDPsender.send(std::move(text));
            
            if(VERBOSE == 1){
                UDPsender.send("The car STOPPED");
            }

        }else if(message == "EXIT"){
            pedalReq.position(0.0);
            steerReq.groundSteering(0.0);
            od4Speed.send(pedalReq);
            od4Turn.send(steerReq);

            if(VERBOSE == 1){
                UDPsender.send("GOOD NIGHT");
            }
            running = 0;

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(systemDelay));
   }
   
    return 0;
}
//checks a objects ditance from the car using Ultra-Sonic Sensors, panic stops if distance to small
float calculatePedel(float distance, float currentVelocity){
    float panicStop{0.0};
    float minDistance{0.15};
   
   if(distance > minDistance){
       return currentVelocity;
   }else {
       return panicStop;
      
   } 
    return currentVelocity;
}

//Captures the messages from the Object Detection and returns a state based on the messages
int16_t stateView(uint16_t CID, int16_t delay, bool VERBOSE){

    int16_t state{0};
    cluon::OD4Session od4CarReading{CID};
    cluon::OD4Session od4SignReading{CID};
    list <carObj> temp, snapShot;

    carObj car1{"a",1,1,1};
    carObj car2{"b",2,2,2};        
    carObj car3{"c",3,3,3};
    carObj car4{"d",4,4,4};

    list <carObj> trial{car4,car2,car1,car3};
    trial.sort();

    list <carObj> :: iterator it; 
    for(it = trial.begin(); it != trial.end(); ++it){ 
        carObj temp = *it;
        temp.print();
    }
      auto onCarReading{[&temp,VERBOSE](cluon::data::Envelope &&envelope)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::CarReading>(std::move(envelope));
                    carObj tempCar{msg};
                    temp.push_back(tempCar);
                    if(VERBOSE == 1){
                        tempCar.print();
                    }
                
            }
    };

      auto onSignReading{[&temp,VERBOSE](cluon::data::Envelope &&envelope)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::SignReading>(std::move(envelope));
                    carObj tempSign{msg};
                    temp.push_back(tempSign);
                    if(VERBOSE == 1){
                        tempSign.print();
                    }
            }
    };



    return state;
}