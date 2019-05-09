/*
    Author: Hakim El Amri (@Jorelsin) 
*/
#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <algorithm>
#include <list>
#include "cluon-complete.hpp"
#include "messages.hpp"

float calculatePedel(float distance, float currentVelocity);
std::string stateView(std::vector<carObj> &snapShot, int16_t delay, bool VERBOSE);

class carObj {
    std::string ID;
    uint32_t height, Xpos, Ypos;

    public:
    carObj();
    carObj(std::string _ID, uint32_t _height, uint32_t _Xpos, uint32_t _Ypos);

    void print(){
        std::cout << ID <<": "<< height << ", " << Xpos << " ," << Ypos << std::endl;
    }
    std::string getID(){    
        return ID;
    }

    uint32_t getHeight(){
        return height;
    }

    uint32_t getX(){
        return Xpos;
    }

    uint32_t getY(){
        return Ypos;
    }
};

carObj::carObj(std::string _ID, uint32_t _height, uint32_t _Xpos, uint32_t _Ypos){
        ID = _ID;
        height = _height;
        Xpos = _Xpos;
        Ypos = _Ypos;
    }

int32_t main(int32_t argc, char **argv){

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    bool logicIsRunning = 0;

    uint16_t CID{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Distance{CID};   
    cluon::OD4Session od4Speed{CID};
    cluon::OD4Session od4Turn{CID};
    cluon::OD4Session od4CarReading{CID};
    cluon::OD4Session od4SignReading{CID};
    cluon::UDPSender UDPsender{"255.0.0.112", 1239};
   
    float baseSpeed = std::stof(commandlineArguments["s"]);
    std::string message = "";
    std::string realMessage = "";
    float speed{0.0};
    float sonicDistReading{0.0};
    std::vector <carObj> snapShot;
    
    // recives commands from the Car Command Software
    cluon::UDPReceiver reciverCar("225.0.0.111", 1238,[VERBOSE, &realMessage](std::string &&data, std::string &&sender,  std::chrono::system_clock::time_point &&/*timepoint*/) noexcept {
            realMessage = data;
            if(VERBOSE == 1){
                std::cout << data << " was sent by: " << sender << std::endl;
            }
        });

    /* auto onDistanceReading{[VERBOSE, &speed, &baseSpeed, &sonicDistReading](cluon::data::Envelope &&envelope)
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
    }; */

    auto onCarReading{[&snapShot,VERBOSE,logicIsRunning](cluon::data::Envelope &&envelope)
            {   
                if(logicIsRunning == 1){
                auto msg = cluon::extractMessage<opendlv::proxy::CarReading>(std::move(envelope));
                    std::string ID = msg.objID();
                    uint32_t height = msg.height();
                    uint32_t Xpos = msg.Xpos();
                    uint32_t Ypos = msg.Ypos();
                    carObj tempCar(ID,height,Xpos,Ypos);
                    snapShot.push_back(tempCar);
                    if(VERBOSE == 1){
                        tempCar.print();
                    }
                
            }
        }
    };

    auto onSignReading{[&snapShot,VERBOSE,logicIsRunning](cluon::data::Envelope &&envelope)
            {
                if(logicIsRunning == 1){
                auto msg = cluon::extractMessage<opendlv::proxy::SignReading>(std::move(envelope));
                    std::string type = msg.type();
                    uint32_t height = msg.height();
                    uint32_t Xpos = msg.Xpos();
                    uint32_t Ypos = msg.Ypos();
                    carObj tempSign(type, height, Xpos, Ypos);
                    snapShot.push_back(tempSign);
                    if(VERBOSE == 1){
                        tempSign.print();
                    }
            }
        }
    };

    od4CarReading.dataTrigger(opendlv::proxy::CarReading::ID(), onCarReading);
    od4SignReading.dataTrigger(opendlv::proxy::SignReading::ID(), onSignReading);
   // od4Distance.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringRequest steerReq;
    
    const int16_t systemDelay{50};
    const int16_t delay{500};
    const int16_t turnDelay{2000};
    bool running = 1;
    std::string state ="";

    // Main loop where the diffrent actions will be in place
    // State 
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    while(running != 0){        

        std::this_thread::sleep_for(std::chrono::milliseconds(delay*2));
        logicIsRunning = 0;
        state = stateView(snapShot,systemDelay,VERBOSE);

        if(state == "SIGN"){
            // 35 is a shitty value
            if(snapShot.begin()->getHeight() > 35){
                //Does nothing as it is just gonna keep going
            }else{
                // Intersection locig
                pedalReq.position(0.11);
                od4Speed.send(pedalReq);
                std::this_thread::sleep_for(std::chrono::milliseconds(turnDelay/2));
                pedalReq.position(0.0);
                od4Speed.send(pedalReq);
                std::this_thread::sleep_for(std::chrono::milliseconds(turnDelay*(4*(snapShot.size()-1))));
                message = realMessage;
            }

        }else if(state == "CAR"){
            // Takes the car in fronts distance height and sets the speed based on it
            speed = calculatePedel(snapShot.begin()->getHeight(), baseSpeed);
            pedalReq.position(speed);
            od4Speed.send(pedalReq);
            if(realMessage == "STOP" || realMessage == "EXIT"){
                message = realMessage;
            }else{
                message = "FORWARD";
            }
        }

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

        }else if(message == "STOP"){
            pedalReq.position(0.0);
            steerReq.groundSteering(0.0);
            od4Speed.send(pedalReq);
            od4Turn.send(steerReq);
            
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
        

        // resets all the modes and then waits for a short time
        snapShot.clear();
        logicIsRunning = 1;
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
std::string stateView(std::vector<carObj> &snapShot, int16_t delay, bool VERBOSE){
    std::string state = "";
    struct HeightCmp{
        inline bool operator() (carObj& a, carObj& b){
            return (a.getHeight() > b.getHeight());
        }
    };
    /*
        carObj car1("a",16,1,1);
        carObj car2("b",7,2,2);        
        carObj car3("c",6,3,3);
        carObj car4("d",22,4,4);
        std::vector <carObj> snapShot{car4,car2,car1,car3};
    */
   

    // sorts the vector by height so we know what is the most prioritized
    std::sort(snapShot.begin(),snapShot.end(), HeightCmp());
    //will print the list out before it is sorted
    if(VERBOSE)
    {
        std::vector <carObj> :: iterator it;

        for(it = snapShot.begin(); it != snapShot.end(); ++it)    
        { 
            carObj temp = *it;        
            temp.print();
        }
    }

    // checks if a car or a sign is the closests
    if(snapShot.begin()->getID().compare("SIGN"))
    {      
        // checks if the sign is close enough or not
        if(snapShot.begin()->getHeight() > 30){
            state = "CAR";
        }else{
            state = "SIGN";
        }
    }else
    {
        state = "CAR";
    }

    return state;
}



