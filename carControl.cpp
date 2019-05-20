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
#include <pthread.h>

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

float calculatePedal(float distance, float currentVelocity);
void stateView(std::vector<carObj> &snapShot, bool VERBOSE, bool &carInFront, bool &signInFront);

int32_t main(int32_t argc, char **argv){

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    cluon::OD4Session od4Distance{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};   
    cluon::OD4Session od4Speed{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Turn{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4CarReading{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4SignReading{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::UDPSender UDPsender{"255.0.0.112", 1239};
    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringRequest steerReq;
	
    const float baseSpeed = std::stof(commandlineArguments["s"]);
    const float intersectionSpeed{0.11};
    const float leftTurnAngle{0.42};
    const float rightTurnAngle{-0.28};
    const float resetValue{0.0};
    const int16_t systemDelay{50};
    const int16_t delay{500};
    const int16_t turnDelay{2000};
    const int16_t fullDealyAtIntersection{24000}

    bool running = true;
    bool turningLeft = false;
    bool turningRight = false;
    bool drivingForward = false;
    bool carInFront = false;
    bool signInFront = false;
    bool stoppedAtSign = false;
    bool stopCar = false;
    bool exitSoftware = false;

    std::string message = "";
    std::string realMessage = "";
    float speed{0.0};

    //setting up and clearing the vector<carObj> snapShot and dataFow
    carObj temp("0",0,0,0);
    std::vector <carObj> snapShot = {temp};
    std::vector <carObj> objectData = {temp};
    snapShot.clear();
    dataFlow.clear();

    // recives commands from the Car Command Software
    cluon::UDPReceiver reciverCar("225.0.0.111", 1238,[VERBOSE,]
    (std::string &&data, std::string &&sender,  std::chrono::system_clock::time_point &&/*timepoint*/)
    noexcept {
            
            //sets the various booleans to the right state for the state machine
            switch(data){
                case "FORWARD":{
                    turningLeft = false;
                    turningRight = false;
                    drivingForward = true;
                    stopCar = false;
                break;
                    }
                case "LEFT":{
                    turningLeft = true;
                    turningRight = false;
                    drivingForward = false;
                    stopCar = false;
                break;
                    }
                case "RIGHT":{
                    turningLeft = false;
                    turningRight = true;
                    drivingForward = false;
                    stopCar = false;
                break;
                    }
                case "STOP": {
                    turningLeft = false;
                    turningRight = false;
                    drivingForward = false;
                    stopCar = true;
                break;
                    }
                case "EXIT":{
                    exitSoftware = true;
                break;
                    }
            }

            if(VERBOSE == 1){
                std::cout << data << " was sent by: " << sender << std::endl;
            }
        });

    // deals with panic stop, if anything is to close enough the car will stop. 
    auto onDistanceReading{[VERBOSE, systemDelay, resetValue, &pedalReq, &od4Speed](cluon::data::Envelope &&envelope)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); 
                if(senderStamp == 0)
                {
                    float sonicDistReading = msg.distance();
                    if(sonicDistReading < 0.2){
                        pedalReq.position(resetValue)
                        od4Speed.send(pedalReq);
                        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                    }

                    if(VERBOSE == 1){
                    std::cout << "The speed of the car is: " << speed << std::endl;
                    }
                }
            }
    }; 
    // grabs the data sent by object detection and pushes it to the snapShot vector if the car is not trying run the logic
    auto onCarReading{[&objectData,VERBOSE,logicIsRunning](cluon::data::Envelope &&envelope)
            {   
                
                auto msg = cluon::extractMessage<opendlv::proxy::CarReading>(std::move(envelope));
               
                    std::string ID = msg.objID();
                    uint32_t height = msg.height();
                    uint32_t Xpos = msg.Xpos();
                    uint32_t Ypos = msg.Ypos();
                    carObj tempCar(ID,height,Xpos,Ypos);
                    objectData.push_back(tempCar);        

                if(VERBOSE == 1){
                        cout << "A " msg.objID << " was sent" << std:endl;
                    }
        }
    };

    /*
    auto onSignReading{[&snapShot,VERBOSE,logicIsRunning](cluon::data::Envelope &&envelope)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::SignReading>(std::move(envelope));
                    std::string type = msg.type();
                    uint32_t height = msg.height();
                    uint32_t Xpos = msg.Xpos();
                    uint32_t Ypos = msg.Ypos();
                    carObj tempSign(type, height, Xpos, Ypos);
                    snapShot.push_back(tempSign);
                    if(VERBOSE == 1)
                    {
                        tempSign.print();
                    }
                }
            
    };
    */

   // Setting up the datatriggers for the logic, one for object detection and one for Ultrasonic sensor
    od4CarReading.dataTrigger(opendlv::proxy::CarReading::ID(), onCarReading);
    // od4SignReading.dataTrigger(opendlv::proxy::SignReading::ID(), onSignReading);
    od4Distance.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

    // Main loop where the diffrent actions will be in place
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    while(!running){      
         // resets all the vectors so no unnecessary data is stored between iterations of the loop
        snapShot.clear();
        objectData.clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(systemDelay*2));
        snapShot = objectData;
        stateView(snapShot, VERBOSE, carInFront, signInFront);

        //will print out the snapShot vector so we know what is in it and in what order
        if(VERBOSE){
            std::vector <carObj> :: iterator it;
            for(it = snapShot.begin(); it != snapShot.end(); ++it)    
            { 
                carObj temp = *it;        
                temp.print();
            }
        }
        
        // checks if there is a sign or a car in front of the vechicle
        if(signInFront){

            speed = calculatePedel(snapShot.begin()->getHeight(), baseSpeed);
            pedalReq.position(speed);
            od4Speed.send(pedalReq);
                 // Intersection locig
            if(speed == 0){
                stoppedAtSign = true;
                if(snap.Shot.size()-1 <= 3){
                std::this_thread::sleep_for(std::chrono::milliseconds(turnDelay*(4*(snapShot.size()-1))));
            }else {
                std::this_thread::sleep_for(std::chrono::milliseconds(fullDealyAtIntersection);
            }
        }
        else if(carInFront){
            // Takes the car in fronts distance height and sets the speed based on it
            speed = calculatePedel(snapShot.begin()->getHeight(), baseSpeed);
            pedalReq.position(speed);
            od4Speed.send(pedalReq);

        }

        if(exitSoftware){
            pedalReq.position(resetValue);
            steerReq.groundSteering(resetValue);
            od4Speed.send(pedalReq);
            od4Turn.send(steerReq);

            if(VERBOSE == 1){
                UDPsender.send("GOOD NIGHT");
            }
            running = false;
        }else if(stopCar){
            pedalReq.position(resetValue);
            steerReq.groundSteering(resetValue);
            od4Speed.send(pedalReq);
            od4Turn.send(steerReq);
            
            if(VERBOSE == 1){
                UDPsender.send("The car STOPPED");
            }

        }else if(drivingForward && stoppedAtSign){
            pedalReq.position(speed);
            od4Speed.send(pedalReq);
            
            if(VERBOSE == 1){
                UDPsender.send("The car is moving FORWARD at the intersection");
            }

        }else if (turningRight && stoppedAtSign){
            pedalReq.position(intersectionSpeed);
            od4Speed.send(pedalReq);
            steerReq.groundSteering(rightTurnAngle);
            od4Turn.send(steerReq);
            
            if(VERBOSE == 1){
                UDPsender.send("The car is turning RIGHT  at the intersection");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(turnDelay));
            steerReq.groundSteering(resetValue);
            od4Turn.send(steerReq);

        }else if (turningLeft && stoppedAtSign){
            pedalReq.position(intersectionSpeed);
            od4Speed.send(pedalReq);
            steerReq.groundSteering(leftTurnAngle);
            od4Turn.send(steerReq);

            if(VERBOSE == 1){
                UDPsender.send("The car is turning LEFT  at the intersection");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(turnDelay));
            steerReq.groundSteering(resetValue);
            od4Turn.send(steerReq);

        }
    }
   
    return 0;
}
//checks a objects ditance from the car using Ultra-Sonic Sensors, panic stops if distance to small
/*float panicStop(float distance, float currentVelocity){
    float panicStop{0.0};
    float minDistance{0.15};
   
   if(distance > minDistance){
       return currentVelocity;
   }else {
       return panicStop;
   } 
    return currentVelocity;
}
*/

// this is the function to calculate how fast the car should be when following the
float calcualtePedal(float distance, float currentVelocity){
    float panicStop{0.0};
    float minDistance{35};
   
   if(distance > minDistance){
       return currentVelocity;
   }else {
       return panicStop;
   } 
    return currentVelocity;
}

//Captures the messages from the Object Detection and returns a state based on the messages
void stateView(std::vector<carObj> &snapShot, bool VERBOSE, bool &carInFront, bool &signInFront){
    
    struct HeightCmp{
        inline bool operator() (carObj& a, carObj& b){
            return (a.getHeight() > b.getHeight());
        }
    };
    
    //Base testing array to see if logic works.
    /*
        carObj car1("CAR",16,1,1);
        carObj car2("CAR",7,2,2);        
        carObj car3("CAR",6,3,3);
        carObj car4("SIGN",25,4,4);
        std::vector <carObj> temp2{car4,car2,car1,car3};
        snapShot = temp2;
    */
   

    // sorts the vector by height so we know what is the most prioritized
    std::sort(snapShot.begin(),snapShot.end(), HeightCmp());
   
    // checks if a car, sign or nothing is the closests object to it
    if(snapShot.begin()->getID() == "SIGN")
    {     
        carInFront = false;
        signInFront = true;
        
    }else if(snapShot.begin()->getID() == "CAR")
    {
        carInFront = true;
        signInFront = false;
    }else{
        carInFront = false;
        signInFront = false;
    }

}



