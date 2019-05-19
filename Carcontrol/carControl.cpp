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
//#include "messages.hpp"
#include "opendlv-standard-message-set.hpp"
#include <pthread.h>

class carObj {
    uint32_t ID, Xpos, Ypos, height, width;


    public:
    carObj();
    carObj(uint32_t _ID, uint32_t _Xpos, uint32_t _Ypos, uint32_t _height, uint32_t _width);

    void print(){
        std::cout << ID <<": "<< height << ", " << width << ", " << Xpos << " ," << Ypos << std::endl;
    }
    uint32_t getID(){    
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
    uint32_t getWidth(){
        return width;
    }
};

carObj::carObj(uint32_t _ID, uint32_t _Xpos, uint32_t _Ypos, uint32_t _height, uint32_t _width){
        ID = _ID;
        Xpos = _Xpos;
        Ypos = _Ypos;
        height = _height;
        width = _width;
    }

float calculatePedel(float distance, float currentVelocity);
std::string stateView(std::vector<carObj> &snapShot, bool VERBOSE);

/*std::vector <std::string> objType;
objType.push_back("kiwicar");
objType.push_back("stopsign");
objType.push_back("noLeft");
objType.push_back("noRight");*/

int32_t main(int32_t argc, char **argv){

    std::cout << "Staring carControl" << std::endl;

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    bool logicIsRunning = 0;

    cluon::OD4Session od4Distance{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};   
    cluon::OD4Session od4Speed{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Turn{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4CarReading{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4SignReading{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::UDPSender UDPsender{"255.0.0.112", 1239};

    float baseSpeed = std::stof(commandlineArguments["s"]);
    std::string message = "";
    std::string realMessage = "";
    float speed{0.0};
    float sonicDistReading{0.0};
    carObj temp(0,0,0,0,0);
    std::vector <carObj> snapShot = {temp};
    snapShot.clear();

    // recives commands from the Car Command Software
    cluon::UDPReceiver reciverCar("225.0.0.111", 1238,[VERBOSE, &realMessage]
    (std::string &&data, std::string &&sender,  std::chrono::system_clock::time_point &&/*timepoint*/)
    noexcept {
            realMessage = data;
            if(VERBOSE == 1){
                std::cout << data << " was sent by: " << sender << std::endl;
            }
        });

    auto onDistanceReading{[VERBOSE, &speed, &baseSpeed, &sonicDistReading](cluon::data::Envelope &&envelope)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); 
                if(senderStamp == 0)
                {
                    sonicDistReading = msg.distance();
                    if(VERBOSE == 1){
                    std::cout << "The reading of the DistanceReading is: " << sonicDistReading << std::endl;
                    }
                }
            }
    };

    auto onCarReading{[&snapShot,VERBOSE](cluon::data::Envelope &&envelope)
            {   
            auto msg = cluon::extractMessage<opendlv::proxy::CarReading>(std::move(envelope));
                uint32_t ID = msg.objID();
                uint32_t Xpos = msg.Xpos();
                uint32_t Ypos = msg.Ypos();
                uint32_t height = msg.height();
                uint32_t width = msg.width();
                carObj tempCar(ID,Xpos,Ypos,height,width);
                snapShot.push_back(tempCar);
                /*if(VERBOSE == 1){
                    std::cout << "car recieved" << std::endl;
                }*/
                
            
        }
    };


    od4CarReading.dataTrigger(opendlv::proxy::CarReading::ID(), onCarReading);

    od4Distance.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

    opendlv::proxy::PedalPositionRequest pedalReq;
    opendlv::proxy::GroundSteeringRequest steerReq;
    
    const int16_t systemDelay{50};
    const int16_t delay{500};
    const int16_t turnDelay{2000};
    bool running = 1;
    std::string state ="";
    bool stopSignDetected = false;
    bool carRightDetected = false;
    bool carMidDetected = false;
    bool carLeftDetected = false;
    bool noLeftDetected = false;
    bool noRightDetected = false;
    bool atStopSign = false;
		
    // Main loop where the diffrent actions will be in place
    // State 
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    while(running != 0){      

        std::this_thread::sleep_for(std::chrono::milliseconds(systemDelay*2));
        logicIsRunning = 0;
        //state = stateView(snapShot,VERBOSE);
        int16_t stopSignLocation= -1;
        for (size_t i = 0; i < snapShot.size() && !atStopSign; ++i)
        {    
            if(VERBOSE){
                snapShot[i].print();
            }
            //Car DETECTED LÖGIC
            if (snapShot[i].getID() == 0){
                //Intersection logic for car to the right
                if (snapShot[i].getX() > 400){
                    if(VERBOSE){
                        std::cout << "Car to the RIGHT detected:" << std::endl;
                    }
                    carRightDetected = true;
                }
                //Intersection logic for car in the middle
                else if (snapShot[i].getX() > 100 && snapShot[i].getX() < 240 && snapShot[i].getWidth() < snapShot[i].getHeight()*2){
                    if(VERBOSE){
                        std::cout << "Car to the MIDDLE detected" << std::endl;
                    }
                    carMidDetected = true;
                }
            }  


            //Stop sign detected, insert good logic
            else if(snapShot[i].getID() == 1){
                stopSignDetected = true;
                if (VERBOSE){
                    std::cout << "Stop sign detected" << std::endl; 
                }
                stopSignLocation = i;

            }


            //No left detected
            else if (snapShot[i].getID() == 2){
                if(VERBOSE){
                    std::cout << "sign noLEFT detected" << std::endl;
                }
                noLeftDetected = true;
            }


            //No Right detected
            else if (snapShot[i].getID() == 3){
                if(VERBOSE){
                    std::cout << "sign noRIGHT detected" << std::endl;
                }
                noRightDetected = true;
            }
        }

        if(sonicDistReading > 30 && sonicDistReading != 0 && !atStopSign){
            pedalReq.position(0.11);
            od4Speed.send(pedalReq);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            pedalReq.position(0.0);
            od4Speed.send(pedalReq);
            if(stopSignLocation != -1){
                if(snapShot[stopSignLocation].getX() > 400){
                    int goTimes = (640 - snapShot[stopSignLocation].getX()) / 30;
                    for(int i = 0; i < goTimes; i++){
                        if(sonicDistReading > 30 && sonicDistReading != 0){
                            pedalReq.position(0.11);
                            od4Speed.send(pedalReq);
                        }
                        else{
                            pedalReq.position(0.0);
                            od4Speed.send(pedalReq);
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(150));
                    }
                    atStopSign = true;
                }
            }
        }

        else if (atStopSign){
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            if(snapShot.size() == 0){
                //Logik för att köra bil
                pedalReq.position(0.11);
                od4Speed.send(pedalReq);
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                pedalReq.position(0.0);
                od4Speed.send(pedalReq);
            }
        }
        else{
            pedalReq.position(0.0);
            od4Speed.send(pedalReq);
        }
        
        snapShot.clear();
        logicIsRunning = 1;



/*

            else if(snapShot[i]->getID() = 0){
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
        */
        // resets all the modes and then waits for a short time
   }
   
    return 0;
}

//____________________________________________________________________________________________________________________
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
//_____________________________________________________________________________________________________________________
//Captures the messages from the Object Detection and returns a state based on the messages
/*std::string stateView(std::vector<carObj> &snapShot, bool VERBOSE){
    std::string state = "";
    struct HeightCmp{
        inline bool operator() (carObj& a, carObj& b){
            return (a.getHeight() > b.getHeight());
        }
    };
    
        carObj car1("CAR",16,1,1);
        carObj car2("CAR",7,2,2);        
        carObj car3("CAR",6,3,3);
        carObj car4("SIGN",25,4,4);
        std::vector <carObj> temp2{car4,car2,car1,car3};

        snapShot = temp2;
    */
   

    // sorts the vector by height so we know what is the most prioritized
    //std::sort(snapShot.begin(),snapShot.end(), HeightCmp());
    //will print the list out before it is sorted
    /*if(VERBOSE)
    {
        std::vector <carObj> :: iterator it;

        for(it = snapShot.begin(); it != snapShot.end(); ++it)    
        { 
            carObj temp = *it;        
            temp.print();
        }
    }
    std::cout << (snapShot.begin()->getID() == "SIGN") << std::endl;


    // checks if a car or a sign is the closests
    if(snapShot.begin()->getID() == "SIGN")
    {     
        //std::cout << "WRONG PLACE" << std::endl;
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
*/



