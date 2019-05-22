#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <algorithm>
#include <list>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <pthread.h>

cluon::OD4Session od4Speed{112};
cluon::OD4Session od4Turn{112};
cluon::UDPSender UDPsender{"225.0.0.112", 1239};

opendlv::proxy::PedalPositionRequest pedalReq;
opendlv::proxy::GroundSteeringRequest steerReq;

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
void turnCarLeft();
void turnCarRight();
void driveForward(float speed);
void stopCar();
bool exitSoftware();
void printMessage(uint16_t driveCommand);

int32_t main(int32_t argc, char **argv){

    std::cout << "Staring carControl" << std::endl;

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};

    cluon::OD4Session od4Distance{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};   
    cluon::OD4Session od4CarReading{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    

    //Booleans for states of interest
    bool running = true;
    bool stopSignDetected = false;
    bool carRightDetected = false;
    bool carMidDetected = false;
    bool carLeftDetected = false;
    bool noLeftDetected = false;
    bool noRightDetected = false;
    bool atStopSign = false;

    float baseSpeed = std::stof(commandlineArguments["s"]);
    float sonicDistReading{0.0};
    uint16_t detectAngle{static_cast<uint16_t>(std::stoi(commandlineArguments["angle"]))};
    uint16_t driveCommand = 0;


    carObj temp(0,0,0,0,0);
    carObj stopSignLastLocation(0,0,0,0,0);
    std::vector <carObj> snapShot = {temp};
    snapShot.clear();

    
    // recives commands from the Car Command Software
    cluon::UDPReceiver reciverCar("225.0.0.111", 1238,[VERBOSE, &driveCommand]
    (std::string &&data, std::string &&sender,  std::chrono::system_clock::time_point &&/*timepoint*/)
    noexcept {
            if(data == "FORWARD"){
                driveCommand = 1;
            }else if(data == "LEFT"){
                driveCommand = 2;
            }else if(data == "RIGHT"){
                driveCommand = 3;
            }else if(data == "STOP"){
                driveCommand = 4;
            }else if(data == "EXIT"){
                driveCommand = 5;
            }

            if(VERBOSE == 1){
                std::cout << data << " command was sent by: " << sender << std::endl;
            }
        });
    //onDistanceReading recives messages with data from sonar sensor.
    auto onDistanceReading{[VERBOSE, &sonicDistReading](cluon::data::Envelope &&envelope)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); 
                if(senderStamp == 0)
                {
                    sonicDistReading = msg.distance();
                    /*if(VERBOSE == 1){
                    std::cout << "The reading of the Ultrasonic Sensor is: " << sonicDistReading << std::endl;
                    }*/
                }
            }
    };
    //onCarReading is recieving all objects, not only cars.
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
                if(VERBOSE == 1){
                    if(tempCar.getID() == 0){
                        std::cout << "A car was recieved" << std::endl;
                    }else{
                       std::cout << "A sign was recieved" << std::endl; 
                    }
                } 
        }
    };

    od4CarReading.dataTrigger(opendlv::proxy::CarReading::ID(), onCarReading);
    od4Distance.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

    // Main loop where the diffrent actions will be in place
    // State 
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    while(running != 0){      
        //General delay to minimize performance required
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(driveCommand == 5){
            running = exitSoftware();
            if(VERBOSE){
                printMessage(driveCommand);
            }
        }else if(driveCommand == 4){
            stopCar();
            if(VERBOSE){
                printMessage(driveCommand);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }else{
        //We check if there is any object infront of us 
        //We make sure that we're not at the stopsign yet
        if(sonicDistReading > 0.2 && !atStopSign){
            //Check if we have a stopsign in our recieved snapShots
                //Locates where on the x-axis the stopsign is
                if(stopSignLastLocation.getX() > detectAngle){
                    //Based on where the stopsign is on the x-axis we drive forward 
                    //a dynamic amount of "goTimes"
                    int goTimes = (640 - stopSignLastLocation.getX()) / 15;
                    //Iterate through goTimes with a small delay inbetween to have
                    //a more consistent length, to not be affected by momentum.
                    std::cout << "Going forward to stopsign" << std::endl;
                    pedalReq.position(baseSpeed);
                    od4Speed.send(pedalReq);
                    steerReq.groundSteering(-0.04);
                    od4Turn.send(steerReq);
                    std::this_thread::sleep_for(std::chrono::milliseconds(goTimes*50));
                    atStopSign = true;
                    pedalReq.position(0.0);
                    od4Speed.send(pedalReq);
                    steerReq.groundSteering(0.0);
                    od4Turn.send(steerReq);
                    //Set the state that we've arrived at the stopsign. 
            }
            else{
                if(VERBOSE){
                    std::cout << "Standard driving forward" << std::endl;
                }

                //In the "normal" case we simply wait and move forward.
                std::this_thread::sleep_for(std::chrono::milliseconds(10000));
                pedalReq.position(baseSpeed);
                od4Speed.send(pedalReq);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                pedalReq.position(0.0);
                od4Speed.send(pedalReq);
            }
        }
        else if (atStopSign){
            //We wait for a new frame to check if we see any cars, if not, 
            //we can drive. This only works for a t-cross.
            /*
            int que = 0;
            if(firstStopAtSign){
                if(carLeftDetected){
                    que ++;
                }

                if(carMidDetected){
                    que ++;
                }

                if(carRightDetected){
                    que ++;
                }

                firsStopAtSign = false;
            }else if(waitingAtIntersection && que > 0){
                que -= 1;
            }else{

                //all the normal logic but end with the following statement
                firstStopAtSign = true;
                waitingAtIntersection = false;
            }
            */

            std::this_thread::sleep_for(std::chrono::milliseconds(20000));
            // bool godToGo = false;
            size_t counter = 0;
            for(counter = 0; counter < snapShot.size(); counter++){
                if(snapShot[counter].getID() != 0){
                    break;
                }
            }
            if(snapShot.size() == 0 || counter == snapShot.size()){
                std::cout << "Waiting for command to drive" << std::endl;
                if(noRightDetected || noLeftDetected){
                    //Only allowed to go staight
                    if(noRightDetected && noLeftDetected){
                        if(driveCommand == 1){
                            driveForward(baseSpeed);
                            if(VERBOSE){
                                printMessage(driveCommand);
                            }
                        }
                        else if(driveCommand == 2){
                            std::cout << "We're not allowd to go in this direction" << std::endl;
                        }
                        else if(driveCommand == 3){
                            std::cout << "We're not allowd to go in this direction" << std::endl;
                        }
                        
                    }
                    //allowed to go right or straight
                    else if(noLeftDetected){
                        if(driveCommand == 1){
                            driveForward(baseSpeed);
                            if(VERBOSE){
                                printMessage(driveCommand);
                            }
                        }
                        else if(driveCommand == 2){
                            std::cout << "We're not allowd to go in this direction" << std::endl;
                        }

                        else if(driveCommand == 3){
                            turnCarRight();
                            if(VERBOSE){
                                printMessage(driveCommand);
                            }
                        }
                    }
                    //allowed to got left or straigt
                    else if(noRightDetected){
                        if(driveCommand == 1){
                           driveForward(baseSpeed);
                           if(VERBOSE){
                                printMessage(driveCommand);
                            }
                        }
                        else if(driveCommand == 2){
                            turnCarLeft();
                            if(VERBOSE){
                                printMessage(driveCommand);
                            }
                        }
                        else if(driveCommand == 3){
                            std::cout << "We're not allowd to go in this direction" << std::endl;
                        }
                    }
                }
                //allowed to go anywhere
                else{
                    if(driveCommand == 1){
                        driveForward(baseSpeed);
                        if(VERBOSE){
                            printMessage(driveCommand);
                        }
                    }
                    else if(driveCommand == 2){
                        turnCarLeft();
                        if(VERBOSE){
                            printMessage(driveCommand);
                        }
                    }
                    else if(driveCommand == 3){
                        turnCarRight();
                        if(VERBOSE){
                            printMessage(driveCommand);
                        }
                    }
                }
            }
            else{
                std::cout << "Cars found, not going" << std::endl;
                //Do nothing
            }
            // resets the booleans after finishing driving through a intersection
            //atStopSign = false;
            //carLeftDetected = false;
            //carRightDetected = false;
            //carMidDetected = false;
            //noLeftDetected = false;
            //noRightDetected = false;
        }
        //In cases where sonicdistance isn't > .2 nor at the stopsign (almost all cases);
        else{
            pedalReq.position(0.0);
            od4Speed.send(pedalReq);
        }
        //Iterating through the recieved snapShots (objects detected)
        for (size_t i = 0; i < snapShot.size() && !atStopSign; ++i)
        {    
            if(VERBOSE){
                snapShot[i].print();
            }
            //Car DETECTED LOGIC
            if (snapShot[i].getID() == 0){
                //Intersection logic for car to the right
                //Handles states
                if (snapShot[i].getX() > 400){
                    if(VERBOSE){
                        std::cout << "Car to the RIGHT detected:" << std::endl;
                    }
                    carRightDetected = true;
                }
                //Intersection logic for car in the middle
                //Handles states
                else if (snapShot[i].getX() > 100 && snapShot[i].getX() < 200 && snapShot[i].getWidth() < snapShot[i].getHeight()*2 && snapShot[i].getHeight() < 80){
                    if(VERBOSE){
                        std::cout << "Car to the MIDDLE detected" << std::endl;
                    }
                    carMidDetected = true;
                }
                else if(snapShot[i].getX() < 100 && snapShot[i].getWidth() > snapShot[i].getHeight()*2){
                    if(VERBOSE){
                        std::cout << "Car to the LEFT detected" << std::endl;
                    }
                    carLeftDetected = true;
                }
            } 
            //Stopsign state logic
            else if(snapShot[i].getID() == 1){
                stopSignDetected = true;
                if (VERBOSE){
                    std::cout << "Stop sign detected" << std::endl; 
                }
                //Saves latest stopsign seen.
                stopSignLastLocation = snapShot[i];
            }


            //No left detected
            else if (snapShot[i].getID() == 2){
                if(VERBOSE){
                    std::cout << "sign no LEFT detected" << std::endl;
                }
                noLeftDetected = true;
            }


            //No Right detected
            else if (snapShot[i].getID() == 3){
                if(VERBOSE){
                    std::cout << "sign no RIGHT detected" << std::endl;
                }
                noRightDetected = true;
            }
            }
        }
        snapShot.clear();
   }
   
    return 0;
}

void turnCarLeft(){
    pedalReq.position(0.12);
    od4Speed.send(pedalReq);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    steerReq.groundSteering(0.42);
    od4Turn.send(steerReq);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    steerReq.groundSteering(0.0);
    od4Turn.send(steerReq);
    pedalReq.position(0.0);
    od4Speed.send(pedalReq);
}

void turnCarRight(){
    //Turn full on right
    steerReq.groundSteering(-0.40);
    pedalReq.position(0.12);
    od4Turn.send(steerReq);
    od4Speed.send(pedalReq);
    std::this_thread::sleep_for(std::chrono::milliseconds(2700));
    steerReq.groundSteering(0.0);
    pedalReq.position(0.0);
    od4Turn.send(steerReq);
    od4Speed.send(pedalReq);
}

void driveForward(float speed){
    pedalReq.position(speed);
    od4Speed.send(pedalReq);  
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));      
    pedalReq.position(0.0);
    od4Speed.send(pedalReq);
}

void stopCar(){
    pedalReq.position(0.0);
    steerReq.groundSteering(0.0);
    od4Speed.send(pedalReq);
    od4Turn.send(steerReq);
}

bool exitSoftware(){
    pedalReq.position(0.0);
    steerReq.groundSteering(0.0);
    od4Speed.send(pedalReq);
    od4Turn.send(steerReq);
    return false;
}

void printMessage(uint16_t driveCommand){

    switch(driveCommand){
        case 1: UDPsender.send("Car will go FORWARD at the intersection"); break;
        case 2: UDPsender.send("Car will take a LEFT at the intersection");break;
        case 3: UDPsender.send("Car will take a RIGHT at the intersection");break;
        case 4: UDPsender.send("Car will STOP as soon as it can");break;
        case 5: UDPsender.send("Car is shutting of and so is this terminal, GOOD NIGHT"); break;
    }
}