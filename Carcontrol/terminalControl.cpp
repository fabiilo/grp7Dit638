#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include "cluon-complete.hpp"

int32_t main(int32_t argc, char **argv){

        auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
        cluon::UDPSender sender{"225.0.0.111", 1238};
    
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const int16_t delay{50};
        std::string message = "";
    
        // The front messages the user gets each when the app is first opened!!
        std::cout << "Welcome to GRP 7 communications app for the DIT638 course at GU/Chalmers." << std::endl;
        std::cout << "The CAR takes several commands all listed below, and they are case sensative." << std::endl;
        std::cout << "FORWARD - The car will go foward"<< std::endl;
        std::cout << "LEFT - The car will make a shallow left turn" << std::endl;
        std::cout << "RIGHT - The car will make a sharp right turn" << std::endl;
        std::cout << "STOP - The car will stop and await a new command" << std::endl;
        std::cout << "EXIT- The car will stop and the system will shut down" << std::endl;

        // verbose messages from the Car Control software
       cluon::UDPReceiver reciver("225.0.0.112", 1239,[VERBOSE](std::string &&data, std::string &&sender,  std::chrono::system_clock::time_point &&/*timepoint*/) noexcept {
            if(VERBOSE == 1){
                std::cout << data << " was sent by: " << sender << std::endl;
            } 
        });     

        while(message != "EXIT"){
            
            std::cout << "Command : ";
            std::cin >> message;
            //make
            if(message.size() < 10){
            transform(message.begin(), message.end(), message.begin(), ::toupper);
            sender.send(std::move(message));
            }   
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
    
    return 0;
}
