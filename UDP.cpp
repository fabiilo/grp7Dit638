
#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include "cluon-complete.hpp"
#include "messages.hpp"

int32_t main(int32_t argc, char **argv){
    
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if(commandlineArguments["sender"] == 1){
        cluon::UDPSender sender{"225.0.0.111", 1238};
    }else{
        cluon::UDPReceiver reciver("225.0.0.111", 1238, [](std::string &&data, std::string &&sender,  std::chrono::system_clock::time_point &&/*timepoint*/) noexcept {
            cout << data << std::endl;
        });
    }

    if(int32_t i = 0; i < 10; i++){
        sender.send(msg+i);
    }

    return 0;
}
