#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <algorithm>
#include <list>
#include "cluon-complete.hpp"
#include "messages.hpp"

int32_t main(int32_t argc, char **argv){

cluon::OD4Session od4{112};

opendlv::proxy::CarReading car;

// code som läger in värderna i car
car.objID("STRONK");
car.height(50);
car.Xpos(2);
car.Ypos(50);

od4.send(car);

}