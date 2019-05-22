/*
 * Copyright (C) 2019  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>



using namespace cv;
using namespace dnn;
using namespace std;
using namespace cluon;

float confThreshold = 0.5;
float nmsThreshold = 0.4;
int inpWidth = 416;
int inpHeight = 416;
vector<string> classes;
OD4Session od4(112);

// Give the configuration and weight files for the model
String modelConfiguration = "/opt/sources/src/darknet-yolov3.cfg";
String modelWeights = "/opt/sources/src/darknet-yolov3_final.weights";

void postprocess(Mat& frame, const vector<Mat>& out);

vector<String> getOutputsNames(const Net& net);
void loop(string NAME, uint32_t HEIGHT, uint32_t WIDTH);

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.i420 --width=640 --height=480" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};


            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            //cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
                // Load the network
            cout << "staritng thread 1 ...\n";
            thread t1(loop, NAME, HEIGHT, WIDTH);
            usleep(1000000);
            cout << "staritng thread 2 ...\n";
            thread t2(loop, NAME, HEIGHT, WIDTH);
            usleep(1000000);
            cout << "staritng thread 3 ...\n";
            thread t3(loop, NAME, HEIGHT, WIDTH);
            //usleep(1000000);
            //cout << "staritng thread 4 ...\n";
            //thread t4(loop, NAME, HEIGHT, WIDTH);
            t1.join();
            t2.join();
            t3.join();
            //t4.join();
        retCode = 0;
    }
    
    return retCode;
}


void loop(string NAME, uint32_t HEIGHT, uint32_t WIDTH){
    //cout << "New thread started\n";


    Net net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
    //cout << "Loaded network\n";
    // Attach to the shared memory.
    std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
    if (sharedMemory && sharedMemory->valid()) {
        std::clog  << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;
    }
    //cout << "Attached to shared memory done\n";
    Mat temp, frame, blob;
    // Endless loop; end the program by pressing Ctrl-C.
    while (1) {
        sharedMemory->wait();
        sharedMemory->lock();
        {
            Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
            frame = wrapped.clone();
        }
        //frame = temp(Rect(0, 120, WIDTH, 240)).clone();
        sharedMemory->unlock();
        cvtColor(frame, frame, CV_BGRA2BGR);
        blobFromImage(frame, blob, 1/255.0, cvSize(inpWidth, inpHeight), Scalar(0,0,0), true, false);
        net.setInput(blob);
        vector<Mat> outs;
        net.forward(outs, getOutputsNames(net));
        postprocess(frame, outs);
        //cout << "postprocess done\n";
    }


}


// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat& frame, const vector<Mat>& outs)
{
    //cout << "postprocessing initiated\n";
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {

        float* data = (float*)outs[i].data;
        int centerX = 0;
        int centerY = 0;
        int width = 0;
        int height = 0;
        int left = 0;
        int top = 0;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                centerX = (int)(data[0] * frame.cols);
                centerY = (int)(data[1] * frame.rows);
                width = (int)(data[2] * frame.cols);
                height = (int)(data[3] * frame.rows);
                left = centerX - width / 2;
                top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.emplace_back(Rect(left, top, width, height));
            }
        }
    }

    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    int counter = 0;
    for (size_t i = 0; i < indices.size(); ++i)
    {
    uint32_t idx = indices[i];
    Rect box = boxes[idx];
    //int width = box.width;
    uint32_t classID = classIds[idx];
    uint32_t uheight =  box.height;
    uint32_t uwidth = box.width;
    uint32_t boxX = box.x;
    uint32_t boxY = box.y;
    opendlv::proxy::CarReading object;
    object.Xpos(boxX);
    object.Ypos(boxY);
    object.height(uheight);;
    object.width(uwidth);
    object.objID(classID);
    od4.send(object);
    counter++;
    cout << counter << "Object found, Message sent\n" << "ID: "<< classID << "  xpos: " << boxX << "  ypos: " << boxY << "  height: " << uheight << "  width: " << uwidth << "\n";
    }
}

vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
