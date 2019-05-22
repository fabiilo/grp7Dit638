1. Add your own user into the video group to allow access to the webcam (needed to be done only once):

sudo groupmod -aG video $USER

2. To enable the video encoding, you need to build the h264 video encoder based on the libx264 library:


docker build -t x264encoder -f Dockerfile.amd64 https://github.com/chalmers-revere/opendlv-video-x264-encoder.git

3. Next, open a new terminal and run h264-decoder-viewer.yml from template/example_video on GitLab (cf. our previous post) so that you have a web frontend to which you can connect with your web browser (cf. README.md on the GitLab repository):


docker-compose -f h264-decoder-viewer.yml up

4. Point your web browser to http://localhost:8081

5. Next, you need to open a new terminal to start the microservice that will open your webcam (on VirtualBox, you might need to enable routing the webcam from your host into the virtual machine); you can verify that you have a webcam available on Ubuntu by running:

ls -l /dev/video0

This should return a file entry representing your web cam.

Now, you need to enable access to your graphical user interface (only necessary when you want to check whether the OpenDLV microservice that opens the webcam can actually capture frames by displaying them directly on your screen; necessary each time you logout from your Ubuntu session):


xhost +

Next, you can start the microservice to open the webcam:

docker run --rm -ti --init --ipc=host -e DISPLAY=$DISPLAY --device /dev/video0 -v /tmp:/tmp chalmersrevere/opendlv-device-camera-opencv-multi:v0.0.11 --camera=/dev/video0 --width=640 --height=480 --freq=20 --verbose
6. Once you see a video feed from your webcam, you need to open a new terminal and start the video encoder to feed that video into your web browser:

docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp x264encoder --cid=112 --name=video0.i420 --width=640 --height=480 --verbose
Now, you should see the same video as displayed in a separate window on your Linux inside your web browser. From a software architecture point of view, this microservices-based application is similar to the setup that you run on your Kiwi car with capturing frames, encoding frames using a video compressor, and sending them to a web browser. Hence, you can use this setup to experiment a bit with your computer vision algorithms.