1. Donwload the weights from, to src folder:
```
https://drive.google.com/file/d/1Fhq_XYKfyXufize-8wP9ChaxexKopT2a/view?usp=sharing
```

2. To build for run on kiwi-car: 
```bash
sudo docker build -t myapp -f Dockerfile.armhf .
```

3. To build for run on Desktop:
```bash
sudo docker build -t myapp -f Dockerfile.amd64 .
```

4. To run the built docker on dekstop:
```bash
sudo docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY myapp --cid=253 --name=img.argb --width=640 --height=480 --verbose
```