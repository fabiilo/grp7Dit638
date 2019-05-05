1. Donwload the weights from:
```
https://drive.google.com/file/d/1Fhq_XYKfyXufize-8wP9ChaxexKopT2a/view?usp=sharing
```

2. To run kiwi-car detection, check "object_detection_yolo.py" line 33-34 for paths to cfg and weights.

2. Have an image in the folder

3. To run kiwi-car detection on image
```
python3 object_detection_yolo.py --image=img-00001copy2.jpg
```

4. To run kiwi-car detection on video
```
python3 object_detection_yolo.py --video=somevideopath
```