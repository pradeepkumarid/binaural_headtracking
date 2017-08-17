# HRTF based binaural rendering with head tracking
###Pradeep Kumar Govindaraju

This a project done for MAT 240D course at University of California, Santa Barbara.

Pre-requisites:

1. To use this source code, you need Allosystem. This can be found at https://github.com/AlloSphere-Research-Group/AlloSystem

2. You need a smartphone that can provide orientation sensor readings (from accelerometer and gyroscope). The simplest way is to use SensorUdp android app by Takashi or write your own app. I have written parser for the data sent using SensorUdp app. If you use your own app, modify the parser accordingly in _getOrientations_ function of Project/MAIN.cpp

3. The smartphone and PC should be on same network. Adjust the port settings in Project/Utilities.h.

4. Run Project/MAIN.cpp through ./run.sh of Allosystem. Check Allosystem examples to find how to compile and run a cpp file.

[Click here for some more details on implementation and screenshot](/Project/Project%20presentation.pdf)
