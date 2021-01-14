# Code Readme

There are two aspects to this project:  
- The hurricaneboxESP folder includes the code which should be flashed onto the ESP-32.  
- The hurricanebox folder includes code which should be run on the RaspberryPi. Inside, index.js is the server side node.js script. It will serve index.html to the client. MJPG Streamer should be started in the background on the Pi so it can stream the live video.

