# Hurricane Box
Authors: Abdelaziz Hussein, Yanni Pang, Sam Krasnoff

Date: 2020-22-10
-----

## Summary
Creating a ‘Hurricane Box’ that would be suitable for recording data experienced over an episodic weather event.


### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
|Measures acceleration, temperature, and battery level| 1 |  1     | 
| Displays real-time data (temperature, vibration, battery level) at remote client via portal using separate IP network | 1 |  1     | 
| Controls LED on box from remote client
via portal.  | 1 |  1     | 
| Sources web cam video into remote
client. | 1 |  1     | 
| ESP32 and Rpi are
connected wirelessly to (or as) router;
ESP32 sensor data are delivered to local
node server (on local laptop or Rpi) | 1 |  1     | 
| Demo delivered at scheduled time and
report submitted in team folder with all
required components| 1 |  1     | 
| Investigative question response | 1 |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 5 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 3 |  3     | 
| Quality of video presentation | 3 |  3     | 


## Solution Design
In this quest, a hurricane box was made using a RaspberryPi, ESP-32 Feather, camera, accelerometer, thermistor and an LED. The ESP-32 is used to collect data from the thermistor to measure temperature, accelerometer to measure how stong the wind is by seeing how much the box moves, and controls an LED which can be toggled to indicate an emergency. Lastly the camera streams a feed to a site for monitoring. The data is sent from the ESP-32 to the Pi which is hosting a website displaying the data, live video as well as  button to toggle the LED.


## Storyboard
<center><img src="./images/Screenshot (414).png" width="100%" /></center>  
<center> </center>

## Investigative Question
In order to limit power usage, we would have a very low polling rate for the accelerometer until it polled values higher than reasonable. After reading high values, the polling rate would increase to give accurate readings.


## Supporting Artifacts
- [Link to video demo](). Not to exceed 120s  
Screenshot:
![image](https://github.com/BU-EC444/Team10-Hussein-Krasnoff-Pang/blob/master/quest-3/images/Screenshot%202020-10-24%20at%209.00.17%20AM.png)


## Modules, Tools, Source Used Including Attribution
- [Thermistor example code](https://github.com/espressif/espidf/tree/39f090a4f1dee4e325f8109d880bf3627034d839/examples/peripherals/adc)

- [Raspberry PI setup](https://www.raspberrypi.org/downloads/)

- [ESP-32 wifi setup](https://github.com/espressif/espidf/tree/master/examples/wifi/getting_started/station)
- [Socket.io](https://socket.io/get-started/chat/)
- [dgram sockets](https://nodejs.org/api/dgram.html)
- [express](https://expressjs.com/en/starter/installing.html)

[Video Presentation](https://drive.google.com/file/d/1vBy649xiRRbpnhTl6iRLtQcSdC-Xk213/view?usp=sharing)
-----

