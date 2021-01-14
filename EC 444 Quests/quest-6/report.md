# Final Rollup
Authors: Abdelaziz Hussein, Yanni Pang, Sam Krasnoff

Date: 2020-12-10
-----

## Summary
In this quest, we have created a smart "art" safe, featuring :

- remote lock and unlock.
- temperature monotoring to ensure a controlled environment to preserve the art.
- A live stream of the inside of the safe.
- If the safe has been unlocked for 30 minutes, it shuts and locks the door.
- Theft detection.

## Self-Assessment

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Achieves specified functions |  1 |  1     | 
| At least 2 functional actuators | 1  |  1     | 
|At least 3 functional sensors| 1  |  1     | 
| At least 1 camera | 1 |  1     | 
| Demonstrates remote control and data presentation on separate network | 1  | 1     | 
| Has a time based function | 1 |  1     | 
| Multiple participating distributed ESP32s | 1|  1     


### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Achieves specified functions |  |  1     | 
| At least 2 functional actuators |  |  1     | 
|At least 3 functional sensors|  |  1     | 
| At least 1 camera |  |  1     | 
| Demonstrates remote control and data presentation on separate network |  |  1     | 
| Has a time based function |  |  1     | 
| Multiple participating distributed ESP32s |  |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution |  |  5     | 
| Quality of report.md including use of graphics |  |  3     | 
| Quality of code reporting |  |  3     | 
| Quality of video presentation |  |  3     | 


## Solution Design

Remote lock and unlock :
Using an ESP32 Feather a motor connected to the Prongs that lock and unlocks the door was connected. Thus, the esp can run current in one direction or the other to lock/unlock the safe. Two limit switches were used to detect when the door is locked/unlocked to issue the stop instruction to the motors.

Temperature sensing:
A thermistor from quest 2 (skill 13) was  used to monitor the temperature inside the safe and plot the temperature vs time graph and if the temperature is higher/lower than a set limit, an alert message is displayed on the website.

Livestream :

A raspberry pi with a camera is placed inside the safe, along with a torch for illumination, and livestreams the inside of the safe to the host website.

Auto-lock:

Using time interrupts, if the limit switch designated to indicate the status of the door (locked/unlocked) is not engaged for more than 30 minutes (since the safe was open) , then a Geared DC motor,which is attached to a string and a magnet to the door of the safe, is engaged to pull the door in (motor operates for 7 seconds determined by trial and error) Then the motor in the safe door is engaged to lock the door and stops once the limit switch is engaged. 

Theft Detection :

The accelerometer used in Quest 3 (skill 23)  was placed inside the door of the safe and ti monitors the position of the safe. The X,Y,Z coordinates are plotted against time on the host website. If a thief takes the safe from its position, the user will see the data spike, indicating activity.


## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to video demo](). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

## References

-----

