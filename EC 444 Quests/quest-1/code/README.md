# Code Readme

The fishfeeder folder contains all the necessary componets for it to be downloaded locally and flashed onto an ESP-32 Feather. 

## Pin Outs:

Servo:  
sck(pin 18)  
3V, GND  

[Alpha display](https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing?view=all)   
SDA onto SDA  
SCL onto SCL  
3V, GND  


## Code Adaptations:

[Servo example code refrenced](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control)

[i2c display example code](https://github.com/BU-EC444/code-examples/tree/master/i2c-display)

[Timer interrputs example code](https://github.com/BU-EC444/code-examples/tree/master/timer-example)

[FreeRTOS example code](https://www.freertos.org/implementing-a-FreeRTOS-task.html)
