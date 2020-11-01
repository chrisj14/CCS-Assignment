Jenifer Christina
chrisj14@unlv.nevada.edu
5004809012

TIRTOS_Assignment:

Task 01: ADC task (AD0)
Connect the Joystick-Horizontal to the ADC pinA0. If using the MKII EBPA0(DIO23) isconnected to the JS-H pin. 

Task 02: UART display task
at 10th instance of timerCallbackthe task UART displays the cur-rent value ADC in the terminal, 

Task 03: PWM task (DIO7)
Also initialize a PWM signal to the GREEN LED(DIO7). Initial value of the PWM dutycycle is set to 0. Create a timer 0/1/2 timer Callback for every 1 ms, at 5th instance of timer Callback the task ADC is performed. and at 15thinstance of timerCallbackthe update the cur-rent value of dutycycle of the PWM signal based on the ADC value. 

Task 04: Heartbeat function (RED LED/DIO6)
The heartbeat function is performed throughout the execution of the program. Each task will be executed in order specified every 15 ms. 

