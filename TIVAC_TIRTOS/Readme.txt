Jenifer Christina
chrisj14@unlv.nevada.edu
5004809012

TIRTOS_Assignment:

Task 01: ADC task
Connect a potentiometer to the ADC pin. Use ADC0 CH4. Also initialized a PWM signal to a LED (PF1). Initial value of the PWM dutycycle is set to 0. Create a timer 0/1/2 HWI for every 1 ms, at 5th instance of HWI the task ADC is performed. Note that the duty cycle of the PWM does not change unless the switch is pressed, even when ADC value changes. 

Task 02: UART display task
At 10th instance of HW1 the task UART displays the current value ADC in the terminal. UART should display the dynamic value of the ADC.

Task 03: Switch Read task
At 15th instance of HWI the task Switch Read is performed to check the status of the SW1/SW2 to update the current value of dutycycle based on the ADC value. 

Task 04: Heartbeat function (PF3)
The heartbeat function is performed throughout the execution of the program. Each task will be executed in order specified every 15 ms. 

