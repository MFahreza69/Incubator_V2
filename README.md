# Incubator_V2 #
## Xinfant ##
Code for main PCB incubator (atMega64)


## fastWrite ##
Code for pcb display Incubator (atMega64)


## SkinTemp_Isolation/Mikro_Sensor ##
at tiny1616 code for receiving 2,2kOhm skin sensor data and send them to main microcontroler

## Temperature Control Including ##
| Control Mode |        Temperature Range      |
|--------------|-------------------------------|
| Airway Temperature      |         27.0 - 38.5 °C (adjustable)        |
| Skin Temperature         |  34.5 °C - 35 °C (adjustable) |
| Humidity       |  50 - 90% RH        |


## Alarm Failure Including ##
|Error            | Description                       |
|-----------------|-----------------------------------|
| Probe Failure   | Sensor probe failure (disconnected or cutted)|
| Over Temperature| Temperature control increase reach over 37.5 °C|
| Temperature Deviation | After short time temperature reach steady state condition and temperature control increase or decrease 1.5 °C from set point, error will triggered|
| Power Failure   | Input voltage 220VAC/50-60hz not detected|
| Fan Failure     | fan connector disconnected or cutted| 

