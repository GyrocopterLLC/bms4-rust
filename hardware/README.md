# bms4_f051 Hardware Design Files
Designed in Kicad 7.0  
<https://www.kicad.org/>  

## Board Notes
Based on STM32F051K8 microcontroller.  
Uses 2x dual LM2904 op-amps to measure the voltage of the lithium ion batteries. Stepping down the voltage is necessary to avoid damaging the microcontroller's input pins which are limited to 3.3V.
Balancing is performed by 33 ohm shunt resistors, one per cell. The are turned on by the microcontroller through external FETs (AO3400A).
Communication is done through an isolation circuit (ISO7221D). This is used to be able to communicate to other bms4_f051 boards in a series battery configuration. In series, the boards will not share grounds. Connecting one board's ground to another board's ground would create a dangerous short circuit through the LiIon batteries! The boards are designed to be daisy-chained in communication, with the bottom-most (or top-most, your choice) board connected to an external controller, for example an ebike motor controller.

## Board Images
![Board Top](/img/board_top.png "Board Top")
![Board Bottom](/img/board_bottom.png "Board Bottom")
![Schematic](/img/schematic.png "Schematic")
