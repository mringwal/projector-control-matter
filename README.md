# Projector Control Matter

## Motivation

...

## Requirements

This project assumes that the projector has two individual LEDs for ON and OFF, as well as a power button. 
The ON LED is active when the projector is turned on, the OFF LED is active when the projector is turned off.
During startup, the ON LED is blinking, while during shutdown, the OFF LED is blinking. 
To turn on, the Power button is pressed once. To turn off, the Power button needs to pressed twice.

The LEDs are connected on one side to VCC and are turned on by connecting the other side to GND.
The power button is connected to GND on one side and the other has a pull-up resistor to VCC.
By pressing the button, the voltage level goes from high to low.

## Implementation

The LEDs are connected to GPIOs that have ADC functionality. The button is connected to another GPIO.

## Control Logic

By observing the LEDs, the four main states can be distinguished. If the projector is off and there's a request to turn it on, the button is pressed for a short time. Similary, when the projector is on and there's a request to turn it off, the button is pressed twice. In any other state, nothing happens.

## Matter

...
