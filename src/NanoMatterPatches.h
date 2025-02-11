#ifndef NANOMATTERPATCHES_H
#define NANOMATTERPATCHES_H

#include "Arduino.h"

// returns bit mask for n-th bit
#define _BV(bit) (1 << (bit))

// Pin mapping - maps Arduino pin names (printed on board) to Silabs internal PinNames
// Example: write A0 -> get PB0 (=66). Possible to use getPortPinFromArduinoPin -> (= (66 - 50) % 16) -> 0 (A0 is really first (0) bit on port PB)
#define D0 PA4 // Tx1 - SPI1 SDO
#define D1 PA5 // Rx1 - SPI1 SDI - WU
#define D2 PA3 // SPI1 SCK
#define D3 PC6 // SPI1 SS
#define D4 PC7 // SDA1 - WU
#define D5 PC8 // SCL1
#define D6 PC9 //
#define D7 PD2 // WU
#define D8 PD3 //
#define D9 PD4 //
#define D10 PD5 // SPI SS
#define D11 PA9 // SPI SDO
#define D12 PA8 // SPI SDI
#define D13 PB4 // SPI SCK

#define D14 PB0 // DAC0
#define A0 PB0 // DAC0

#define D15 PB2 // DAC2
#define A1 PB2 // DAC2

#define D16 PB5 //
#define A2 PB5 //

#define D17 PC0 // WU
#define A3  PC0 // WU

#define D18 PA6 // SDA
#define A4  PA6 // SDA

#define D19 PA7 // 19 - SCL 
#define A5  PA7 // SCL

#define D20 PB1 // DAC1 - WU
#define A6  PB1 // DAC1 - WU

#define D21 PB3 // DAC3 - WU
#define A7  PB3 // DAC3 - WU

#define LED_R PC1 // LED R
#define LED_G PC2 // LED G
#define LED B PC3 // LED B
#define BUTTON PA0 // Button
#define SERIAL_TX PC4 // Serial Tx
#define SERIAL_RX PC5 // Serial Rx - WU

// MARK: TODO - there are now 3 files which do the pin mapping, this one,
// SiliconLabsSrc\cores\silabs\pinDefinitions.h
// and SiliconLabsSrc\variants\nano_matter\pins_arduino.h
// How this mapping hell works??


// returns port bit number for Arduino pin (D0, D5, A0 or PB0, PC1... or 0, 1, 2...)
#define arduinoPinToPortBit(arduino_pin) (pinToPinName(arduino_pin) - PIN_NAME_MIN) % 16

// Data In Register (GPIO_Px_DIN) can be used to read the level of each pin in the port 
// (bit n in the register is connected to pin n on the port). When configured as an output,
// the value of the Data Out Register (GPIO_Px_DOUT) will be driven to the pin.
// returns the bitmask of a specified Arduino pin (D0, D5, A0 or PB0, PC1... or 0, 1, 2...)
#define digitalPinToBitMask(arduino_pin)  (_BV(arduinoPinToPortBit(arduino_pin)))

// returns the port (1(PB), 3(PD)...) of a specified pin (D0, D5, A0 or PB0, PC1... or 0, 1, 2...)
#define digitalPinToPort(arduino_pin)  getSilabsPortFromArduinoPin(pinToPinName(arduino_pin))

// returns an input port register of the specified port (1(PB), 3(PD)...), this register contains input values
#define portInputRegister(port_number)  ((volatile uint32_t *) &GPIO->P[port_number].DIN) 

// returns an output port register of the specified port (1(PB), 3(PD)...), this register contains output values
#define portOutputRegister(port_number) ((volatile uint32_t *) &GPIO->P[port_number].DOUT) 

#endif // NANOMATTERPATCHES_H





