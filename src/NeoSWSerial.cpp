//
// NeoSWSerial
// Copyright (C) 2015-2017, SlashDevin
//
// https://github.com/SlashDevin/NeoSWSerial/
//
// NeoSWSerial is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// NeoSWSerial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details:
//
//     <http://www.gnu.org/licenses/>.
//
// Methods
// -------
//
// begin(baudRate) - initialization, optionally set baudrate, and then enable RX
// listen() - enables RX interrupts, allowing data to enter the RX buffer
// ignore() - disables RX interrupts
// setBaudRate(baudRate) - selects the baud rate (9600, 19200, 31250, 38400)
//                             - any other value is ignored
// available() - returns the number of characters in the RX buffer
// read() - returns a single character from the buffer
// write(s) - transmits a string
//
// print() is supported
//=============================================================================

#include <NeoSWSerial.h>

// Default baud rate is 9600
static const uint8_t TICKS_PER_ONE_BIT_WHEN_9600 = (uint8_t) 26; // How many 4us ticks (x) has to elapse to transmit one bit during 9 600 Baud
                              // Transfer one bit (no matter if status or data bit) when there is 9 600 bit_changes/s takes 1/9600 s
                              // x * 4us = 1/9600
                              // x = 26.04 -> ~26

static const uint8_t TICKS_PER_ONE_BIT_WHEN_31250 = 8; // How many 4us ticks (x) has to elapse to transmit one bit during 31 250 Baud
                              // 31250 baud bit width in units of 4us

static const uint8_t BITS_TRANSMITTED_PER_ONE_TICK_WHEN_31250_MULTIPLIED_BY_1024 = 128; // How many bits is transmitted after 4us (tick) during 31 250 Baud multiplied by 1024 to get non decimal number
                     // 31250 bps * 0.000004 s (= bits per 4 us) * 2^10 
static const uint8_t BITS_TRANSMITTED_PER_ONE_TICK_WHEN_3840_MULTIPLIED_BY_1024 = 157;
                     // 1s/(38400 bits) * (1 tick)/(4 us) * 2^10  "multiplier"

#if F_CPU == 16000000L // F_CPU tells compiler what CPU speed to use for various delays and functions
  #define TCNTX TCNT0 // Assign Timer Counter register
  #define PCI_FLAG_REGISTER PCIFR // Assign Pin Change interrupt flag register
#elif F_CPU == 8000000L
  #if defined(__AVR_ATtiny25__) | \
      defined(__AVR_ATtiny45__) | \
      defined(__AVR_ATtiny85__) 
    #define TCNTX TCNT1
    #define PCI_FLAG_REGISTER GIFR
  #else
    #define TCNTX TCNT2
    #define PCI_FLAG_REGISTER PCIFR
  #endif
// MARK: TODO Nano Matter uses 39 000 000 Hz -> possible to use ARDUINO_NANO_MATTER flag (defined in boards.txt and platform.txt)
#elif F_CPU == 39000000L
  #define TCNTX TCNT0 // Assign Timer Counter register, just tried, not sure if it is TCNT0
#endif

static NeoSWSerial *listener = (NeoSWSerial *) NULL;

static uint8_t oneTxBitTicks;
static uint8_t rxWindowWidth;
static uint8_t bitsPerTick_Q10; // multiplier!

static const uint8_t WAITING_FOR_START_BIT = 0xFF;
static uint8_t rxState;  // 0: got start bit; >0: bits rcvd
static uint8_t prev_t0;  // previous RX transition: timer0 time stamp (4us)
static uint8_t rxMask;   // bit mask for building received character
static uint8_t rxValue;  // character being built

static const uint8_t RX_BUFFER_SIZE = 64;  // power of 2 for optimal speed
static uint8_t rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxHead;   // buffer pointer input
static uint8_t rxTail;   // buffer pointer output

static          uint8_t rxBitMask, txBitMask; // port bit masks
static volatile uint8_t *txPort;  // port register

//#define DEBUG_NEOSWSERIAL
#ifdef DEBUG_NEOSWSERIAL

  uint8_t bitTransitionTimes[16];
  uint8_t bitTransitions;
  static const uint8_t MAX_DIFF_RX_BITS = 8;
  struct rbd_t { uint8_t loop_bits; uint8_t mul_bits; uint16_t prod; };
  rbd_t diffRXbits[MAX_DIFF_RX_BITS];
  uint8_t diffRXbitsCount;

  uint16_t availCompletions;
  uint16_t rxStartCompletions;
  uint8_t rxStartCompletionBits[128];
  uint16_t checkRxCompletions;
  uint16_t polledPCI;
  uint16_t polledPCICompletions;
  uint16_t stopBitCompletions;

  #define DBG_NSS_COUNT(v) { v++; }
  #define DBG_NSS_COUNT_RESET(v) { v = 0; }
  #define DBG_NSS_ARRAY(a,i,v) \
    { if (i < sizeof(a)/sizeof(a[0])) \
      a[ i++ ] = v; }

#else

  #define DBG_NSS_COUNT(v)
  #define DBG_NSS_COUNT_RESET(v)
  #define DBG_NSS_ARRAY(a,i,v)

#endif

// MACROS MISSING FOR ARDUINO MATTER
#define _BV(bit) (1 << (bit))

static uint16_t mul8x8to16(uint8_t x, uint8_t y)
{return x*y;}

//..........................................

/**
 * @brief Return number of bits transmitted/received in given period dt.
 * Number of elapsed ticks multiplied by bits per tick
 * gives number of transmitted bits. Because "bits per tick" was previously
 * multiplied by 1024 to get non decimal number, the result is now divided by 1024 and returned.
 * 
 * @param dt 
 * @return uint16_t 
 */
static uint16_t bitTimes( uint8_t dt ) // dt = data transition time
{
  return mul8x8to16( dt + rxWindowWidth, bitsPerTick_Q10 ) >> 10;

} // bitTimes

//----------------------------------------------------------------------------

/**
 * @brief initialize, set baudrate, listen
 * 
 * @param baudRate 
 */
void NeoSWSerial::begin(uint16_t baudRate)
{
  setBaudRate( baudRate );
  listen();
}

//----------------------------------------------------------------------------

/**
 * @brief enable RX interrupts
 * 
 * @param baudRate 
 */
void NeoSWSerial::listen()
{
  if (listener)
    listener->ignore();

  pinMode(rxPin, INPUT);
  rxBitMask = digitalPinToBitMask( rxPin );
  rxPort    = portInputRegister( digitalPinToPort( rxPin ) ); // returns an input port register of the specified port => PIND, PINB, PINC..., this register contains input values

  txBitMask = digitalPinToBitMask( txPin );
  txPort    = portOutputRegister( digitalPinToPort( txPin ) );
  if (txPort)
    *txPort  |= txBitMask;   // high = idle
  pinMode(txPin, OUTPUT);

  if (F_CPU == 8000000L) {
    // Have to use timer 2 for an 8 MHz system.
    #if defined(__AVR_ATtiny25__) | \
        defined(__AVR_ATtiny45__) | \
        defined(__AVR_ATtiny85__) 
      TCCR1  = 0x06;  // divide by 32
    #else
      TCCR2A = 0x00;
      TCCR2B = 0x03;  // divide by 32
    #endif
  }

  volatile uint8_t *pcmsk = digitalPinToPCMSK(rxPin); // PCMSKx -> controls which pin from specific port can produce interrupt
  if (pcmsk) { // if interrupt on some pin is enabled
    rxState  = WAITING_FOR_START_BIT;
    rxHead   = rxTail = 0;    // no characters in buffer
    flush();

    // Set up timings based on baud rate
    
    switch (_baudRate) {
      case 9600:
        oneTxBitTicks      = TICKS_PER_ONE_BIT_WHEN_9600          ; // Bit width expressed in number of 4us intervals
        bitsPerTick_Q10 = BITS_TRANSMITTED_PER_ONE_TICK_WHEN_3840_MULTIPLIED_BY_1024 >> 2;
        rxWindowWidth   = 10; // MARK: TODO Not fully understand...
        break;
      case 31250:
        if (F_CPU > 12000000L) {
          oneTxBitTicks = TICKS_PER_ONE_BIT_WHEN_31250;
          bitsPerTick_Q10 = BITS_TRANSMITTED_PER_ONE_TICK_WHEN_31250_MULTIPLIED_BY_1024;
          rxWindowWidth = 5;
          break;
        } // else use 19200
      case 38400:
        if (F_CPU > 12000000L) {
          oneTxBitTicks      = TICKS_PER_ONE_BIT_WHEN_9600    >> 2;
          bitsPerTick_Q10 = BITS_TRANSMITTED_PER_ONE_TICK_WHEN_3840_MULTIPLIED_BY_1024   ;
          rxWindowWidth   = 4;
          break;
        } // else use 19200
      case 19200:
        oneTxBitTicks      = TICKS_PER_ONE_BIT_WHEN_9600      >> 1;
        bitsPerTick_Q10 = BITS_TRANSMITTED_PER_ONE_TICK_WHEN_3840_MULTIPLIED_BY_1024 >> 1;
        rxWindowWidth   = 6;
        break;
    }

    // Enable the pin change interrupts

    uint8_t prevSREG = SREG;
    cli(); // Clear interrupt global enable flag bit (disable all interrupts).
    {
      *pcmsk                    |= _BV(digitalPinToPCMSKbit(rxPin)); // Enable interrupts from specific pin (rxPin)
      *digitalPinToPCICR(rxPin) |= _BV(digitalPinToPCICRbit(rxPin)); // Enable interrupts for bunch of pins (port) containing rxPin
      listener = this;
    }
    SREG = prevSREG;
  }

} // listen

//----------------------------------------------------------------------------
/**
 * @brief disable RX interrupts
 * 
 * @param baudRate 
 */
void NeoSWSerial::ignore()
{
  // About interrupts, example: When a logic change on any PCINT23..16 pin triggers an interrupt request, PCIF2 becomes set (one). 
  // If the I-bit (bit 7) in SREG and the PCIE2 bit in PCICR are set (one), the MCU will jump to the corresponding interrupt vector.
  // The flag is cleared when the interrupt routine is executed.
  // External Interrupts are limited to only a couple pins, while the Pin Change interrupts (used here) can occur on all input pins.
  // Pin Change Interrupts share an ISR between all the pins on a port (port B, C, and D).  And anytime a pin changes on that port, 
  // it calls the port’s ISR which must then decide which pin caused the interrupt.
  if (listener) {
    volatile uint8_t *pcmsk = digitalPinToPCMSK(rxPin); // PCMSKx -> controls which pin from specific port can produce interrupt

    // SREG is the processor Status REGister. Global Interrupt Enable bit, carry and zero bit and so on... Save it.
    uint8_t prevSREG = SREG;
    cli(); // disable global interrupts

    // Disable interrupts on RX pin
    {
      listener = (NeoSWSerial *) NULL;
      if (pcmsk) { // if pcmsk != 0, something is enabled
        // PCICR = Pin Change Interrupt Control Register - controls which port is open to interrupts from pins
        *digitalPinToPCICR(rxPin) &= ~_BV(digitalPinToPCICRbit(rxPin)); // Disable interrupts for bunch of pins (port) containing rxPin
        *pcmsk &= ~_BV(digitalPinToPCMSKbit(rxPin)); // Disable interrupts from specific pin (rxPin)
      }
    }
    SREG = prevSREG; // Restore Status register
  }

} // ignore

//----------------------------------------------------------------------------
/**
 * @brief Sets baud rate to 9600 [default], 19200, 38400 and restarts.
 * 
 * @param baudRate 
 */
void NeoSWSerial::setBaudRate(uint16_t baudRate)
{
  // If requested baud rate is supported and F_CPU is big enough -> set baud rate and restart
  if ((
        ( baudRate ==  9600) ||
        ( baudRate == 19200) ||
        ((baudRate == 31250) && (F_CPU == 16000000L)) ||
        ((baudRate == 38400) && (F_CPU == 16000000L))
       )
           &&
      (_baudRate != baudRate)) {

    _baudRate = baudRate;

    if (this == listener)
      listen();
  }
} // setBaudRate

//----------------------------------------------------------------------------

/**
 * @brief Checks if some bit is available in RX buffer. If interrupt routine is 
 * attached, return always false.
 */
int NeoSWSerial::available()
{
  uint8_t avail = ((rxHead - rxTail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE); // (0 + x) % x = 0, (x + x) % x = 0, otherwise 1
  
  if (avail == 0) { // MARK: TODO Probably solving some edge case?
    cli(); //  Disable all interrupts - Clear interrupt global enable flag
      if (checkRxTime()) {
        avail = 1;
        DBG_NSS_COUNT(availCompletions);
      }
    sei(); // Set interrupt global enable flag bit (re-enable interrupts after being disabled).
  }

  return avail;

} // available

//----------------------------------------------------------------------------
/**
 * @brief Returns oldest read character (FIFO)
 */
int NeoSWSerial::read()
{
  if (rxHead == rxTail) return -1;
  uint8_t c = rxBuffer[rxTail];
  rxTail = (rxTail + 1) % RX_BUFFER_SIZE;

  return c;

} // read

//----------------------------------------------------------------------------
/**
 * @brief Assign function called whenever character is received. 
 * The registered procedure will be called from the ISR whenever a character is received. 
 * After attaching the function, the received character will not be stored in the rx_buffer, and it will not be returned from read().
 * Any characters that were received and buffered before attachInterrupt was called remain in rx_buffer,
 * and could be retrieved by calling read(). If attachInterrupt is never called,
 * or it is passed a NULL procedure, the normal buffering occurs, and all received characters
 * must be obtained by calling read().
 * 
 * @param fn 
 */
void NeoSWSerial::attachInterrupt( isr_t fn )
{
  uint8_t oldSREG = SREG; // save status register
  cli(); // disable global interrupts
    _isr = fn;
  SREG = oldSREG; // Restore status register

} // attachInterrupt

//----------------------------------------------------------------------------
/**
 * @brief Clear internal variables for next transmission after start bit detection.
 */
void NeoSWSerial::startChar()
{
  rxState = 0;     // got a start bit
  rxMask  = 0x01;  // bit mask, lsb first
  rxValue = 0x00;  // RX character to be, a blank slate

  DBG_NSS_COUNT_RESET(bitTransitions);

} // startChar

//----------------------------------------------------------------------------
/**
 * @brief Called when pin change the value. 
 * 
 * @param rxPort 
 */
void NeoSWSerial::rxISR( uint8_t rxPort )
{
  uint8_t t0 = TCNTX;            // time of data transition (plus ISR latency)
  uint8_t rd_value  = rxPort & rxBitMask; // read RX data level

  if (rxState == WAITING_FOR_START_BIT) {

    // If it looks like a start bit then initialize;
    //   otherwise ignore the rising edge and exit.

    if (rd_value != 0){
      return;   // it's high so not a start bit, exit
    } else {
      startChar();
    }
    

  } else {  // data bit or stop bit (probably) received

    DBG_NSS_ARRAY(bitTransitionTimes, bitTransitions, (t0-prev_t0));

    // Determine how many bit periods have elapsed since the last transition.
    // It is necessary because for example chain of many zeros dont produce any transition
    // -> no call of ISR, but bits arrived! 

    uint16_t rxBitsReceived  = bitTimes( t0-prev_t0 ); // number of received bits from the previous ISR call
    uint8_t  bitsLeft        = 9 - rxState; // ignores stop bit
    bool     nextCharStarted = (rxBitsReceived > bitsLeft);

    if (nextCharStarted)
      DBG_NSS_ARRAY(rxStartCompletionBits,rxStartCompletions,(10*rxBitsReceived + bitsLeft));

    uint8_t  bitsThisFrame   =  nextCharStarted ? bitsLeft : rxBitsReceived;

    rxState += bitsThisFrame;

    // Set all those bits

    if (rd_value == 0) {
      // back fill previous bits with 1's
      while (bitsThisFrame-- > 0) {
        rxValue |= rxMask;
        rxMask   = rxMask << 1;
      }
      rxMask = rxMask << 1;
    } else { // rd_value==1
      // previous bits were 0's so only this bit is a 1.
      rxMask   = rxMask << (bitsThisFrame-1);
      rxValue |= rxMask;
    }

    // If 8th bit or stop bit then the character is complete.

    if (rxState > 7) {
      rxChar( rxValue );

      if ((rd_value == 1) || !nextCharStarted) {
        rxState = WAITING_FOR_START_BIT;
        // DISABLE STOP BIT TIMER

      } else {
        // The last char ended with 1's, so this 0 is actually
        //   the start bit of the next character.

        startChar();
      }
    }
  }

  prev_t0 = t0;  // remember time stamp

} // rxISR

//----------------------------------------------------------------------------
/**
 * @brief TODO
 */
bool NeoSWSerial::checkRxTime()
{
  if (rxState != WAITING_FOR_START_BIT) {

    uint8_t rd_value  = *rxPort & rxBitMask;

    if (rd_value) {
      // Ended on a 1, see if it has been too long
      uint8_t  t0        = TCNTX; // save current timer value = now
      uint16_t rxBitsReceived    = bitTimes( t0-prev_t0 ); // how many bits should be already received, prev_t0 is set in every rxISR
      uint8_t  bitsLeft  = 9 - rxState; // 9 - bits_received
      bool     completed = (rxBitsReceived > bitsLeft); // MARK: TODO Dont understand...

      if (completed) {
        DBG_NSS_COUNT(checkRxCompletions);

        while (bitsLeft-- > 0) { // create character from the received bits, LSB came first
          rxValue |= rxMask;
          rxMask   = rxMask << 1;
        }

        rxState = WAITING_FOR_START_BIT;
        rxChar( rxValue ); // process received character = save to buffer or call routine
        if (!_isr) // if interrupt routine is not attached...
          return true;
      }
    }
  }
  return false;

} // checkRxTime

//----------------------------------------------------------------------------
/**
 * @brief Called when character successfully received. Forwards character to 
 * attached interrupt rutine (if exists) or saves it to the rxBuffer 
 * 
 * @param c 
 */
void NeoSWSerial::rxChar( uint8_t c )
{
  if (listener) {
    if (listener->_isr) // if interrupt rutine is attached...
      listener->_isr( c ); // call it with character
    else { // otherwise save character to the buffer
      uint8_t index = (rxHead+1) % RX_BUFFER_SIZE;
      if (index != rxTail) {
        rxBuffer[rxHead] = c;
        rxHead = index;
      }
    }
  }

} // rxChar

//----------------------------------------------------------------------------

#ifdef NEOSWSERIAL_EXTERNAL_PCINT

  // Client code must call NeoSWSerial::rxISR(PINB) in PCINT handler

#else

  // Must define all of the vectors even though only one is used.

  // This handy PCINT code for different boards is based on PinChangeInterrupt.*
  // from the excellent Cosa project: http://github.com/mikaelpatel/Cosa

  #define PCINT_ISR(vec,pin)		\
  extern "C" { \
  ISR(PCINT ## vec ## _vect)		\
  {								              \
    NeoSWSerial::rxISR(pin);	  \
  } }

  #if defined(__AVR_ATtiny261__) | \
      defined(__AVR_ATtiny461__) | \
      defined(__AVR_ATtiny861__)

  ISR(PCINT0_vect)
  {
    if (GIFR & _BV(INTF0)) {
      NeoSWSerial::rxISR(PINA);
    } else {
      NeoSWSerial::rxISR(PINB);
    }
  }

  #elif defined(__AVR_ATtiny25__) | \
        defined(__AVR_ATtiny45__) | \
        defined(__AVR_ATtiny85__) 

  PCINT_ISR(0, PINB);

  #elif defined(__AVR_ATtiny24__) | \
        defined(__AVR_ATtiny44__) | \
        defined(__AVR_ATtiny84__) 

  PCINT_ISR(0, PINA);
  PCINT_ISR(1, PINB);

  #elif defined(__AVR_ATmega328P__)

  PCINT_ISR(0, PINB);
  PCINT_ISR(1, PINC);
  PCINT_ISR(2, PIND);

  #elif defined(__AVR_ATmega32U4__)

  PCINT_ISR(0, PINB);

  #elif defined(__AVR_AT90USB1286__)

  PCINT_ISR(0, PINB);

  #elif defined(__AVR_ATmega2560__)

  PCINT_ISR(0, PINB);
  PCINT_ISR(1, PINJ);
  PCINT_ISR(2, PINK);

  #elif defined(__AVR_ATmega1281__)

  PCINT_ISR(0, PINB);
  // PCINT8 on PE0 not supported.  Other 7 are on PJ0..6
  PCINT_ISR(1, PINJ);
  PCINT_ISR(2, PINK);

  #elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)

  PCINT_ISR(0, PINA);
  PCINT_ISR(1, PINB);
  PCINT_ISR(2, PINC);
  PCINT_ISR(3, PIND);

  #elif defined(__AVR_ATmega2560RFR2__)

  PCINT_ISR(0, PINB);
  PCINT_ISR(1, PINE);

  #else
    #error MCU not supported by NeoSWSerial!
  #endif

#endif

//-----------------------------------------------------------------------------
// Instead of using a TX buffer and interrupt
// service, the transmit function is a simple timer0 based delay loop.
//
// Interrupts are disabled while the character is being transmitted and
// re-enabled after each character.

size_t NeoSWSerial::write(uint8_t txChar)
{
  if (!txPort)
    return 0;

  uint8_t width;         // ticks for one bit
  uint8_t txBit  = 0;    // first bit is start bit
  uint8_t b      = 0;    // start bit is low
  uint8_t PCIbit = bit(digitalPinToPCICRbit(rxPin));

  uint8_t prevSREG = SREG;
  cli();        // send the character with interrupts disabled

    uint8_t t0 = TCNTX; // start time

    // TODO: This would benefit from an early break after 
    //    the last 0 data bit.  Then we could wait for the
    //    remaining 1 data bits and stop bit with interrupts 
    //    re-enabled.

    while (txBit++ < 9) {   // repeat for start bit + 8 data bits
      if (b)      // if bit is set
        *txPort |= txBitMask;     //   set TX line high
      else
        *txPort &= ~txBitMask;    //   else set TX line low

      width = oneTxBitTicks;
      if ((F_CPU == 16000000L) &&
          (width == TICKS_PER_ONE_BIT_WHEN_9600/4) &&
          (txBit & 0x01)) {
        // The width is 6.5 ticks, so add a tick every other bit
        width++;  
      }

      // Hold the line for the bit duration

      while ((uint8_t)(TCNTX - t0) < width) {
        // Receive interrupt pending?
        if (PCI_FLAG_REGISTER & PCIbit) {
          PCI_FLAG_REGISTER |= PCIbit;   // clear it because...
          rxISR( *rxPort );  // ... this handles it
          DBG_NSS_COUNT(polledPCI);
        } else if (checkRxTime()) {
          DBG_NSS_COUNT(polledPCICompletions);
        }
      }
      t0    += width;         // advance start time
      b      = txChar & 0x01; // get next bit in the character to send
      txChar = txChar >> 1;   // shift character to expose the following bit
                   // Q: would a signed >> pull in a 1?
    }

  *txPort |= txBitMask;   // stop bit is high
  SREG = prevSREG;        // interrupts on for stop bit
  while ((uint8_t)(TCNTX - t0) < width) {
    if (checkRxTime())
      DBG_NSS_COUNT(stopBitCompletions);
  }

  return 1;               // 1 character sent

} // write
