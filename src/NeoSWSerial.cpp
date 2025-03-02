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
#include <NanoMatterPatches.h>

#define ARDUINO_NANO_MATTER // MARK: Remove later

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
// Nano Matter uses 39 000 000 Hz, possible to use F_CPU or ARDUINO_NANO_MATTER flag (defined in boards.txt and platform.txt)
#elif F_CPU == 39000000L
  #define TCNTX TCNT0 // MARK: TODO Assign Timer Counter register, just tried, not sure if it is TCNT0
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

static          uint16_t rxBitMask, txBitMask; // port bit masks
static volatile uint16_t *txPort;  // port register

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

  #if defined(ARDUINO_NANO_MATTER)
  // MARK: Different implementation for Nano Matter
  #else
  volatile uint8_t *pcmsk = digitalPinToPCMSK(rxPin); // PCMSKx -> controls which pin from specific port can produce interrupt
  #endif

  if (pcmsk) { // if interrupt on some pin is enabled
    rxState  = WAITING_FOR_START_BIT;
    rxHead   = rxTail = 0;    // no characters in buffer
    flush();

    // Set up timings based on baud rate
    
    switch (_baudRate) {
      case 9600:
        oneTxBitTicks      = TICKS_PER_ONE_BIT_WHEN_9600          ; // Bit width expressed in number of 4us intervals
        bitsPerTick_Q10 = BITS_TRANSMITTED_PER_ONE_TICK_WHEN_3840_MULTIPLIED_BY_1024 >> 2;
        rxWindowWidth   = 10; // MARK: TODO Not fully understand... Probably receive window is shorter than bit duration to read it in right moment?
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

    // Enable the pin change interrupts in block protected against interrupts
    #if defined(ARDUINO_NANO_MATTER)
    // MARK: Different implementation for Nano Matter
    #else
    uint8_t prevSREG = SREG;
    cli(); // Clear interrupt global enable flag bit (disable all interrupts).
    {
      *pcmsk                    |= _BV(digitalPinToPCMSKbit(rxPin)); // Enable interrupts from specific pin (rxPin)
      *digitalPinToPCICR(rxPin) |= _BV(digitalPinToPCICRbit(rxPin)); // Enable interrupts for bunch of pins (port) containing rxPin
      listener = this;
    }
    SREG = prevSREG;
    #endif 
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

    #if defined(ARDUINO_NANO_MATTER)
    // MARK: Different implementation for Nano Matter
    #else
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
    #endif 
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
 * @brief Checks if some new char is available in RX buffer. If interrupt routine is 
 * attached, return always false.
 */
int NeoSWSerial::available()
{
  uint8_t avail = ((rxHead - rxTail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE); // (0 + x) % x = 0, (x + x) % x = 0, otherwise != 0
  
  // Even when there is not new char saved in the buffer, all bits could arrive,
  // but because all of them were ones, it didnt trigger saving mechanism -> check this edge case.
  if (avail == 0) { 
    #if defined(ARDUINO_NANO_MATTER)
    // MARK: Different implementation for Nano Matter
    #else
    cli(); //  Disable all interrupts - Clear interrupt global enable flag
      if (checkRxTime()) {
        avail = 1;
        DBG_NSS_COUNT(availCompletions);
      }
    sei(); // Set interrupt global enable flag bit (re-enable interrupts after being disabled).
    #endif 
    
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
  #if defined(ARDUINO_NANO_MATTER)
  // MARK: Different implementation for Nano Matter
  #else
  uint8_t oldSREG = SREG; // save status register
  cli(); // disable global interrupts
  _isr = fn;
  SREG = oldSREG; // Restore status register
  #endif 

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
 * @brief Called when pin change the value, so we received one or more (chained)
 * bits. According type and position of bit(s) gradually creates
 * comming character (8 bits, rxValue) and on the end calls rxChar().
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
    // It is necessary because for example chain of many zeros/ones dont produce any transition
    // -> no call of ISR, but bits arrived! 

    uint16_t rxBitsReceived  = bitTimes( t0-prev_t0 ); // number of arrived bits from the previous ISR call
    uint8_t  bitsLeft        = 9 - rxState; // ignores stop bit
    bool     nextCharStarted = (rxBitsReceived > bitsLeft); // Did I receive more than one frame of bits?

    if (nextCharStarted)
      DBG_NSS_ARRAY(rxStartCompletionBits,rxStartCompletions,(10*rxBitsReceived + bitsLeft));

    uint8_t  bitsThisFrame   =  nextCharStarted ? bitsLeft : rxBitsReceived; // Number of received bits for current frame without bits of the next frame

    rxState += bitsThisFrame; // Update number of received bits in this ISR

    // Update rxValue by received bits

    if (rd_value == 0) { // Before transition to zero, there was a chain of ones
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
        // the start bit of the next character.

        startChar();
      }
    }
  }

  prev_t0 = t0;  // remember time stamp

} // rxISR

//----------------------------------------------------------------------------
/**
 * @brief Called when there is a suspicious that frame was already received,
 * but due to the stable value (all following bits were in 1) on pin
 * it did not trigger ISR. In that case method creates rxValue automatically
 * saves it to buffer, call user routine  and returns true.
 */
bool NeoSWSerial::checkRxTime()
{
  if (rxState != WAITING_FOR_START_BIT) {

    uint8_t rd_value  = *rxPort & rxBitMask;

    if (rd_value) {
      // Start bit came and than all bits had value 1. 
      // Check whether I should already have all frame -> time of frame elapsed.
      uint8_t  t0        = TCNTX; // save current timer value = now
      uint16_t rxBitsReceived    = bitTimes( t0-prev_t0 ); // // number of arrived bits from the previous ISR call, prev_t0 is set in every rxISR
      uint8_t  bitsLeft  = 9 - rxState; // = 9 - bits_processed
      bool     completed = (rxBitsReceived > bitsLeft); // Did I already receive all bits of current frame OR more?

      if (completed) { // I already have all bits -> process them!
        DBG_NSS_COUNT(checkRxCompletions);

        while (bitsLeft-- > 0) { // create character from the received bits (chain of ones), LSB came first
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
 * the attached interrupt routine (if exists) or saves it to the rxBuffer.
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

  // Each PCINTx-bit selects whether pin change interrupt is enabled on the corresponding I/O pin.
  // If PCINTx-bit is set and the PCIE2 bit in PCICR is set, pin change interrupt is enabled.

  // extern "C" means that function name is put into symbol table under the same name.
  // (C++ usually adds some extensions to function names to make overloaded functions unique) 
  // So some client C code can link (use) this function using a C compatible header file,
  // that contains just the declaration of my function under the same name.
  //
  // '##' operator in define concatenate tokens:
  // #define ArgArg(x, y)   x##y
  // ArgArg(lady, bug) -> expanded to: "ladybug"


  // Macro to subscribe handler (rxISR) to the selected interrupt vector ISR(PCINTx_vect)
  #define PCINT_ISR(vec,pin) extern "C" { ISR(PCINT ## vec ## _vect) { NeoSWSerial::rxISR(pin);	} }

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

  PCINT_ISR(0, PINB); // subscribe rxISR to PCINT0_vect vector and tell rxISR that interrupt possibly come from input buffer of PORT B
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


  #elif defined(ARDUINO_NANO_MATTER)

  // Only ports PA and PB are able to produce interrupts in low power modes (EM2, EM3).
  // All ports support interrupts in other modes.

  // Arduino pins mapping:
  // PB -> A0, A1, A2, A6, A7, CLK
  // PB -> SWCLK, SWDIO, SWV, UART_TX, UART_RX, A4, A5, MISO, MOSI


  // Každý interrupt má enable bit v registru GPIO_IEN. Pokud přijde, nastaví se bit v registru GPIO_IF. 
  // Každý interrupt má k sobě přiřazeny 4 piny na každém portu, ze kterých si může vybrat. 
  // Nastavením bitu EXTIPSELx (External Interrupt Port Selection) v registru GPIO_EXTIPSELL vyberu port, který bude podporovat interrupty a tím zvolím i 4 piny.
  // Mezi názvem interruptu a čísly pinů je souvislost. Interrupt EXTI0 může být použit s piny 0,1,2,3. Stejně tak EXTI(1,2,3).
  // Interrupt EXTI4 - EXTI7 souvisí s piny 4,5,6,7 atd. 
  // Abych vybral konkrétní pin pro interrupt, musím do registru GPIO_EXTIPINSELL (External Interrupt Pin Selection) zapsat offset který vypočtu jako:
  // offset = pin_number - base
  // base = 0 pro EXTI0-EXTI3
  //      = 4 pro EXTI4-EXTI7
  //      = 4 * int(interrupt_number/4)
  
  // Příklad: Namapovat EXTI5 na pin 7 znamená base 4, offset 3

  // Na jakou hranu bude pin reagovat se nastaví v registrech GPIO_EXTIRISE[n] and GPIO_EXTIFALL[n] -> možné reagovat na obojí.

  // Nastavením EXT[n] sudého bitu v registru GPIO_IEN odstartuji sudou interrupt line a naopak.

  // Pro zapnutí filtrace vstupního signálu zapiš do MODEx field(s) v GPIO_Px_MODEL registru.

  // Po interruptu je třeba smazat interrupt flag v GPIO_IF registru, dělá se to zápisem do GPIO_IF_CLR. Přímý zápis nemá vliv.

  // Více o interruptech v EFR32xG24 Wireless SoC Reference Manual, stránka 37 -> Interrupt operation.

  // How to enable interrupt on pin in ATMEL:
  // 1. Set I-bit (bit 7) in SREG (AVR Status Register)  -> I-bit = Enable Global Interrupt. The I-bit is cleared by hardware after an interrupt
  //    has occurred and restored by RETI instruction on the end. The I-bit can also be set and cleared by the application with the SEI and CLI instructions.
  // 2. Set PCIEx bit in PCICR (Pin Change Interrupt Control Register) -> Enable interrupts for bunch of pins (port), but still not enough to fire...
  // 3. Set PCINTy bit in PCMSKx register -> controls which pin (y) from enabled bunch of pins (port) can produce interrupt


  // How to enable interrupt on pin in ARM(SiliconLabs):
  // 1. Set EXTIPSELx (External Interrupt Port Selection) bit in the GPIO_EXTIPSELL register -> enable 4 pins on specific port to generate interrupts
  // EXTIPSEL0 = Port select for external interrupt 0 (EXTI0) and EXTI0 could be used with a pins 0,1,2,3
  // EXTIPSEL1 = Port select for external interrupt 1 (EXTI1) and EXTI1 could be used with a pins 0,1,2,3
  // .......
  // EXTIPSEL3 = Port select for external interrupt 1 (EXTI1) and EXTI1 could be used with a pins 0,1,2,3
  // EXTIPSEL4 = Port select for external interrupt 4 (EXTI4) and EXTI4 could be used with a pins 4,5,6,7
  // ....
  // EXTIPSEL7 = Port select for external interrupt 7 (EXTI7) and EXTI7 could be used with a pins 4,5,6,7
  // ....
  // 2. Select specific pin by writing OFFSET to GPIO_EXTIPINSELL (External Interrupt Pin Selection) register where:
  // offset = pin_number - BASE
  //    BASE = 0 for EXTI0-EXTI3
  //         = 4 pro EXTI4-EXTI7
  //         = 4 * int(interrupt_number/4)
  // 3. Enable interrupt by setting corresponding bit in GPIO_IEN register



  // MARK: TODO assign interrupt vector to function for arduino Matter

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

  #if defined(ARDUINO_NANO_MATTER)
  // MARK: Different implementation for Nano Matter
  #else

  uint8_t PCIbit = bit(digitalPinToPCICRbit(rxPin)); // Pin change interrupt control register

  uint8_t prevSREG = SREG;
  cli();        // send the character with interrupts disabled

    uint8_t t0 = TCNTX; // start time

    // TODO: This would benefit from an early break after 
    // the last 0 data bit. Then we could wait for the
    // remaining 1 data bits and stop bit with interrupts 
    // re-enabled.

    while (txBit++ < 9) {   // repeat for start bit + 8 data bits
      if (b)      // if bit is set
        *txPort |= txBitMask;     //   set TX line high
      else
        *txPort &= ~txBitMask;    //   else set TX line low

      width = oneTxBitTicks;
      if ((F_CPU == 16000000L) &&
          (width == TICKS_PER_ONE_BIT_WHEN_9600/4) &&
          (txBit & 0x01)) {
        // The width is TICKS_PER_ONE_BIT_WHEN_9600/4 = 6.5 ticks, so add a tick every other bit
        width++;  
      }

      // Hold the line for the bit duration and in the meantime service the the possible incoming bits
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

  // Send stop bit
  *txPort |= txBitMask;   // stop bit is high
  SREG = prevSREG;        // interrupts on for stop bit
  while ((uint8_t)(TCNTX - t0) < width) {
    if (checkRxTime())
      DBG_NSS_COUNT(stopBitCompletions);
  }

  #endif 

  return 1;               // 1 character sent

} // write
