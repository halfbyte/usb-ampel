/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "requests.h"       /* The custom request numbers we use */

#define LED_PORT_DDR        DDRB
#define LED_PORT_OUTPUT     PORTB
#define RED_BIT            3
#define GREEN_BIT            4

PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};


static uint8_t count = 0;
static volatile uint8_t red = 0;
static volatile uint8_t green = 0;
static volatile uint8_t pulse = 0;
static uint8_t redBuffer = 0;
static uint8_t greenBuffer = 0;
static uint8_t pulseBuffer = 0;
static uint8_t pulseCount = 0;
static uint8_t pulseIncCount = 0;
static int8_t pulseIncDir = 1;

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
        usbRequest_t    *rq = (usbRequest_t *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR)
        {
                switch(rq->bRequest)
                {
                case CUSTOM_RQ_SET_RED:
                        red = rq->wValue.bytes[0];
                        break;
                case CUSTOM_RQ_SET_GREEN:
                        green = rq->wValue.bytes[0];
                        break;
                case CUSTOM_RQ_SET_PULSE:
                        pulse = rq->wValue.bytes[0];
                        break;
                }
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    // proportional to current real frequency
        if(x < targetValue)             // frequency still too low
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; // this is certainly far away from optimum
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}

void usbEventResetReady(void)
{
    cli();  // usbMeasureFrameLength() counts CPU cycles, so disable interrupts.
    calibrateOscillator();
    sei();
    eeprom_write_byte(0, OSCCAL);   // store the calibrated value in EEPROM
}

void update_leds()
{

  if (++count == 0) {
    redBuffer = red;
    greenBuffer = green;
    pulseBuffer = pulse;
    // check if switch on r, g and b
    if (redBuffer > 0) {
      PORTB |= (1 << RED_BIT);
    }
    if (greenBuffer > 0) {
      PORTB |= (1 << GREEN_BIT);
    }
    if (pulseBuffer > 0) {
      if (++pulseIncCount % 2 == 0) {
        pulseCount += pulseIncDir;
        if ((pulseCount == pulseBuffer) || (pulseCount == 0)) {
          pulseIncDir *= -1;
        }
      }
    }
  }
  // if pulsing, ignore value of red and green, just pulse
  if (pulseBuffer > 0) {
    if (count == pulseCount) {
      if (redBuffer > 0) PORTB &= ~(1 << RED_BIT);
      if (greenBuffer > 0) PORTB &= ~(1 << GREEN_BIT);
    }
  } else {
    // check if switch off r, g and b
    if (count == redBuffer) {
      PORTB &= ~(1 << RED_BIT);
    }
    if (count == greenBuffer) {
      PORTB &= ~(1 << GREEN_BIT);
    }
  }
}


int main(void)
{
    uchar   i;

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */

    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    LED_PORT_DDR |= _BV(RED_BIT);   /* make the LED bit an output */
    LED_PORT_DDR |= _BV(GREEN_BIT);   /* make the LED bit an output */

    sei();

    for(;;){                /* main event loop */
        update_leds();
        wdt_reset();
        usbPoll();
    }
    return 0;
}
