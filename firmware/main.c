#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>     /* for _delay_ms() */

#include "usbdrv.h"
#include "oddebug.h"

/* ------------------------------------------------------------------------- */

#define NUM_KEYS    12

/* The following function returns an index for the first key pressed. It
 * returns 0 if no key is pressed.
 */
static unsigned char keyPressed(void)
{
  PORTB &= (~_BV(PORTB5));
  PORTB |= _BV(PORTB3);
  PORTC |= _BV(PORTC1);
  _delay_ms(1);
  if(!(PINB & _BV(PINB4))){
    return 1;
  }
  if(!(PINC & _BV(PINC0))){
    return 11;
  }
  if(!(PINC & _BV(PINC2))){
    return 7;
  }
  if(!(PINC & _BV(PINC3))){
    return 4;
  }

  PORTB |= _BV(PORTB5);
  PORTB &= (~_BV(PORTB3));
  PORTC |= _BV(PORTC1);
  _delay_ms(1);
  if(!(PINB & _BV(PINB4))){
    return 2;
  }
  if(!(PINC & _BV(PINC0))){
    return 10;
  }
  if(!(PINC & _BV(PINC2))){
    return 8;
  }
  if(!(PINC & _BV(PINC3))){
    return 5;
  }

  PORTB |= _BV(PORTB5);
  PORTB |= _BV(PORTB3);
  PORTC &= (~_BV(PORTC1));
  _delay_ms(1);
  if(!(PINB & _BV(PINB4))){
    return 3;
  }
  if(!(PINC & _BV(PINC0))){
    return 12;
  }
  if(!(PINC & _BV(PINC2))){
    return 9;
  }
  if(!(PINC & _BV(PINC3))){
    return 6;
  }

  return 0;
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

static unsigned char reportBuffer[2];    /* buffer for HID reports */
static unsigned char idleRate;           /* in 4 ms units */

const PROGMEM char usbHidReportDescriptor[35] = {   /* USB report descriptor */
  0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
  0x09, 0x06,                    // USAGE (Keyboard)
  0xa1, 0x01,                    // COLLECTION (Application)
  0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
  0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
  0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
  0x75, 0x01,                    //   REPORT_SIZE (1)
  0x95, 0x08,                    //   REPORT_COUNT (8)
  0x81, 0x02,                    //   INPUT (Data,Var,Abs)
  0x95, 0x01,                    //   REPORT_COUNT (1)
  0x75, 0x08,                    //   REPORT_SIZE (8)
  0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
  0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
  0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
  0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
  0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

#define KEY_A       4
#define KEY_B       5
#define KEY_C       6
#define KEY_D       7
#define KEY_E       8
#define KEY_F       9
#define KEY_G       10
#define KEY_H       11
#define KEY_I       12
#define KEY_J       13
#define KEY_K       14
#define KEY_L       15
#define KEY_M       16
#define KEY_N       17
#define KEY_O       18
#define KEY_P       19
#define KEY_Q       20
#define KEY_R       21
#define KEY_S       22
#define KEY_T       23
#define KEY_U       24
#define KEY_V       25
#define KEY_W       26
#define KEY_X       27
#define KEY_Y       28
#define KEY_Z       29
#define KEY_1       30
#define KEY_2       31
#define KEY_3       32
#define KEY_4       33
#define KEY_5       34
#define KEY_6       35
#define KEY_7       36
#define KEY_8       37
#define KEY_9       38
#define KEY_0       39

#define KEY_F1      58
#define KEY_F2      59
#define KEY_F3      60
#define KEY_F4      61
#define KEY_F5      62
#define KEY_F6      63
#define KEY_F7      64
#define KEY_F8      65
#define KEY_F9      66
#define KEY_F10     67
#define KEY_F11     68
#define KEY_F12     69

static const unsigned char keyReport[NUM_KEYS + 1][2] PROGMEM = {
  /* none */  {0, 0},                     /* no key pressed */
  /*  1 */    {0, KEY_1},
  /*  2 */    {0, KEY_2},
  /*  3 */    {0, KEY_3},
  /*  4 */    {0, KEY_4},
  /*  5 */    {0, KEY_5},
  /*  6 */    {0, KEY_6},
  /*  7 */    {0, KEY_7},
  /*  8 */    {0, KEY_8},
  /*  9 */    {0, KEY_9},
  /* 10 */    {0, KEY_0},
  /* 11 */    {0, KEY_L},
  /* 12 */    {0, KEY_R},
};

static void buildReport(unsigned char key)
{
  /* This (not so elegant) cast saves us 10 bytes of program memory */
  *((int*)reportBuffer) = pgm_read_word(keyReport[key]);
}

unsigned char usbFunctionSetup(unsigned char data[8])
{
  usbRequest_t    *rq = (void *)data;

  usbMsgPtr = reportBuffer;
  if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
    if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
      /* we only have one report type, so don't look at wValue */
      buildReport(keyPressed());
      return sizeof(reportBuffer);
    }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
      usbMsgPtr = &idleRate;
      return 1;
    }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
      idleRate = rq->wValue.bytes[1];
    }
  }else{
    /* no vendor specific requests implemented */
  }
  return 0;
}

/* ------------------------------------------------------------------------- */

int	main(void)
{
  int i;

  uchar   key, lastKey = 0, keyDidChange = 0;
  uchar   idleCounter = 0;

  /* pull-up input pin */
  PORTB |= _BV(PORTB4);
  PORTC |= (_BV(PORTC0) | _BV(PORTC2) | _BV(PORTC3));
  /* set output select-pin */
  DDRB |= (_BV(DDB3) | _BV(DDB5));
  DDRC |= _BV(DDC1);
  /* set output buzzer-pin */
  DDRD |= _BV(DDD7);

  DDRD |= _BV(DDD1); /* 0000 0010 bin: remove USB reset condition */
  /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
  TCCR0B = 5;      /* timer 0 prescaler: 1024 */
  
  wdt_enable(WDTO_2S);
  odDebugInit();
  usbInit();
  sei();
  for(;;){	/* main event loop */
    wdt_reset();
    usbPoll();
    key = keyPressed();
    if(lastKey != key){
      lastKey = key;
      keyDidChange = 1;
    }
    if(TIFR0 & (1<<TOV0)){   /* 22 ms timer */
      TIFR0 = 1<<TOV0;
      if(idleRate != 0){
        if(idleCounter > 4){
          idleCounter -= 5;   /* 22 ms in units of 4 ms */
        }else{
          idleCounter = idleRate;
          keyDidChange = 1;
        }
      }
    }
    if(keyDidChange && usbInterruptIsReady()){
      /* buzzer */
      for(i = 0; i < 500; i++){
        PORTD |= _BV(PORTD7);
        _delay_us(100);
        PORTD &= ~_BV(PORTD7);
        _delay_us(100);
      }
      keyDidChange = 0;
      /* use last key and not current key status in order to avoid lost
         changes in key status. */
      buildReport(lastKey);
      usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
    }
  }

  return 0;
}

/* ------------------------------------------------------------------------- */
