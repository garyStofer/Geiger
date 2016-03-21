// battery Ni-MH settings are here. 
#define LOW_BAT 4550     // Battery is NOT OK 
#define OK_BAT 4850     // Battery is OK

// define constants, do not modify pins
#define LED_TIME 5       // for led/buzzer to be ON; 5-20ms for most LED's. If you also use active 5V buzzer, then find the optimal time for the clicker sound. 
#define LED_PIN A3           // led and buzzer on Analog Pin 3
#define BEEPER_PIN 12       //aka: MISO,PB4,  Active buzzer
#define ALARM_LED  10        // backlight on Digital Pin 3
#define ALARM 600        // 600 cpm default alarm level


// PWM starting points, you need to calibrate with 1Giga ohm resistor the tube starting voltage!
// connect 5.00V power supply and find optimal PWM_FUL value
// then add 5 points to PWM_MID and 10 points to PWM_LOW
// start with 90!


#define PWM 90
#define PWM_ADJ 5
#define PWN_FREQ 8    // PWM frequency setup
#define PWM_PIN 11    // aka: MOSI, PB3,OC2, NOTE: pin assigment is not regotiable, Hardware and setup code rely on it beeing OC2


#define TRIG_PIN 16  // aka: PC2, A2  // Using Pin change interrupt

// The pin definitions are as per obfuscated Arduino pin defines -- see aka for ATMEL pin names as found on the MEGA328P spec sheet
#define Enc_A_PIN 2     // This generates the interrupt, providing the click  aka PD2 (Int0)
#define Enc_B_PIN 14    // This is providing the direction aka PC0,A0
#define Enc_PRESS_PIN 3  // aka PD3 ((Int1)
#define Enc_DIRECTION (-1)  // polarity of encoder , either -1 or +1 depending on the phase relation of the two signals in regard of the turn direction
#define LONGPRESS_TIMEOUT 100000 // Note: not in milli seconds since this is used in a SW while loop timer

// GM Tube setting
#define FACTOR_USV 0.0057    // Sieverts Conversion Factor, known value from tube datasheet 
/*
SBM-20   0.0057
SBM-19   0.0021
SI-29BG  0.0082
SI-180G  0.0031
J305     0.0081
SBT11-A  0.0031
SBT-9    0.0117
*/

#define DOSE_LIMIT 999.98       // Maximum allowed absorbed value in uSV
                             // be sure it can be written to 6 lcd characters "000.00"
                             

// EEPROM addresses
#define UNITS_ADDR    0     // int 2 bytes
#define FLAG_ADDR     10    // byte 
#define ABSOR_ADDR    2     // unsigned long 4 bytes

 

// battery indicators
byte batfull[8] = {
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b00000
};

// battery indicators
byte batmid[8] = {
  0b00000,
  0b01110,
  0b10001,
  0b10011,
  0b10111,
  0b11111,
  0b11111,
  0b00000
};

byte batlow[8] = {
  0b00000,
  0b01110,
  0b10001,
  0b10011,
  0b10101,
  0b11001,
  0b11111,
  0b00000
};


// 1 bar
byte bar_1[8] = {
  B00000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B00000
};

// 2 bars
byte bar_2[8] = {
  B00000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B00000
};

// 3 bars
byte bar_3[8] = {
  B00000,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B00000
};

// 4 bars
byte bar_4[8] = {
  B00000,
  B11110,
  B11110,
  B11110,
  B11110,
  B11110,
  B11110,
  B00000
};

// 5 bars
byte bar_5[8] = {
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B00000
};


