/* 
 *  GS: 3-12-2016  Aurduino V1.6.6 -- All libs from default installation of IDE
 *  GS: Adapted for AirLCDuino board with rotary encooder and 16x2 LCD  
 *  GS: BASED ON ARDUINO IDE GEIGER COUNTER ver 1.01 produced by RH Electronics http://rhelectronics.net
 *  GS: This sketch is not pin compatible with RH-Electronic's hardware!

  // text below here is per RH Electronics unless otherwise stated by GS: 
	Main Board Features:

	1. High voltage is programmed to 400V. Voltage stabilization IS NOT strict, but when battery is too low it will correct the
	output. With the default pre-set high voltage can be trimmed up to 480V, I have checked with 1G resistor divider. If you need 500V
	or more I recommend to use external voltage multiplier x2 or x3.
	2. My improved counting algorithm: moving average with checking of rapid changes.
	3. GS: Alarm LED -- can be used as trigger for external device 
	4. Sieverts or Rouentgen units for displaying current dose. User selection saved into EE prom for next load, but can be changed
	with key "down" any time.

	5. UART logging with RH Electronics "Radiation Logger" http://www.rhelectronics.net/store/radiation-logger.html
	Other similar logging software is compatible. Please also visit: http://radmon.org

	6. Short LED flash for each nuclear event from tube as well as click sound (can be muted).

	7. Fast bar graph on lcd display. No sense for background radiation, but can be very useful for elevated values. Shows counts
	per second measurement (0 to 30 cps )

	8. Two tact switch buttons. One to switch units, second to log absorbed data to EE prom.

	9. Automatic EE prom logging of absorbed dose once per hour.

	------------------------------------------------------------------------------------------------------------------------------

	Additional hardware required:

	1. GM Tube

	2. USB TTL Arduino Module

	-------------------------------------------------------------------------------------------------------------------------------
	Warranty Disclaimer:
	This Geiger Counter kit is for EDUCATIONAL PURPOSES ONLY.
	You are responsible for your own safety during soldering or using this device!
	Any high voltage injury caused by this counter is responsibility or the buyer only.
	DO NOT BUY IF YOU DON'T KNOW HOW TO CARRY HIGH VOLTAGE DEVICES!
	The designer and distributor of this kit is not responsible for radiation dose readings
	you can see, or not see, on the display. You are fully responsible for you safety and health
	in high radiation area. The counter dose readings cannot be used for making any decisions.
	Software and hardware of the kit provided AS IS, without any warranty for precision radiation measurements.
	When you buy the kit it mean you are agree with the disclaimer!

*/

//----------------- load parameters--------------------//

#define RAD_LOGGER    true          // enable serial CPM logging to computer for "Radiation Logger"
//#define EEPROM_LOG    true          // if true will log absorbed dose to eeprom every 60 minutes. Use only if there is no Logging Shield available!

#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include "Configurator.h"


// Global LCD control class
LiquidCrystal lcd(9, 8, 6, 7, 4, 5); // in 4 bit interface mode
#define LCD_ROWS 2
#define LCD_COLS 16


//-----------------Define global project variables-------------------------//
// Globals for rotary encoder user interface
char EncoderCnt = 0;    // Default startup display
char EncoderPressedCnt = 0;
char EncoderDirection = -1; // So that it decremets to a valid display in case the default display is not currently valid due to sensor lacking
unsigned char ShortPressCnt = 0;
unsigned char LongPressCnt = 0;

//Globals for geiger counter application
unsigned long CurrentBat;                        // voltage of the battery
unsigned long counts =0;                            // counts counter
unsigned int  countsPSecond;                     //  second counter
static unsigned long value[] = {0, 0, 0, 0, 0, 0};      // array variable for cpm moving algorithm
static unsigned long cpm;                               // cpm variable
static unsigned long rapidCpm;                          // rapidly changed cpm
static unsigned long minuteCpm;                         // minute cpm value for absorbed radiation counter
static unsigned long previousValue;                     // previous cpm value

unsigned long previousMillis = 0;                // millis counters
unsigned long previousMillis_bg  = 0;
unsigned long previousMillis_pereferal = 0;
unsigned long previousMillis_hour = 0;
int n = 0;                                       // counter for moving average array
long result;                                     // voltage reading result
static float absorbedDose;                       // absorbed dose
static float dose;                               // radiation dose
static float minuteDose;                         // minute absorbed dose
boolean savedEeprom = false;                     // eeprom flag

boolean event         = false;                   // GM tube event received, lets make flag
//boolean limit         = false;                   // absorbed dose limit flag

//-------------Roentgen/ Sieverts conversion factor----------------//
float factor_Rn = (FACTOR_USV * 100000) / 877;   // convert to Roentgen
float factor_Sv = FACTOR_USV;
float factor_Now = factor_Sv;                    // Sieverts by default
int units;                                       // 0 - Sieverts; 1 - roentgen
//-----------------------------------------------------------------//


/*
  This ISR handles the reading of a quadrature encoder knob and inc or decrements  two global encoder variables
  depending on the direction of the encoder and the state of the button. The encoder functions at 1/4 the maximal possible resolution
  and generally provides one inc/dec per mechanical detent.

  One of the quadrature inputs is used to trigger this interrupt handler, which checks the other quadrature input to decide the direction.
  It is assumed that the direction  quadrature signal is not bouncing while the first phase is causing the initial interrupt as it's signal
  is 90deg opposed.  RC filtering of the contacts is required.

  Three encoder count variables are being modified by this ISR
    1) EncoderCnt increments or dewcrements when the knob is turned without the button being press
    2) EncoderPressCnt increments or decrements when the knob is  turned while the button is also pressed.
       This happens only after a LONG press timeout
    3) EncoderDirection either +1 or -1 depending on the direction the user turned the knob last
*/
void ISR_KnobTurn( void)
{
  if ( digitalRead( Enc_B_PIN ) )
    EncoderDirection = Enc_DIRECTION;
  else
    EncoderDirection = -Enc_DIRECTION;

  if ( digitalRead( Enc_PRESS_PIN ) )
    EncoderCnt += EncoderDirection;
  else
    EncoderPressedCnt += EncoderDirection;

}


/*
  ISR to handle the button press interrupt.

  Two modes of button presses are recognized. A short, momentary press and a long, timing-out press.
  While the hardware de bounced button signal is sampled for up to TIMEOUT time in this ISR no other code is being executed. If the time-out occurs
  a long button press-, otherwise a short button press is registered. Timing has to be done by a software counter since interrupts are disabled
  and function millis() and micros() don't work during this time.
  Software timing is CPU clock dependent and therefore has to be adjusted to the clock frequency.
*/
void ISR_ButtonPress(void)
{
  volatile unsigned long t = 0;
  while ( !digitalRead( Enc_PRESS_PIN ))
  {
    if (t++ > LONGPRESS_TIMEOUT)
    {
      LongPressCnt++;
      return;
    }
  }
  ShortPressCnt++;
}


// interrupt for Pin-change event from trigger pin
ISR(PCINT1_vect)
{
  if( digitalRead( TRIG_PIN ) ) // only count positive  edge 
  {
    // check PCINT1 interrupt flags  if any other pin change interrupts are used in this code
    counts++;                         // increase 10 seconds cpm counter
    countsPSecond++;                  // increase 1/2 second cpm counter for bar graph
    minuteCpm++;                      // increase 60 seconds cpm counter
    event = true;                     // make event flag
  }
}

unsigned long
readVcc()
{
  unsigned long voltage;
  // READ 1.1V INTERNAL REFERENCE VOLTAGE
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  voltage = ADCL;
  voltage |= ADCH << 8;
  voltage = 1126400L / voltage;
  return voltage;
}


void
adjust_PWM( void )
{
  CurrentBat = readVcc();

  if ( CurrentBat > LOW_BAT && CurrentBat < OK_BAT )
  {
    analogWrite(PWM_PIN, PWM);    //correct tube voltage if battery on the middle
    lcd.setCursor(15, 0);
    lcd.write(6);
  }
  else if (CurrentBat >= OK_BAT)
  {
    analogWrite(PWM_PIN, PWM - PWM_ADJ);    //correct tube voltage if battery ok
    lcd.setCursor(15, 0);
    lcd.write((unsigned char) 0);
  }
  else if (CurrentBat <= LOW_BAT)
  {
    analogWrite(PWM_PIN, PWM + PWM_ADJ);    //correct tube voltage if battery low
    lcd.setCursor(15, 0);
    lcd.write(7);
  }


}

///////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------SETUP AREA-----------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // zero some important variables first
  counts = 0;
  countsPSecond = 0;
  minuteCpm = 0;
  minuteDose = 0;
  n = 0;
  cpm = 0;

  // configure atmega IO
  pinMode(ALARM_LED, OUTPUT);          // turn on backlight
  digitalWrite(ALARM_LED, LOW);
  pinMode(LED_PIN, OUTPUT);          // configure led pin as output
  pinMode(BEEPER_PIN, OUTPUT);       // making this pin output for PWM
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BEEPER_PIN, LOW);
  

  // Setup the Encoder pins to be inputs with pullups
  pinMode(Enc_A_PIN, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(Enc_B_PIN, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(Enc_PRESS_PIN, INPUT);// Use external 10K pullup and 100nf to gnd for debounce

  Serial.begin(57600);
  Serial.print(" Geiger counter V1.01\n\r");


  // start LCD display
  lcd.begin(LCD_COLS, LCD_ROWS);
  // create and load custom characters to lcd memory

 // load 8 custom characters in the LCD
  lcd.createChar(0, batfull);   
  lcd.createChar(1, bar_1);
  lcd.createChar(2, bar_2);
  lcd.createChar(3, bar_3);
  lcd.createChar(4, bar_4);
  lcd.createChar(5, bar_5);
  lcd.createChar(6, batmid);  
  lcd.createChar(7, batlow);  

  // extract starting messages from memory and print it on lcd
  lcd.setCursor(0, 0);
  lcd.print("  Air LCD  "); //
  lcd.setCursor(0, 1);

  lcd.print( " Geiger Counter "); //
  blinkLed(3, 50);                                             // say hello!
  delay(1000);

  pinMode(PWM_PIN, OUTPUT);          // making this pin output for PWM
  TCCR2B = TCCR2B & 0b11111000 | 0x2; //set PWM frequency to 4Khz. See datasheet for specifics
  adjust_PWM();

  delay(1000);

  //---------------------read EEPROM---------------------------------//

  byte units_flag = EEPROM.read(FLAG_ADDR);   // check if we have a new atmega chip with blank eeprom
  if (units_flag == 0xFF)
  {
    EEPROM.write(UNITS_ADDR, 0x00);          // and writing zeros instead of 0xFF for new atmega
    EEPROM.write(UNITS_ADDR + 1, 0x00);
    EEPROM.write(ABSOR_ADDR, 0x00);
    EEPROM.write(ABSOR_ADDR + 1, 0x00);
    EEPROM.write(ABSOR_ADDR + 2, 0x00);
    EEPROM.write(ABSOR_ADDR + 3, 0x00);
    EEPROM.write(FLAG_ADDR,  0x00);
  }

  units = EEPROM.read(UNITS_ADDR);


#if (EEPROM_LOG)
  absorbedDose = eepromReadFloat(ABSOR_ADDR);
  if (readButtonUp() == LOW)
  {
    absorbedDose = 0.00;
    eepromWriteFloat(ABSOR_ADDR, absorbedDose);
  }
#endif

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TOTAL DOSE   uSv");   //
  lcd.setCursor(0, 1);
  pFloat(absorbedDose);
 // delay(2000);
  //------------------------------------------------------------------//

  // prepare lcd
 // lcd.clear();

  // Enable the pin change interrupt for PC2 -- trigger signal from tube
  // Note: this enabling of the pinchange interrupt pin has to go hand in hand with the chosen trigger pin above
  pinMode(TRIG_PIN, INPUT);              // set pin INT0 input for capturing GM Tube events
  digitalWrite(TRIG_PIN, HIGH);          // turn on pullup resistors
  PCMSK1 |= 1 << PCINT10 ; // enable PCinterupt10 , aka PC2
  PCICR |= 1 << PCIE1; // enable PCinterupt 1 vector


  attachInterrupt(0, ISR_KnobTurn, FALLING);    // for the rotary encoder knob rotating
  attachInterrupt(1, ISR_ButtonPress, FALLING);    // for the rotary encoder knob push

}

///////////////////////////////////////////////////////////////////////////////////////
//-----------------------------MAIN PROGRAM CYCLE IS HERE----------------------------//
///////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  unsigned long currentMillis = millis();

  //--------------------------------makes beep and led---------------------------------//
  if (event == true)
  { // make led and beep for every tube event
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BEEPER_PIN, HIGH);
    delay( LED_TIME);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BEEPER_PIN, LOW);
    event = false;
  }

  //--------------------------What to do if buttons pressed----------------------------//
  // swith dose units
#ifdef kjhhkjhkjh
  if (readButtonDo() == LOW) {
    units = 1 - units;
    digitalWrite(LIGHT, HIGH);
    if (units == 0)
    {
      lcd.setCursor(0, 0);
      lcdprint_P("Sieverts"); //
    }
    else
    {
      lcd.setCursor(0, 0);
      lcdprint_P("Roentgen"); // "Roentgen"
    }
    EEPROM.write (UNITS_ADDR, units); //save units to eeprom
  }


  if (readButtonUp() == LOW)
  {
    digitalWrite(LIGHT, HIGH);
    float absorbedDose_tmp = eepromReadFloat(ABSOR_ADDR);  // check if eeprom value is smaller than
    // current absorbed dose and write to eeprom
    if (absorbedDose > absorbedDose_tmp)
    { // should safe eerpom cycles
      eepromWriteFloat(ABSOR_ADDR, absorbedDose);
      lcd.setCursor(10, 0);
      lcd.print( "SAVED!"); //
    }
  }
#endif



  //---------------------------What to do every Second ----------------------------------//
  if (currentMillis - previousMillis_bg > 1000)
  {
    previousMillis_bg = currentMillis;

    if (countsPSecond > (ALARM / 60))
    { 
      digitalWrite(ALARM_LED, HIGH);          
    }

    lcdBarGraph(countsPSecond);
    countsPSecond = 0;


  }

  //-------------------------What to do every 10 seconds-------------------------------------//
  if (currentMillis - previousMillis > 10000)
  { // calculating cpm value every 10 seconds
    previousMillis = currentMillis;

    lcd.setCursor(0, 0);
    lcd.write("\xe4");                          // print Mu , use "u" if have problem with lcd symbols table
    lcd.setCursor(0, 1);

    lcd.print("CPM:");
    value[n] = counts;
    previousValue = cpm;

    cpm = value[0] + value[1] + value[2] + value[3] + value[4] + value[5];

    if (n == 5)
      n = 0;
    else
      n++;



    //-------check if cpm level changes rapidly and make new recalculation-------//
    if (previousValue > cpm)
    {
      rapidCpm = previousValue - cpm;
      if (rapidCpm >= 50)
        cpm = counts * 6;

    }

    if (previousValue < cpm)
    {
      rapidCpm = cpm - previousValue;
      if (rapidCpm >= 50)
        cpm = counts * 6;
    }


    // calculate and print radiation dose on lcd
    clearLcd(4, 0, 6);

    if (units == 0)
    {
      factor_Now = factor_Sv;
      lcd.setCursor(1, 0);
      lcd.print(F("Sv:"));
    }
    else
    {
      factor_Now = factor_Rn;
      lcd.setCursor(1, 0);
      lcd.print(F("Rn:"));
    }

    dose = cpm * factor_Now;

    if (dose >= 99.98)
    { // check if dose value are bigger than 99.99 to make the value printable on 5 lcd characters
      int roundDose;
      roundDose = (int) dose;
      lcd.setCursor(4, 0);
      lcd.print(roundDose);
    }
    else
      pFloat(dose);


    // print cpm on lcd
    clearLcd(4, 1, 6);

    lcd.setCursor(4, 1);
    lcd.print(cpm);

    if (cpm > (ALARM / 3))
      digitalWrite(ALARM_LED, HIGH);
    else
      digitalWrite(ALARM_LED, LOW);
      
    // send cpm value to Radiation Logger http://www.rhelectronics.net/store/radiation-logger.html

#if (RAD_LOGGER)
    Serial.print(cpm);  // send cpm data to Radiation Logger
    Serial.write(' ');  // send null character to separate next data
#endif
    counts = 0;            // clear variable for next turn
  }
  //--------------------------------------------------------------------------------------------//

  //------------------------What to do every minute---------------------------------------------//
  if (currentMillis - previousMillis_pereferal > 60000)
  {
    previousMillis_pereferal = currentMillis;

    float minuteDose = (minuteCpm * factor_Sv) / 60;     // count absorbed radiation dose during last minute

    absorbedDose += minuteDose;
    if (absorbedDose > DOSE_LIMIT)
    { // check if you get 1mSv absorbed limit
      absorbedDose = 0.00;
    }


    clearLcd(10, 0, 6);
    lcd.setCursor(10, 0);
    pFloat(absorbedDose);

    adjust_PWM();

    minuteDose = 0.00;
    minuteCpm = 0;                         // reset minute cpm counter
  }


  //--------------------------------What to do every hour-------------------------------------//
  // because eeprom has limited write cycles quantity, use once per hour auto logging

#if (EEPROM_LOG)
  if (currentMillis - previousMillis_hour > 3600000)
  {
    previousMillis_hour = currentMillis;
    eepromWriteFloat(ABSOR_ADDR, absorbedDose);      // log by auto to eeprom once per hour
  }
#endif


}


void lcdBarGraph(int cnts_per_sec)
{  
  // Bar graph displays counts per second -- Max 30 counts per second , i.e. 1800 CMP
  // There are 6 charcter positioins with 5 lines each which provides for a resolution of 30. 
  // amount of bars needed to display the count
  unsigned char fullBlock = (cnts_per_sec / 5);   // divide for full "blocks" of 5 bars
  unsigned char prtlBlock = (cnts_per_sec % 5 );  // calc the remainder of bars

  lcd.setCursor(10, 1);
  
  if (fullBlock > 6)  // check for overflow, only 6 LCD positions are available
  {
    fullBlock = 6;
    prtlBlock = 0;
  }
  for (unsigned int i = 0; i < fullBlock; i++)
  {
    lcd.write((unsigned char) 5);                          // print full blocks
  }
  
  if (prtlBlock )
  {
     lcd.write(prtlBlock);
     prtlBlock =1;
  }
  

  for (int i = (fullBlock + prtlBlock); i <= 6; i++)
  {
    lcd.write(' ');                     // blank spaces to clean up leftover
  }
}

float eepromReadFloat(int address)
{
  union u_tag
  {
    byte b[4];
    float fval;
  } u;
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address + 1);
  u.b[2] = EEPROM.read(address + 2);
  u.b[3] = EEPROM.read(address + 3);
  return u.fval;
}

void eepromWriteFloat(int address, float value)
{
  union u_tag
  {
    byte b[4];
    float fval;
  } u;
  u.fval = value;

  EEPROM.write(address  , u.b[0]);
  EEPROM.write(address + 1, u.b[1]);
  EEPROM.write(address + 2, u.b[2]);
  EEPROM.write(address + 3, u.b[3]);
}


//------------------------------------LED BLINK--------------------------------------//

void blinkLed(int i, int time)
{ // make beeps and blink signals
  int ii;                                            // blink counter
  for (ii = 0; ii < i; ii++)
  {
    digitalWrite(BEEPER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(time);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BEEPER_PIN, LOW);
    delay(time);
  }
}


void pFloat(float dd)
{  
  lcd.print(int(dd));                            // convert it to text before
  lcd.write('.');                                // sending to lcd
  if ((dd - int(dd)) < 0.10) {
    lcd.write('0');
    lcd.print(int(100 * (dd - int(dd))));
  }
  else {
    lcd.print(int(100 * (dd - int(dd))));
  }
}

//--------------------------------clear lcd zone-------------------------------------//
void clearLcd(byte x, byte y, byte zone)
{
  int ii;
  lcd.setCursor (x, y);
  for (ii = 0; ii < zone; ii++) {
    lcd.write(' ');
  }
}




