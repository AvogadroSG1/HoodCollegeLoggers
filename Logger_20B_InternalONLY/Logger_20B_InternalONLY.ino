
/*

The MIT License (MIT)

Code Rewrite by Peter O'Connor 2016

Portions of Code
Copyright (c) 2015 Jim Blaney

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#include "observation.h"
#include "logger.h"



/**
 * Data Collection Parameters
 */
//#define NUM_SENSORS                               2
#define SLEEP_INTERVAL                            125
#define LOGGING_FREQUENCY_MINUTES                 10
//#define TIME_ZONE_OFFSET_HOURS                    -4
#define TEMPERATURE_SAMPLE_READINGS               5

/**
 * Pin Configurations
 * 2 and 7 cannot be used.
 */
#define ONE_WIRE_PIN                              3
#define CLOCK_POWER_PIN                           1
#define INT_SENSOR_POWER_PIN                      0
#define POWERON_LED                               5
#define INFO_LED                                  6
#define INT_SENSOR_DETECT_PIN                     8
#define EXT_SENSOR_DETECT_PIN                     9
//#define O_SCK                                     13
//#define O_MISO                                    12
//#define O_MOSI                                    11
//#define O_SS                                      10
//#define I2C_SDA                                   A4
//#define I2C_SCL                                   A5
//#define I2C_INT_POWER                             0
#define mickaylaConstant                            1

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

/**
 * sensor variables
 */
RTC_DS3231 RTC;
Adafruit_MCP9808 tempSensor;
OneWire ds(ONE_WIRE_PIN);

/**
 * temporary variables
 */
//File Variables

String stationId = "";

//Time Variables
String strTime = "";

//Sensor On/Off variables
bool int_sensor = false;
bool ext_sensor = false;


//Testing Variables
bool testing = false;
bool startUp = true;
int count = 0;

DateTime obsTime;
DateTime currentTime;
DateTime afterObsTime;
DateTime nextObsTime;


volatile int cycles_remain;

/**
 * functions
 */
 
void setup() 
{
  
  //Setup the LEDs
  pinMode(POWERON_LED, OUTPUT);
  pinMode(INFO_LED,OUTPUT);

  //Setup Temperature Sensor Detection Pins
  pinMode(INT_SENSOR_DETECT_PIN, INPUT);
  pinMode(EXT_SENSOR_DETECT_PIN, INPUT);

  //Check Current Sensor State for Start
  checkTempSelects();
  delay(5);

//  pinMode(INT_SENSOR_DETECT_PIN, INPUT_PULLUP);
//  pinMode(EXT_SENSOR_DETECT_PIN, INPUT_PULLUP);
 
  delay(5);
  Wire.begin();
  RTC.begin();
  delay(5);
       flashLED(POWERON_LED);

  // Ready the Internal sensor
  if(int_sensor)
  {
    
    if(!tempSensor.begin())
      error(2);
    delay(5);
  }

  pinMode(SS, OUTPUT);
  
  if(SD.begin(SS) == false)
    error(4);

  readID();


  if(!int_sensor && !ext_sensor)
    error(6);


  
  nextObsTime = getTime();
}

void logTime(String time) 
{
  
  File file;

  if(!SD.exists("time.csv"))
  {
    file = SD.open("time.csv", FILE_WRITE);
    file.println(stationId);
    file.close();
  }
  
  if (file = SD.open("time.csv", FILE_WRITE)) 
  {
    flashLED(INFO_LED);
    file.println(time);
    file.close();
  }
  else
  {
    error(5);
  }

}

ISR(WDT_vect)
{
        // not hanging, just waiting
        // reset the watchdog
        wdt_reset();
}

// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution
void configure_wdt(void)
{
  ADCSRA = 0; 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags
                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b0000011; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts

  // reminder of the definitions for the time before firing
  // delay interval patterns:
  //  16 ms:     0b000000
  //  500 ms:    0b000101
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001
 
}

// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int ncycles)
{  
  configure_wdt();
  cycles_remain = ncycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Turn off the ADC while asleep.
  power_adc_disable();
 
  while (cycles_remain > 0){ // while some cycles left, sleep!

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart
 
  // When awake, disable sleep mode
  sleep_disable();
  
  // we have slept one time more
  cycles_remain = cycles_remain - 1;
 
  }
 
  // put everything on again
  power_all_enable();
 
}

void waitForNextObs()
{
  calcNextObs();
  int numSleeps = (nextObsTime.secondstime() - currentTime.secondstime())/(SLEEP_INTERVAL/1000.0) - mickaylaConstant;
  
  sleep(numSleeps);
 
 do
 {
    currentTime = getTime();
    delay(10);
 }while((nextObsTime - currentTime).seconds() > 0);
  
}

void calcNextObs()
{
   currentTime = getTime();
   nextObsTime = currentTime + TimeSpan((60 * (LOGGING_FREQUENCY_MINUTES)) - currentTime.second());
}

void readID()
{
  File file;

  if(!SD.exists("ID.txt"))
    error(1);
    
  if (file = SD.open("ID.txt")) 
  {
    
    while(file.available())
    {
      flashLED(POWERON_LED);
      stationId += (char) file.read();
    }
    file.close();
  }
  else
  {
    error(3);
  }
}

void checkTempSelects()
{
  if(digitalRead(INT_SENSOR_DETECT_PIN) == HIGH)
  {
      int_sensor = true;
      ext_sensor = false;
  }
  else
  {
    int_sensor = 0;
    ext_sensor = digitalRead(EXT_SENSOR_DETECT_PIN) == HIGH;
  }
}

void loop() 
{  
  Wire.begin();
  obsTime = nextObsTime;

  
  if(int_sensor)
  {
    logObservation(getObservation(0));
  }
  
  if(ext_sensor)
  {
    logObservation(getObservation(1));
  }
  
  waitForNextObs();
}

//0 means INT, 1 means ext
Observation getObservation(int i)
{
  refreshTimeString();
  if(i == 0)
  {
    Observation internal_ob = getInternalTemp();
    return internal_ob;
  }
  else 
  {
    Observation external_ob = getExternalTemp();
    return external_ob;
  }
}

Observation getInternalTemp() {

  //Setup Averaging Data Table
  float readings[TEMPERATURE_SAMPLE_READINGS];
  float average;

  //Get the Current Voltage from Voltage Regualtor
  float battery_voltage = getBatteryVoltage();
  
  delay(5);

  average = 0;
  for (int i = 0; i < TEMPERATURE_SAMPLE_READINGS; i++) 
  {
    delay(300); // must wait for the sensor to refresh the temperature value
    readings[i] = tempSensor.readTempC();
    average += readings[i];
  }
  average /= (double) TEMPERATURE_SAMPLE_READINGS; // get the temperature
  Observation obs = { false, battery_voltage, average };
  return obs;
}

Observation getExternalTemp()
{
  
  //Setup Averaging Data Table
  float readings[TEMPERATURE_SAMPLE_READINGS];
  float average;

  //Get the Current Voltage from Voltage Regualtor
  float battery_voltage = getBatteryVoltage();

  average = 0;
  
  for (int i = 0; i < TEMPERATURE_SAMPLE_READINGS; i++) 
  {
    readings[i] = readExternalTemp();
    average += readings[i];

  }

  average /= (double) TEMPERATURE_SAMPLE_READINGS; // get the temperature
  
  Observation obs = { true,battery_voltage, average };
  return obs;
}

void logObservation(Observation obs) 
{
  
  File file;

  if(!SD.exists("data.csv"))
  {
    file = SD.open("data.csv", FILE_WRITE);
    file.println(stationId);
    file.close();
  }
  
  if (file = SD.open("data.csv", FILE_WRITE)) 
  {
    flashLED(INFO_LED);
    file.println(formatObservation(obs));
    file.close();
  }
  else
  {
    error(5);
  }

}

void refreshTimeString()
{

  DateTime now = obsTime;
  strTime = "";
  strTime += String(now.month(), DEC);
  strTime += String('/');
  strTime += String(now.day(), DEC);
  strTime += String('/');
  strTime += String(now.year(), DEC);
  strTime += ", ";
  strTime += String(now.hour(), DEC);
  strTime += String(':');
  strTime += String(now.minute(), DEC);
  strTime += String(':');
  strTime += String(now.second(), DEC);

}


float readExternalTemp(){

byte data[12];
byte addr[8];

if ( !ds.search(addr)) {
//no more sensors on chain, reset search
ds.reset_search();
return -9000;
}

if (OneWire::crc8( addr, 7) != addr[7]) {
error(2);
return -9100;
}

if ( addr[0] != 0x10 && addr[0] != 0x28) {
error(2);
return -1000;
}

ds.reset();
ds.select(addr);
ds.write(0x44,0);

delay(1000);

byte present = ds.reset();
ds.select(addr); 
ds.write(0xBE);


for (int i = 0; i < 9; i++) { 
data[i] = ds.read();
}

ds.reset_search();

byte MSB = data[1];
byte LSB = data[0];

float TRead = ((MSB << 8) | LSB); 
float Temperature = TRead / 16;

return Temperature;

}

String formatObservation(Observation obs) 
{
  String msg = stationId + "," + strTime+ ","+ String(obs.battery_voltage/1000.0) + ",*," 
  + String(obs.internal) + "," + String(obs.temperature) +",";

  msg += String(crc(msg), HEX);

  return msg;
}

DateTime getTime() {
  //digitalWrite(CLOCK_POWER_PIN, HIGH);
  delay(5);
  DateTime now = RTC.now(); 
  //digitalWrite(CLOCK_POWER_PIN, LOW);
  return now; 
}

/**
 * Error Handling --  functions will stop and Logger will continually loop
 * 
 * @param type -- Integer conforming to documented error codes.  See Following Table
 * 
 * -----+------------------------+----------------+---------------+
 * Code |  Description           |   Green   LED  |   Red LED     |
 * -----+------------------------+----------------+---------------|
 * 1    | ID.txt Missing         |       ON       |      OFF      |
 * 2    | ExtTemp Error          |    Flashing    |      OFF      |
 * 3    | ID.txt Couldn't Open   |       OFF      |    Flashing   |
 * 4    | SD Not Init            |    Flashing    |    Flashing   |
 * 5    | CSV Couldn't Open      |       OFF      |      ON       |
 * 6    | Battery Low            |       ON       |      ON       |
 * -----+------------------------+----------------+---------------+
 */
void error(int type)
{
  digitalWrite(POWERON_LED, LOW);
  digitalWrite(INFO_LED,LOW);
  if(testing)
    digitalWrite(13, HIGH);
  while(1==1)
  {
    switch(type)
    {
    case 1:
      digitalWrite(POWERON_LED, HIGH);
    break;
    case 2:
      flashLED(POWERON_LED);
    break;
    case 3:
      flashLED(INFO_LED);
    break;
    case 4:
      flashLED(POWERON_LED);
      flashLED(INFO_LED);
    break;
    case 5:
      digitalWrite(INFO_LED, HIGH);
    break;
    case 6:
      digitalWrite(POWERON_LED, HIGH);
      digitalWrite(INFO_LED,HIGH);
    break;
    default:
    break;
    }
  }
}

void flashLED(int pin)
{
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
    delay(100);
}

