
#ifndef LOGGER_H
#define LOGGER_H

#include "observation.h";

// I2C
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_MCP9808.h>

// SPI
#include <SPI.h>
#include <SD.h>

//Power
//#include <LowPower.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

//OneWire
#include <OneWire.h>

void checkTempSelects();
void logObservation(Observation obs);
Observation getObservation(int i);
void refreshTime();
Observation getInternalTemp();
Observation getExternalTemp();
float readExternalTemp();
String formatObservation(Observation obs);
DateTime getTime();
long getDecimal(float val);
void error(int type);
void flashLED(int pin);


#endif
