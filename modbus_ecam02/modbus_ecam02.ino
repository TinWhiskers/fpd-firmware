// ******************************************************************************************
// TITLE: 
// https://github.com/TinWhiskers/fpd-design/wiki/ECAM02---End-Effector-PCB-w-USB-pass-through
// ******************************************************************************************

#define MODBUS 1 //Comment this line out to use normal serial console instead of MODBUS!

//TODO: 
// * Implement rolling averaging for analog reading.  
//     Put analog read function in an interrupt.  
//     Make a fixed ring array, and put readings into the ring array
//     When querying the end effector, return the averaged reading instead of ther instantaneous one.
// * Add thread to handle stepping in background.

#include "EEPROM.h"
#include <Wire.h> // I2C
#include <avr/eeprom.h>
#include <Adafruit_MCP4725.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

// ******************************************************************************************
// PIN DEFINES, MACROS, ETC
// ******************************************************************************************
#define PUMP     28  // Vacuum pump (active HIGH)
#define SOL1     3   // Solenoid #1: PnP vacuum to nozzle tip (normally closed)
#define SOL2     5   // Solenoid #2: PnP vacuum 'dump' to atmosphere (normally closed)
#define SOL3     6   // Solenoid #3: Solder paste pressure to paste nozzle (normally closed)
#define SOL4     9   // Solder paste pressure 'dump' to atmosphere (normally closed)
#define LED_EN   7   // Enable for AP3032 controller
#define VACSENSE A0  // MPXV5100DP sensor, analog input
#define STEP     15  // DRV8825 step pin
#define DIR      16  // DRV8825 direction pin
#define ENABLE   17  // DRV8825 enable
#define DTR      4   // RS-485 transmit enable

#define 

// Useful macros
#define CW       1   // For stepper motor
#define CCW      0   // For stepper motor

#define ANALOG_BITWEIGHT (5.0 / 1024.0)

#define DRIVER_MICROSTEPS      32.0
#define MOTOR_STEPS_PER_DEG    1.8
#define STEP_PER_DEGREE 88.888


// ******************************************************************************************
// GLOBAL VERIALBES
// ******************************************************************************************
Adafruit_MCP4725 dac;

// Global registers that can go across the MODBUS tubes
//-------------------------------------------------------------------------------------------
modbusRegister  *reg_vacsense;


float   reg_vacsense  = 0;     // 0-5 volts, float
float   reg_thetaAngle= 0;     // 0-360 degrees.  -1 to disable the motor.
uint8_t reg_solenoid1 = 0;     // 0=off.  255=fully on.
uint8_t reg_solenoid2 = 0;     // 0=off.  255=fully on.
uint8_t reg_solenoid3 = 0;     // 0=off.  255=fully on.
uint8_t reg_solenoid4 = 0;     // 0=off.  255=fully on.
uint8_t reg_pump      = 0;     // 0=off.  255=fully on.
uint8_t reg_ringlight = 0;     // 0=off.  255=fully on.
//-------------------------------------------------------------------------------------------
// MODBUS globals
modbusDevice regBank; // Setup the brewtrollers register bank.  All of the data accumulated will be stored here
modbusSlave slave;    // Create the modbus slave protocol handler
//-------------------------------------------------------------------------------------------

long rawStepCount = 0; // Raw step count for the stepper motor.

// Fast I/O stuff for stepping at a moderately fast rate, without digitalWrite()
uint8_t stepBitmask = digitalPinToBitMask(STEP);
uint8_t stepPort    = digitalPinToPort(STEP);
volatile uint8_t *stepOut;


// ******************************************************************************************
// SETUP
// ******************************************************************************************
void setup()
{
  //Configure pin modes and set default states
  pinMode(PUMP,    OUTPUT);  digitalWrite(PUMP,   LOW);
  pinMode(SOL1,    OUTPUT);  digitalWrite(SOL1,   LOW);
  pinMode(SOL2,    OUTPUT);  digitalWrite(SOL2,   LOW);
  pinMode(SOL3,    OUTPUT);  digitalWrite(SOL3,   LOW);
  pinMode(SOL4,    OUTPUT);  digitalWrite(SOL4,   LOW);
  pinMode(LED_EN,  OUTPUT);  digitalWrite(LED_EN, LOW);
  pinMode(STEP,    OUTPUT);  digitalWrite(STEP,   LOW);
  pinMode(DIR,     OUTPUT);  digitalWrite(DIR,    LOW);
  pinMode(ENABLE,  OUTPUT);  digitalWrite(ENABLE, HIGH); // active low
  pinMode(DTR,     OUTPUT);  digitalWrite(DTR,    LOW);

  // Configure analog input
  analogReference(DEFAULT);
  analogRead(VACSENSE); //this should put it into analog mode
  
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);

#ifdef MODBUS
  regBank.setId(1); //Modbus set device ID
  slave._device = &regBank;
  slave.setBaud(9600);   
#else
  Serial.begin(9600);
  Serial.println("EndEffector Demo");
#endif


}

// ******************************************************************************************
// MAIN LOOP
// ******************************************************************************************
void loop() 
{

#ifdef MODBUS
  slave.run();
  //Update the end effector peripherals based off of the new values received from MODBUS.
  
#else
  // put your main code here, to run repeatedly:
  Serial.print("Vacuum sensor reading: ");
  Serial.print(readVacuumSensor();
  delay(1000);
#endif

}

// ******************************************************************************************
// VACUUM SENSOR
// ******************************************************************************************
float readVacuumSensor()
{
  int reading = analogRead(VACSENSE);
  float scaled = reading * ANALOG_BITWEIGHT;
  return scaled;
}

// ******************************************************************************************
// STEPPER MOTOR
// ******************************************************************************************

void setMotorAngle(float angle)
{
  //todo: Implement this.
}

void pulseMotor(int steps)
{
  uint32_t numSteps;
  boolean dir;
  if (steps == 0)
  {
    return;
  }
  else if (steps < 0)
  {
    numSteps = -steps;
    digitalWrite(DIR,
  }
  
  for (int i=0;i<steps;i++)
  {
    pulsefast();
  }
}

// Optimized routine to send a really fast pulse to the DRV8825.
// It is hard-coded to use the STEP pin, since we've only got the one motor.
void pulsefast()
{
    stepOut = portOutputRegister(stepPort);

    uint8_t oldSREG = SREG;
    cli();

    *out |= stepBitmask;
    STEPPER_PULSE_DELAY;
    *out &= ~stepBitmask;
    SREG = oldSREG;
}

void enableMotor(boolean enable)
{
  if (enable)
  {
    digitalWrite(ENABLE, LOW); //active low
  }
  else
  {
    digitalWrite(ENABLE, HIGH);
  }
}

// ******************************************************************************************
// LED RING LIGHT
// ******************************************************************************************

void setLedBrightness(uint8_t brightness)
{
  //Input is 0-255. Output is 16-bit DAC.  
  uint16_t dacvalue = brightness * (uint16_t) 16;
  //Configure DAC
  dac.setVoltage(dacvalue, false);
}

void enableRingLight(boolean enable) 
{
  if (enable)
  {
    digitalWrite(ENABLE, LOW); //active low
  }
  else
  {
    digitalWrite(ENABLE, HIGH);
  }
}

