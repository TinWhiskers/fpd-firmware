// ******************************************************************************************
// TITLE:    ECAM02 Modbus Slave firmware
// AUTHOR:   Neil Jansen (neil@tinwhiskers.io)
// PLATFORM: Atmega328
// 
// https://github.com/TinWhiskers/fpd-design/wiki/ECAM02---End-Effector-PCB-w-USB-pass-through
// ******************************************************************************************

#define MODBUS 1 //Comment this line out to use normal serial console instead of MODBUS!

//TODO: 
// * Implement rolling averaging for analog reading.  
//     Put analog read function in an interrupt.  
//     Make a fixed ring array, and put readings into the ring array
//     When querying the end effector, return the averaged reading instead of ther instantaneous one.
// * Add thread to handle stepping in background.

#include <EEPROM.h>
#include <Wire.h> // I2C
#include <AccelStepper.h>
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

// Useful macros
#define CW       1   // For stepper motor
#define CCW      0   // For stepper motor

#define ANALOG_BITWEIGHT (5.0 / 1024.0)


// ******************************************************************************************
// GLOBAL VERIALBES
// ******************************************************************************************
Adafruit_MCP4725 dac;

// Global registers that can go across the MODBUS tubes
//-------------------------------------------------------------------------------------------
modbusDevice   regBank;                // Setup the brewtrollers register bank.  All of the data accumulated will be stored here
modbusSlave    slave(DTR);             // Create the modbus slave protocol handler
modbusRegister *reg_vacsense;          // Raw value from 10-bit ADC.
modbusRegister *reg_thetaCount;        // 0=0 degrees.  32767 = 360 degrees.
modbusRegister *reg_thetaEn;           // 0=off.  1=enable.
modbusRegister *reg_thetaCurrent;      // The current position of the theta stepper motor.
modbusRegister *reg_solenoid1;         // 0=off.  255=fully on.
modbusRegister *reg_solenoid2;         // 0=off.  255=fully on.
modbusRegister *reg_solenoid3;         // 0=off.  255=fully on.
modbusRegister *reg_solenoid4;         // 0=off.  255=fully on.
modbusRegister *reg_pump;              // 0=off.  255=fully on.
modbusRegister *reg_ringlight;         // 0=off.  255=fully on.

long thetaRawStepCurrent = 0; // Current raw step count for the stepper motor.
long thetaRawStepDesired = 0; // Desired raw step count for the stepper motor.
boolean thetaDirection = CW;
boolean thetaEnabled = false;

// Stepper motor
// http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR, 99, 99);

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

  //stepper.setEnablePin(ENABLE); // handle the enable pin outside of the library
  stepper.setMaxSpeed(100);    // Steps per second
  stepper.setSpeed(50);        // Steps per second
  stepper.setAcceleration(20); // Steps per second

  // stepBitmask = digitalPinToBitMask(STEP);
  // stepPort    = digitalPinToPort(STEP);

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);

#ifdef MODBUS
  regBank.setId(1); //Modbus set device ID
  slave._device = &regBank;
  slave.setBaud(9600);   

  // 00001-09999  Digital Outputs, A master device can read and write to these registers
  reg_thetaEn      = regBank.getRegister(00001);
  
  // 10001-19999  Digital Inputs, A master device can only read the values from these registers

  // 30001-39999  Analog Inputs, A master device can only read the values from these registers
  reg_vacsense     = regBank.getRegister(30001);  
  reg_thetaCurrent = regBank.getRegister(30002);  
  
  // 40001-49799  Analog Outputs, A master device can read and write to these registers 
  // 498512-49999  Analog Outputs, A master device can read and write to these registers 
  reg_thetaCount   = regBank.getRegister(40001);  
  reg_solenoid1    = regBank.getRegister(40003);  
  reg_solenoid2    = regBank.getRegister(40004);  
  reg_solenoid3    = regBank.getRegister(40005);  
  reg_solenoid4    = regBank.getRegister(40006);  
  reg_pump         = regBank.getRegister(40007);  
  reg_ringlight    = regBank.getRegister(40008);  

  // 49800-498511 EEPROM registers, 0-511 respectively
  

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

  //Read the vacuum sensor and update the register.
  reg_vacsense    ->set((word)readVacuumSensor() );
  reg_thetaCurrent->set((word)stepper.currentPosition() );

  updateTheta(      reg_thetaCount->get() );
  setCoil(    SOL1, reg_solenoid1 ->get() );
  setCoil(    SOL2, reg_solenoid2 ->get() );
  setCoil(    SOL3, reg_solenoid3 ->get() );
  setCoil(    SOL4, reg_solenoid4 ->get() );
  setCoil(    PUMP, reg_pump      ->get() );
  setLedBrightness( reg_ringlight ->get() );
  enableMotor( (boolean) reg_thetaEn   ->get() );

  
#else
  // put your main code here, to run repeatedly:
  Serial.print("Vacuum sensor reading: ");
  Serial.print(readVacuumSensor();
  delay(1000);
#endif

  stepper.run();  //Poll the motor and step it if a step is due
}

// ******************************************************************************************
// VACUUM SENSOR
// ******************************************************************************************
int readVacuumSensor()
{
  return analogRead(VACSENSE);
}

// ******************************************************************************************
// PWM COILS / MOTORS
// ******************************************************************************************
void setCoil(uint8_t pin, uint8_t pwmValue)
{
  analogWrite(pin, pwmValue);
}


// ******************************************************************************************
// STEPPER MOTOR
// ******************************************************************************************

void updateTheta(int count)
{
  stepper.moveTo(count);
}

void enableMotor(boolean enabled)
{
  digitalWrite(ENABLE, !enabled);
}

// ******************************************************************************************
// LED RING LIGHT
// ******************************************************************************************

void setLedBrightness(uint8_t brightness)
{
  if (brightness == 0)
  {
    digitalWrite(ENABLE, HIGH);
  }
  else
  {
    digitalWrite(ENABLE, LOW); //active low
  }
  //Input is 0-255. Output is 16-bit DAC.  
  uint16_t dacvalue = brightness * (uint16_t) 16;
  //Configure DAC
  dac.setVoltage(dacvalue, false);
}

