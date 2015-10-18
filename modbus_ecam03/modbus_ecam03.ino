// ******************************************************************************************
// TITLE:    ECAM03 Modbus Slave firmware
// AUTHOR:   Neil Jansen (neil@tinwhiskers.io)
// PLATFORM: Atmega328
// 
// https://github.com/TinWhiskers/fpd-design/wiki/ECAM03---Up-Looking-Camera-Board
// ******************************************************************************************

#define MODBUS 1 //Comment this line out to use normal serial console instead of MODBUS!

//TODO: 
// Add watchdog timer
// Add charge pump
// 

#include <avr/eeprom.h>
#include <EEPROM.h>             // EEPROM (modbus library)
#include <SPI.h>                // SPI for thermocouple chip
#include <Wire.h>               // I2C for DAC
#include <PID_v1.h>             // PID control for the heated bed
#include <Adafruit_MCP4725.h>   // DAC for controlling ring light 
#include <Adafruit_NeoPixel.h>  // For user lighting
#include <Adafruit_MAX31855.h>  // Type K thermocouple chip
#include <Watchdog.h>           // AVR watchdog timer for heated bed safety
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

// ******************************************************************************************
// PIN DEFINES, MACROS, ETC
// ******************************************************************************************
#define SSR      3   // Solid-state relay for heated bed.
#define DTR      4   // RS-485 transmit enable
#define FANS     5   // PWM control for fans.  Active high.
#define LED_EN   7   // Enable for AP3032 controller
#define LED_RGB  8   // WS2812B lighting
#define MAX_SS   10  // Slave select for MAX31855

enum heatmode_t {
  OFF,
  HEAT_PID,
  HEAT_MANUAL,
  COOL_PID,
  COOL_MANUAL
};

#define HEATMAX 255 // Change this to less than 255 to limit the heater


// ******************************************************************************************
// GLOBAL VERIALBES
// ******************************************************************************************
// DAC for setting LED intensity
Adafruit_MCP4725 dac;

//Thermocouple
Adafruit_MAX31855 thermocouple(MAX_SS);

// NeoPixel library
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(8, LED_RGB, NEO_GRB + NEO_KHZ800);

// PID Loop for heating and cooling
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
double heatSetpoint;
double heatInput;
double heatOutput;
PID heatPID(&heatInput, &heatOutput, &heatSetpoint, 0, 0, 0, DIRECT);
double coolSetpoint;
double coolInput;
double coolOutput;
PID coolPID(&coolInput, &coolOutput, &coolSetpoint, 0, 0, 0, DIRECT);

heatmode_t heatMode = OFF;
float   temperature = 0;
uint8_t fanSpeed    = 0;
uint8_t ssrDutyCycle = 0;

// Global registers that can go across the MODBUS tubes
//-------------------------------------------------------------------------------------------
modbusDevice   regBank;              // Setup the brewtrollers register bank.  All of the data accumulated will be stored here
modbusSlave    slave(DTR);           // Create the modbus slave protocol handler
modbusRegister *reg_chargepump;      // Heated bed will turn off if this register isn't flicked once per second.
modbusRegister *reg_hb_temp;         // Heated bed temperature (degrees, C
modbusRegister *reg_fan;             // Fan intensity.
modbusRegister *reg_hb_duty;         // Duty cycle for heating element
modbusRegister *reg_hb_mode;         // Enum (see below)
modbusRegister *reg_hb_setpoint;     // Temperature setpoint, in degrees C.
modbusRegister *reg_hb_p;            // PID P setting for the heated bed heating
modbusRegister *reg_hb_i;            // PID I setting for the heated bed heating
modbusRegister *reg_hb_d;            // PID D setting for the heated bed heating
modbusRegister *reg_cool_p;          // PID P setting for the heated bed cooling
modbusRegister *reg_cool_i;          // PID I setting for the heated bed cooling
modbusRegister *reg_cool_d;          // PID D setting for the heated bed cooling
modbusRegister *reg_ringlight;       // Ringlight intensity. 0=off. 255=fully on.
modbusRegister *reg_rgb_0r;          // RGB LED #0, red
modbusRegister *reg_rgb_0g;          // RGB LED #0, green
modbusRegister *reg_rgb_0b;          // RGB LED #0, blue
modbusRegister *reg_rgb_1r;          // RGB LED #1, red
modbusRegister *reg_rgb_1g;          // RGB LED #1, green
modbusRegister *reg_rgb_1b;          // RGB LED #1, blue
modbusRegister *reg_rgb_2r;          // RGB LED #2, red
modbusRegister *reg_rgb_2g;          // RGB LED #2, green
modbusRegister *reg_rgb_2b;          // RGB LED #2, blue
modbusRegister *reg_rgb_3r;          // RGB LED #3, red
modbusRegister *reg_rgb_3g;          // RGB LED #3, green
modbusRegister *reg_rgb_3b;          // RGB LED #3, blue
modbusRegister *reg_rgb_4r;          // RGB LED #4, red
modbusRegister *reg_rgb_4g;          // RGB LED #4, green
modbusRegister *reg_rgb_4b;          // RGB LED #4, blue
modbusRegister *reg_rgb_5r;          // RGB LED #5, red
modbusRegister *reg_rgb_5g;          // RGB LED #5, green
modbusRegister *reg_rgb_5b;          // RGB LED #5, blue
modbusRegister *reg_rgb_6r;          // RGB LED #6, red
modbusRegister *reg_rgb_6g;          // RGB LED #6, green
modbusRegister *reg_rgb_6b;          // RGB LED #6, blue
modbusRegister *reg_rgb_7r;          // RGB LED #7, red
modbusRegister *reg_rgb_7g;          // RGB LED #7, green
modbusRegister *reg_rgb_7b;          // RGB LED #7, blue


// ******************************************************************************************
// SETUP
// ******************************************************************************************
void setup()
{
  //Configure pin modes and set default states
  pinMode(SSR,     OUTPUT);  digitalWrite(SSR,    LOW);
  pinMode(FANS,    OUTPUT);  digitalWrite(FANS,   LOW);
  pinMode(LED_RGB, OUTPUT);  digitalWrite(LED_RGB,LOW);
  pinMode(LED_EN,  OUTPUT);  digitalWrite(LED_EN, LOW);
  pinMode(DTR,     OUTPUT);  digitalWrite(DTR,    LOW);
  pinMode(MAX_SS,  OUTPUT);  digitalWrite(MAX_SS, LOW);

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);

  pixels.begin(); // This initializes the NeoPixel library.

#ifdef MODBUS
  regBank.setId(2); //Modbus set device ID
  slave._device = &regBank;
  slave.setBaud(9600);   

  // 00001-09999  Digital Outputs, A master device can read and write to these registers
  reg_chargepump  = regBank.getRegister(00001);

  // 10001-19999  Digital Inputs, A master device can only read the values from these registers

  // 30001-39999  Analog Inputs, A master device can only read the values from these registers
  reg_hb_temp     = regBank.getRegister(30001);
  reg_fan         = regBank.getRegister(30002);
  reg_hb_duty     = regBank.getRegister(30003);

  // 40001-49799  Analog Outputs, A master device can read and write to these registers 
  // 498512-49999  Analog Outputs, A master device can read and write to these registers 
  reg_hb_mode     = regBank.getRegister(40001);
  reg_hb_setpoint = regBank.getRegister(40002);
  reg_hb_p        = regBank.getRegister(40003);
  reg_hb_i        = regBank.getRegister(40004);
  reg_hb_d        = regBank.getRegister(40005);
  reg_cool_p      = regBank.getRegister(40006);
  reg_cool_i      = regBank.getRegister(40007);
  reg_cool_d      = regBank.getRegister(40008);
  reg_ringlight   = regBank.getRegister(40009);
  reg_rgb_0r      = regBank.getRegister(40010);
  reg_rgb_0g      = regBank.getRegister(40011);
  reg_rgb_0b      = regBank.getRegister(40012);
  reg_rgb_1r      = regBank.getRegister(40013);
  reg_rgb_1g      = regBank.getRegister(40014);
  reg_rgb_1b      = regBank.getRegister(40015);
  reg_rgb_2r      = regBank.getRegister(40016);
  reg_rgb_2g      = regBank.getRegister(40017);
  reg_rgb_2b      = regBank.getRegister(40018);
  reg_rgb_3r      = regBank.getRegister(40019);
  reg_rgb_3g      = regBank.getRegister(40020);
  reg_rgb_3b      = regBank.getRegister(40021);
  reg_rgb_4r      = regBank.getRegister(40022);
  reg_rgb_4g      = regBank.getRegister(40023);
  reg_rgb_4b      = regBank.getRegister(40024);
  reg_rgb_5r      = regBank.getRegister(40025);
  reg_rgb_5g      = regBank.getRegister(40026);
  reg_rgb_5b      = regBank.getRegister(40027);
  reg_rgb_6r      = regBank.getRegister(40028);
  reg_rgb_6g      = regBank.getRegister(40029);
  reg_rgb_6b      = regBank.getRegister(40030);
  reg_rgb_7r      = regBank.getRegister(40031);
  reg_rgb_7g      = regBank.getRegister(40032);
  reg_rgb_7b      = regBank.getRegister(40033);

  // 49800-498511 EEPROM registers, 0-511 respectively
  

#else
  Serial.begin(9600);
  Serial.println("UpCam Demo");
#endif

  heatPID.SetMode(MANUAL);
  heatPID.SetOutputLimits(0,255);
  heatPID.SetSampleTime(200); //Set the sample time, in milliseconds
  heatPID.SetControllerDirection(DIRECT); //DIRECT or REVERSE
  
  coolPID.SetMode(MANUAL);
  coolPID.SetOutputLimits(0,255);
  coolPID.SetSampleTime(200); //Set the sample time, in milliseconds
  coolPID.SetControllerDirection(DIRECT); //DIRECT or REVERSE

  watchdog.enableIsr(watchdogIsr);
  watchdog.begin(Watchdog::TIMEOUT_250MS); //valid values: 16/32/64/125/250/500/1000/2000/4000/8000 milliseconds

}

// ******************************************************************************************
// MAIN LOOP
// ******************************************************************************************
void loop() 
{
  temperature = thermocouple.readCelsius();
  watchdog.resetTimer();
  
#ifdef MODBUS
  slave.run();
  //Update the end effector peripherals based off of the new values received from MODBUS.

  // Now convert the temperature to MODBUS int.
  // Its resolution is 0.25 degrees C, so we'll just multiply by 4 and cast to an int.
  int rawtemp = (int)(temperature * 4.0);

  //Read the vacuum sensor and update the register.
  reg_hb_temp->set((word) rawtemp);
  reg_fan    ->set((word) fanSpeed );
  reg_hb_duty->set((word) ssrDutyCycle );

  chargepump(       reg_chargepump ->get() ); // Run the charge pump function 
  setLedBrightness( reg_ringlight  ->get() ); // Set ring light brightness

  
  switch (reg_hb_mode->get() )
  {
  // Don't do anything
  case 0:  
    heatMode = OFF;
         
    return;
  // Use PID values to control the heater 
  case 1: 
    heatMode = HEAT_PID;
    heatPID.SetTunings(reg_hb_p->get(),   reg_hb_i->get(),   reg_hb_d->get());
    heatPID.SetMode(AUTOMATIC);

    heatSetpoint = ((float)reg_hb_setpoint->get() / 4.0);
    heatInput = temperature;
    heatPID.Compute();
    setBed(heatOutput);
    return;
    
  // Use a manual PWM setting to control the heater
  case 2: 
    heatMode = HEAT_MANUAL;
    setBed(reg_hb_setpoint->get());
    return;
    
  // Use PID values to control the fans
  case 3:
    heatMode = COOL_PID;
    coolPID.SetTunings(reg_cool_p->get(), reg_cool_i->get(), reg_cool_d->get());
    coolSetpoint = ((float)reg_hb_setpoint->get() / 4.0);
    return;
    
  // Use a manual PWM setting to control the fans
  case 4: 
    heatMode=COOL_MANUAL
    setFans(value);
    return;
    
  case default:
    //TODO: Error out here
    return;
  }


double heatInput;
double heatOutput;
PID heatPID(&heatInput, &heatOutput, &heatSetpoint, 0, 0, 0, DIRECT);
double coolSetpoint;
double coolInput;
double coolOutput;
PID coolPID(&coolInput, &coolOutput, &coolSetpoint, 0, 0, 0, DIRECT);


  // Update NeoPixels based off of register settings
  pixels.setPixelColor(0, pixels.Color(reg_rgb_0r->get(), reg_rgb_0g->get(), reg_rgb_0b->get() ));
  pixels.setPixelColor(1, pixels.Color(reg_rgb_1r->get(), reg_rgb_1g->get(), reg_rgb_1b->get() ));
  pixels.setPixelColor(2, pixels.Color(reg_rgb_2r->get(), reg_rgb_2g->get(), reg_rgb_2b->get() ));
  pixels.setPixelColor(3, pixels.Color(reg_rgb_3r->get(), reg_rgb_3g->get(), reg_rgb_3b->get() ));
  pixels.setPixelColor(4, pixels.Color(reg_rgb_4r->get(), reg_rgb_4g->get(), reg_rgb_4b->get() ));
  pixels.setPixelColor(5, pixels.Color(reg_rgb_5r->get(), reg_rgb_5g->get(), reg_rgb_5b->get() ));
  pixels.setPixelColor(6, pixels.Color(reg_rgb_6r->get(), reg_rgb_6g->get(), reg_rgb_6b->get() ));
  pixels.setPixelColor(7, pixels.Color(reg_rgb_7r->get(), reg_rgb_7g->get(), reg_rgb_7b->get() ));
  pixels.show(); // This sends the updated pixel color to the hardware.


  
#else
  // put your main code here, to run repeatedly:
  Serial.print("Vacuum sensor reading: ");
  Serial.print(readVacuumSensor();
  delay(1000);
#endif

}

// ******************************************************************************************
// HEATED BED
// ******************************************************************************************
void setBed(uint8_t pwmValue)
{
  analogWrite(SSR, pwmValue);
}

// ******************************************************************************************
// THERMOCOUPLE MEASUREMENT
// ******************************************************************************************

// ******************************************************************************************
// NEOPIXEL LIGHTING
// ******************************************************************************************


// ******************************************************************************************
// FANS
// ******************************************************************************************
void setFans(uint8_t pwmValue)
{
  analogWrite(FANS, pwmValue);
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

// ******************************************************************************************
// CHARGE PUMP
// ******************************************************************************************
void chargepump(int newvalue)
{
  //TODO: Implement charge pump
}

// ******************************************************************************************
// WATCHDOG
// ******************************************************************************************
void watchdogIsr()
{
  Serial << "Test isr!" << endl;
}

