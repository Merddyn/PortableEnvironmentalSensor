#include <Wire.h>

#include <SensirionI2CScd4x.h>
#include <SoftwareSerial.h>
#include <ParallaxLCD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_seesaw.h"

#define ROWS 2 // LCD Display Rows
#define COLS 16 // LCD Display Columns


// Data wire is conntec to the Arduino digital pin 6
#define ONE_WIRE_BUS 6

#define SENSOR_MODES 5



/************************************************************ MQ2 Sensor ************************************************************/

// Credit for MQ2 sensor functions goes to https://projecthub.arduino.cc/m_karim02/arduino-and-mq2-gas-sensor-f3ae33

#define         MQ_PIN                       (0)     //define which   analog input channel you are going to use
#define         RL_VALUE                     (5)      //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR           (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                      //which is derived from the   chart in datasheet
 
/**********************Software Related Macros***********************************/
#define          CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are   going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL   (500)   //define the time interal(in milisecond) between each samples in the
                                                      //cablibration phase
#define          READ_SAMPLE_INTERVAL         (50)    //define how many samples you are   going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)      //define the time interal(in milisecond) between each samples in 

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7   = 2;
 
/*********************Application Related Macros*********************************/
#define          GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define          GAS_SMOKE                    (2)
 
/****************************Globals**********************************************/
float            LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent"
                                                    //to   the original curve. 
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float            COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent" 
                                                    //to   the original curve.
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float            SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent" 
                                                    //to   the original curve.
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float            Ro           =  10;                 //Ro is initialized to 10 kilo ohms


/************************************************************************************************************************************/


/************************************************************ CO2 Sensor ************************************************************/
// Reference: example usage file of the library.
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
//CO2 Sensor
SensirionI2CScd4x scd4x;
  uint16_t co2 = 0;
    float CO2temperature = 0.0f;
    float humidity = 0.0f;
    bool isDataReady = false;
    
/************************************************************************************************************************************/

/************************************************************ Soil Sensor ***********************************************************/

// Reference: https://learn.adafruit.com/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor/arduino-test
// Soil Sensor
Adafruit_seesaw SoilSensor;
float soilTempC;
uint16_t capread;

/************************************************************************************************************************************/

// Reference: https://forum.arduino.cc/t/help-with-parallax-serial-lcd-and-arduino-uno/133334/6
ParallaxLCD lcd(18,ROWS,COLS); // Setup LCD on pin 18


// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Reference: https://randomnerdtutorials.com/guide-for-ds18b20-temperature-sensor-with-arduino/
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

/************************************************************************************************************************************/

/************************************************************ LM35 Sensor ***********************************************************/

//LM35 temperature sensor variables
const int LM35_sensor = A3; // Assigning analog pin A5 to variable 'sensor'
float LM35_tempc; //variable to store temperature in degree Celsius
float LM35_tempf; //variable to store temperature in Fahreinheit
float LM35_vout; //temporary variable to hold sensor reading 

/************************************************************************************************************************************/

/******************************************************** Hall Effect Sensor ********************************************************/

// Reference: https://maker.pro/arduino/tutorial/how-to-use-a-hall-effect-sensor-with-arduino

// Hall Effect Sensor variables
 volatile byte half_revolutions;
 unsigned int rpm;
 unsigned long timeold;

// Touch Sensor Module
int SensorMode;
  
/************************************************************************************************************************************/


/*************************************************************************************************************************************************************/
/**********************************************************************      Setup      **********************************************************************/
/*************************************************************************************************************************************************************/

void setup() {
 
  // Start serial communication for debugging purposes
  Serial.begin(9600);
  // Start up the library

  // ***************** Begin Temperature Sensor Setup ****************
  sensors.begin();
  pinMode(LM35_sensor, INPUT); // Configuring sensor pin as input 
  // ****************** End Temperature Sensor Setup *****************


  // ******************** Begin Soil Sensor Setup ********************
  
  if (!SoilSensor.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    //while(1) delay(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(SoilSensor.getVersion(), HEX);
  }
  // ********************* End Soil Sensor Setup ********************* 

  // ************************ Begin LCD setup ************************
  lcd.setup(); // Library assumes 192000 baud.
  delay(250);
  lcd.empty(); // Clear display since it may still have data from  previous testing.

  delay(250);
  lcd.backLightOn();
  // ************************* End LCD setup *************************
  
  Wire.begin();

  // ***************** Begin Capacitive Sensor Setup *****************
  
  // Switch for which sensor to check and display the info of.
  SensorMode = 0;
  attachInterrupt(1, touchSensor, RISING);// Initialize the intterrupt pin (Arduino digital pin 2)

  // ****************** End Capacitive Sensor Setup ******************
  
  // ***************** Begin Hall Effect Sensor Setup ****************

  attachInterrupt(0, magnet_detect, RISING);// Initialize the intterrupt pin (Arduino digital pin 2)
   half_revolutions = 0;
   rpm = 0;
   timeold = 0;

  // ****************** End Hall Effect Sensor Setup *****************
  
  // ********************* Begin MQ2 Sensor Setup ********************
  
  Serial.print("Calibrating...");                
   Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please   make sure the sensor is in clean air 
                                                     //when   you perform the calibration                    
  Serial.println("Calibration   is done..."); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.println("kohm");

  // ********************** End MQ2 Sensor Setup *********************
  
  // ********************* Begin CO2 Sensor Setup ********************


  
  uint16_t error;
  char errorMessage[256];
  
  
  scd4x.begin(Wire);

  Serial.println("Stopping Periodic Measurement");
  
// stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    
    Serial.println("Getting Serial Number");
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    
    Serial.println("Starting Periodic Measurement");
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");

    
  // ********************** End CO2 Sensor Setup *********************
}


/*************************************************************************************************************************************************************/
/**********************************************************************      Loop      ***********************************************************************/
/*************************************************************************************************************************************************************/


void loop() 
{
  // Vars added by CO2 Sensor
    uint16_t error;
    char errorMessage[256];

     delay(100);

  //Measure RPM for Hall Effect Sensor
  
   if (half_revolutions >= 20) 
   { 
     rpm = 30*1000/(millis() - timeold)*half_revolutions;
     timeold = millis();
     half_revolutions = 0;
     Serial.println(rpm,DEC);
   }

/***************************************** READ SENSORS *****************************************/

  // ***************** LM35 Sensor *****************
    
  LM35_vout = analogRead(LM35_sensor); //Reading the value from sensor
  LM35_vout = (LM35_vout * 500) / 1023;
  LM35_tempc = LM35_vout; // Storing value in Degree Celsius
  LM35_tempf = (LM35_vout*1.8)+32; // Converting to Fahrenheit 

  // ***************** Soil Sensor *****************
    
  soilTempC = SoilSensor.getTemp();
  capread = SoilSensor.touchRead(0);

  Serial.print("Soil Temperature: "); Serial.print(soilTempC); Serial.println("*C");
  Serial.print("Capacitive: "); Serial.println(capread);
  
  // ******** Waterproof Temperature Sensor ********

  sensors.requestTemperatures(); 
  
  Serial.print(sensors.getTempCByIndex(0)); 
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(sensors.getTempFByIndex(0));

  // ***************** CO2 Sensor ******************
  // Read Measurement
  co2 = 0;
  CO2temperature = 0.0f;
  humidity = 0.0f;
  isDataReady = false;
  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
      Serial.print("Error trying to execute getDataReadyFlag(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      return;
  }
  if (!isDataReady) {
      return;
  }
  
  error = scd4x.readMeasurement(co2, CO2temperature, humidity);
  
  if (error) 
  {
    lcd.print("Error.");
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else if (co2 == 0) {
    lcd.print("Invalid Sample.");
      Serial.println("Invalid sample detected, skipping.");
  } else {
      Serial.print("Co2:");
      Serial.print(co2);
      Serial.print("\t");
      Serial.print("Temperature:");
      Serial.print(CO2temperature);
      Serial.print("\t");
      Serial.print("Humidity:");
      Serial.println(humidity);
  }

  // ***************** MQ2 Sensor ******************

  Serial.print("LPG:"); 
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG)   );
  Serial.print( "ppm" );
  Serial.print("    ");   
  Serial.print("CO:");   
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
  Serial.print(   "ppm" );
  Serial.print("    ");   
  Serial.print("SMOKE:"); 
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
  Serial.println(   "ppm" );

  // ********************************* DISPLAY SWITCH *********************************

  switch (SensorMode)
  {
    case 0: // Ambient Temperature Sensor
      DisplayAmbientTemperature();
    break;
    case 1: // Soil Sensor
      DisplaySoilSensor();
    break;
    case 2: // Waterproof Temperature Sensor
      DisplayWaterproofTemperature();
    break;
    case 3: // CO2 Sensor
          DisplayCO2Sensor();
    break;
    case 4: // MQ2 Sensor
       DisplayMQ2Sensor();
    break;
  }
    
  delay(2500);                      // wait a few seconds

}





/*************************************************************************************************************************************************************/
/********************************************************************      Functions      ********************************************************************/
/*************************************************************************************************************************************************************/


/************************************************************ Interrupts ************************************************************/

 void magnet_detect() //This function is called whenever a magnet/interrupt is detected by the arduino
 {
   half_revolutions++;
   Serial.println("detect");
   lcd.pos(0,0);
   lcd.print("Magnet Detected!");
 }


 
void touchSensor() // This function is called whenever the capactitive touch sensor is triggered.
{
  SensorMode++;
  if (SensorMode == SENSOR_MODES)
    SensorMode = 0;
    
   Serial.print("Button Pressed! Mode: ");
   Serial.print(SensorMode);
   switch (SensorMode)
  {
    case 0: // Ambient Temperature Sensor
      DisplayAmbientTemperature();
    break;
    case 1: // Soil Sensor
      DisplaySoilSensor();
    break;
    case 2: // Waterproof Temperature Sensor
      DisplayWaterproofTemperature();
    break;
    case 3: // CO2 Sensor
          DisplayCO2Sensor();
    break;
    case 4: // MQ2 Sensor
       DisplayMQ2Sensor();
    break;
  }
}
  
/************************************************************************************************************************************/


/******************************************************** Display Functions *********************************************************/


void QuickClearLCD() // Manually clearing the screen is 1/5 the time of the lcd.empty function. ~18ms vs ~102ms.
{
  lcd.pos(0,0);
  lcd.print("                ");
  lcd.pos(1,0);
  lcd.print("                ");
  lcd.pos(0,0);
}

void DisplayAmbientTemperature() // Displays data from the LM35 temperature sensor.
{
  QuickClearLCD();
  lcd.print("Ambient Temp:");
  lcd.cr();
  lcd.print(LM35_tempf);
  lcd.print("F");
  lcd.print("  ");
  lcd.print(LM35_tempc);
  lcd.print("C  "); // Extra space is to clear screen just in case.
}

void DisplaySoilSensor() // Displays data from the Soil Moisture Sensor
{
  QuickClearLCD();
  lcd.print("Soil Sensor:");
  lcd.pos(1,0);
  lcd.print(ConvertCToF(soilTempC));
  lcd.print("F");
  lcd.print("  ");
  lcd.print(capread);
  lcd.print("cpc");
}

void DisplayWaterproofTemperature() // Displays data from the Dallas Waterproof Temperature Sensor
{
  QuickClearLCD();
  lcd.print("Waterproof Temp:");
  lcd.pos(1,0);
  lcd.print(sensors.getTempFByIndex(0));
  lcd.print("F");
  lcd.print("  ");
  lcd.print(sensors.getTempCByIndex(0));
  lcd.print("C");
}

void DisplayCO2Sensor() // Displays data from the CO2 Sensor
{
  QuickClearLCD();
  lcd.print("CO2: ");
  lcd.print(co2);
  lcd.print("ppm");
  lcd.pos(1,0);
  lcd.print(ConvertCToF(CO2temperature));
  lcd.print("F ");
  lcd.print(humidity);
  lcd.print("%hmd");    
}

void DisplayMQ2Sensor() // Displays data from the gas sensor
{
  QuickClearLCD();
      lcd.print("LPG:");
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG)   );
      lcd.print( "ppm" );
      lcd.print(" ");  
      lcd.pos(0, 9);
    lcd.print("CO:");   
    lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
    lcd.print( "ppm"   );
    lcd.print("    "); 
    lcd.pos(1, 0);  
    lcd.print("SMOKE:");   
    lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
    lcd.print("ppm");
    lcd.print("      ");
}

/************************************************************************************************************************************/

float ConvertCToF(float tempC) // Converts temperature from Centigrade to Fahrenheit
{
  return (tempC * 1.8) + 32;
}


// 

// Used by CO2 Sensor 
void printUint16Hex(uint16_t value) { // Adds padding to the left for consistent length.
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

// Used by CO2 Sensor
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}


// MQ2 Sensor Functions

// Credit for these functions goes to https://projecthub.arduino.cc/m_karim02/arduino-and-mq2-gas-sensor-f3ae33

/****************   MQResistanceCalculation **************************************
Input:   raw_adc   - raw value read from adc, which represents the voltage
Output:  the calculated   sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider.   Given the voltage
         across the load resistor and its resistance, the resistance   of the sensor
         could be derived.
**********************************************************************************/   
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
   
/*************************** MQCalibration **************************************
Input:    mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function   assumes that the sensor is in clean air. It use  
         MQResistanceCalculation   to calculates the sensor resistance in clean air 
         and then divides it   with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs   slightly between different sensors.
**********************************************************************************/   
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
   for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
     val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
   }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average   value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided   by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according   to the chart in the datasheet 
 
  return val; 
}
/***************************   MQRead *******************************************
Input:   mq_pin - analog   channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation   to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor   is in the different consentration of the target
         gas. The sample times   and the time interval between samples could be configured
         by changing   the definition of the macros.
**********************************************************************************/   
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++)   {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
   }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/***************************   MQGetGasPercentage ********************************
Input:   rs_ro_ratio -   Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the   target gas
Remarks: This function passes different curves to the MQGetPercentage   function which 
         calculates the ppm (parts per million) of the target   gas.
**********************************************************************************/   
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id   == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else   if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
   } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
   }    
 
  return 0;
}
 
/***************************  MQGetPercentage   ********************************
Input:   rs_ro_ratio - Rs divided by Ro
          pcurve      - pointer to the curve of the target gas
Output:  ppm of   the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic   value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided.   As it is a 
         logarithmic coordinate, power of 10 is used to convert the   result to non-logarithmic 
         value.
**********************************************************************************/   
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,(   ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}



/************************************************************ Credits and References ************************************************************/

/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


 // https://projecthub.arduino.cc/m_karim02/arduino-and-mq2-gas-sensor-f3ae33
 
 // https://howtomechatronics.com/tutorials/arduino/how-i2c-communication-works-and-how-to-use-it-with-arduino/
 // https://www.electronicshub.org/arduino-mega-pinout/

 // LM35 Datasheet https://www.ti.com/lit/ds/symlink/lm35.pdf?ts=1732137190804
 // SCD4X Datasheet https://sensirion.com/media/documents/48C4B7FB/66E05452/CD_DS_SCD4x_Datasheet_D1.pdf
 
 // https://github.com/Sensirion/arduino-core
 // https://github.com/Sensirion/arduino-i2c-scd4x/tree/master
 // https://sensirion.com/media/documents/7E9BD4B5/6183F7BE/Sensirion_CO2_Sensors_SCD4x_testing_guide.pdf
 // https://projecthub.arduino.cc/hibit/using-touch-sensor-with-arduino-83b957
 // https://github.com/arduino/Arduino/issues/5840
 // https://www.instructables.com/Arduino-Temperature-Sensor-Using-LM35/
 // https://docs.arduino.cc/learn/communication/wire/
 // https://learn.adafruit.com/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor/arduino-test
 // https://randomnerdtutorials.com/guide-for-ds18b20-temperature-sensor-with-arduino/
 // https://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino/the-whole-enchilada
 // https://github.com/iamthechad/parallax_lcd/blob/master/src/ParallaxLCD.cpp
 // https://www.parallax.com/product/parallax-2-x-16-serial-lcd-with-piezo-speaker-backlit/
 
/************************************************************ Components ************************************************************/

// Parallax 2x16 LCD https://www.parallax.com/product/parallax-2-x-16-serial-lcd-with-piezo-speaker-backlit/
// MQ2 Sensor https://www.sunfounder.com/products/mq-2-gas-sensor-module
// Keyes LM35 Temperature Sensor
// SCD41 CO2 Sensor https://developer.sensirion.com/product-support/scd4x-co2-sensor
// Keyes Capacitive Touch Sensor
// Keyes Hall Effect Sensor
// DS18B20 Waterproof Temperature Sensor
// Adafruit Stemma Soil Sensor https://learn.adafruit.com/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor/arduino-test
// Keyes sensors from https://www.microcenter.com/product/603752/inland-37-in-1-sensor-kit
