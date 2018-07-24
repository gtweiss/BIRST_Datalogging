//Ball Intern Remote Sensing Team (BIRST)
//Ball'd Eagles - 2018 Balloon Team

//Import EEPROM for Data Security
#include<EEPROM.h>

//Import Libraries for the Pressure Sensor
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

//Import Libaries for the Air Quality Sensor
#include <SparkFunCCS811.h>

//Import Libraries for the Humidity Sensor
#include <SparkFun_HIH4030.h>

//Import Libraries for SD Card Datalogging and set up SD Port location for Teensy 3.5
#include <SD.h>
#include <SPI.h>
const int chipSelect = BUILTIN_SDCARD;

//Import Libraries for Servo Motor and declare Global class
#include <Servo.h>
Servo Servo1;
int servoPin = 6;
int servoLoop = 0;
bool started = false;

//Create Class for the pressure Sensor with the High Address (0x76)
MS5803 pressure_Sensor(ADDRESS_HIGH);

//Create the global variables for humidity sensor
#define HIH4030_OUT A21     // Pin 17 on Payload Output
#define HIH4030_SUPPLY 5    // 5 volts sourcing
float relativeHumidity = 0;

//Define Class instance for humidity
HIH4030 humidity_Sensor(HIH4030_OUT, HIH4030_SUPPLY);

//Create Class for the Air Quality Sensor with the Default Address (0x5A)
CCS811 air_Sensor(0x5B);

//Create the global variables for pressure data collection.
float temperature_c;        //Temperature in Degrees C
double pressure_data[4];    //[Pressure Baseline, Absolute Pressure, Relative Pressure, Altitude Change]

//Create the global variables for air quality data collection
int i=0;
uint16_t airq_data[2];      //[CO2, tVOC]
volatile bool interr = false;

//Create the global variables for the interior temperature sensor
int AintTemp = 0;
float intTemp = 0;


//Construct General Support Functions
void serial_Print();


//Construct Support Functions for pressure
void pressure_Setup();
void collect_pressure_data();
double altitude(double P, double P0);

//Construct Support Functions for Air Quality
void airq_Setup();
void collect_airq();

//Construct Support Functions for Humidity Sensor
void collect_humidity();

//Construct Support Functons for SD Card Port
void sd_Setup();
void sd_Save();

//Construct Support functions for interior Temp Sensor
void intTemp_Collect();

//Consrtuct Support functions for the heater
void heater_Setup();
void check_Heater();

//Construct Support Functions for the Servo
void servo_Setup();
void servo_Run();

//Set initial time
float initialTime = millis();

void setup() {
  //set Up temp Pin
  pinMode(A22,INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  
  //Uncomment the Serial Line below if using Serial to test
  Serial.begin(9600);

  //Set up Presure Sensor
  pressure_Setup();

  //Set up Air Quality Sensor
  airq_Setup();

  //Set up SD Card
  sd_Setup();

  //Set up heater
  heater_Setup();

  //Set up Servo
  //servo_Setup();


}

void loop() {

  //Collect Pressure Data
  collect_pressure_data();

  //Collect Humidity Data
  collect_humidity();

  //Collect Air Quality Data
  collect_airq();

  //Collect Air Quality Data
  intTemp_Collect();

  //Set Servo Motor
  //servo_Run();

  //Save Values
  sd_Save();
  
  //Print Values
  serial_Print();

  //Check the heater to ensure there is no overheat
  check_Heater();

  delay(1000);
  digitalWrite(13,HIGH);
}

void serial_Print(){
  //This functions prints results and can be easily muted
  Serial.print("External Temperature C = ");
  Serial.println(temperature_c);

  Serial.print("Interior Temperature C = ");
  Serial.println(intTemp);

  Serial.print("Pressure abs (mbar)= ");
  Serial.println(pressure_data[1]);

  Serial.print("Altitude change (m) = ");
  Serial.println(pressure_data[3]);

  Serial.print("Relative Humidity = ");
  Serial.println(relativeHumidity);

  Serial.print("Relative Time = ");
  Serial.println((millis()-initialTime)/60000.0);
  
  Serial.print("CO2[");
  Serial.print(airq_data[0]);
  Serial.print("] tVOC[");
  Serial.print(airq_data[1]);
  Serial.println("]");
  Serial.println();
  
}

void pressure_Setup(){
  //This function Sets up the pressure sensor to collect data and sets base values
  pressure_Sensor.reset();
  pressure_Sensor.begin();

  //Set baseline value for pressure
  delay(500);
  pressure_data[0] = pressure_Sensor.getPressure(ADC_4096);

  //Set Baseline for flight in case of power loss
  if(pressure_data[0]<EEPROM.read(2)){
    pressure_data[0] = EEPROM.read(2);
  }
  else{
    EEPROM.write(2,pressure_data[0]);
  }
}

void collect_pressure_data(){
  //Record Temperature value in Celsius
  temperature_c = pressure_Sensor.getTemperature(CELSIUS, ADC_4096);

  //Record current pressure
  pressure_data[1] = pressure_Sensor.getPressure(ADC_4096);

  //Calculate the difference in meters based on the pressure difference
  pressure_data[3] = altitude(pressure_data[1],pressure_data[0]);
}

double altitude(double P, double P0){
  //This function calculates and returns the current altitude in meters based on pressure
  return(44330.0*(1-pow(P/P0,1.0/5.255)));
}

void airq_Setup(){
  //This function sets up the Air Quality Sensor in 1 second read mode, no interrupts
  CCS811Core::status returnCode = air_Sensor.begin();
  if (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    Serial.println(".begin() returned with an error.");
    while (1); //Hang if there was a problem.
  }
  CCS811Core::status setDriveMode(uint8_t mode = 1);
  CCS811Core::status disableInterrupts();
}

void collect_airq(){
  //This function checks if there is data available, and then reads it.  Every tenth reading, it updates
  //the temperature data of the air quality sensor.

  //Increment i counter
  i++;
  
  if(air_Sensor.dataAvailable()){

    //Tell chip to gather results
    air_Sensor.readAlgorithmResults();

    //Collect Results
    airq_data[0] = air_Sensor.getCO2();
    airq_data[1] = air_Sensor.getTVOC();

    //Update Temp data if available.
    if(i >=10){
      CCS811Core::status setEnvironmentalData( float relativeHumidity, float intTemp );
      i=0;
    }
  }
  
}

void collect_humidity(){
  //This function collects humidity data using the interior temperature
  relativeHumidity = humidity_Sensor.getTrueRH(intTemp);
}

void sd_Setup(){
  //This function ensures proper set up of the SD Card.
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  //Print Headers to file
  File dataFile = SD.open("BALLD.txt", FILE_WRITE);

  String headers = "Time ExteriorTemp InteriorTemp Humidity AbsolutePressure RelativePressure Altitude CO2 TVOC ";
  headers += ("BaselinePressue" + String(pressure_data[0]));
  if(dataFile){
    dataFile.println(headers);
    dataFile.close();
    Serial.println("SD Setup complete");
  }
  else{
    Serial.println("Error opening BALLD.txt");
  }
}

void sd_Save(){
 
  String dataString = String((millis()-initialTime)/60000.0) + " " + (String(temperature_c) + " " + String(intTemp) + " " + String(relativeHumidity) + " ");
  
  for(int i=1;i<=3;i++){
    dataString += String(pressure_data[i]);
    dataString += " ";
  }
  
  dataString += (String(airq_data[0]) + " " + String(airq_data[1]));
  
  //Print Data to file
  File dataFile = SD.open("BALLD.txt", FILE_WRITE);

  if(dataFile){
    dataFile.println(dataString);
    dataFile.close();
  }
  else{
    Serial.println("Error Opening File");
  }
}

void intTemp_Collect(){
  AintTemp = analogRead(A22);
  intTemp = ((AintTemp*3.3/1023)-0.750)/0.01 + 25.0;
}

void heater_Setup(){
  //This function sets up the heater pin default on
  pinMode(3,OUTPUT);
  digitalWrite(3,HIGH);
}

void check_Heater(){
  if(intTemp>=20){
    digitalWrite(3,LOW);
  }
  else if(intTemp<=10){
    digitalWrite(3,HIGH);
  }
}

void servo_Setup(){
  Servo1.attach(servoPin);
  servoLoop = EEPROM.read(1);
}

void servo_Run(){

  if(pressure_data[3]>=24384||started){
    started = true;
    if(servoLoop<5){
      Servo1.write(75);
      servoLoop++;
    }
    else{
      Servo1.write(90);
      EEPROM.write(1,servoLoop);
    }
  }
}

