#include <dht.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include "RTClib.h"

// I used Arduino Mega 2560 for this Data Logger
//DS1307 and BMP280 using pin20(SDA) and pin21(SCL) for I2C communication, both of module are at the same bus

//////////////////////////
//DS1307 use I2C interface
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//////////////////////////

//////////////////////////
//BMP280 use I2C interface
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
//#define BMP280_I2C_ADDRESS  0x77
//////////////////////////

//////////////////////////
dht DHT;
#define DHT11_PIN 2
//////////////////////////

//////////////////////////
//SD card module use SPI interface
const int chipSelect = 53;
//////////////////////////

void setup(){

  Serial.begin(9600);
  Serial.println(F("BMP280 Sensor event test"));
  
   if (! rtc.begin()) {
   Serial.println("Couldn't find RTC");
   while (1);
 }

   if (! rtc.isrunning()) {
   Serial.println("RTC is NOT running!");
   // following line sets the RTC to the date & time this sketch was compiled
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
 }

 
 
  Serial.print("Initializing SD card...");
  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

    if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    
    while (1);
  }
  Serial.println("card initialized.");
  
}

void loop(){
  
  float altitude_ = bmp.readAltitude(1013.25);
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  int chk = DHT.read11(DHT11_PIN);

  //show parameters in serial monitor
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");
  Serial.print("Approx Altitude = ");
  Serial.print(altitude_);
  Serial.println(" m");

   //show data and time in serial monitor
   DateTime now = rtc.now();
   Serial.print(now.year(), DEC);
   Serial.print('/');
   Serial.print(now.month(), DEC);
   Serial.print('/');
   Serial.print(now.day(), DEC);
   Serial.print(" (");
   Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
   Serial.print(") ");
   Serial.print(now.hour(), DEC);
   Serial.print(':');
   Serial.print(now.minute(), DEC);
   Serial.print(':');
   Serial.print(now.second(), DEC);
   Serial.println();

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  //open data file in sd card and write parameters
    if (dataFile) {
      //write data and time
      dataFile.print(now.year(), DEC);
      dataFile.print('/');
      dataFile.print(now.month(), DEC);
      dataFile.print('/');
      dataFile.print(now.day(), DEC);
      dataFile.print(" (");
      dataFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
      dataFile.print(") ");
      dataFile.print(now.hour(), DEC);
      dataFile.print(':');
      dataFile.print(now.minute(), DEC);
      dataFile.print(':');
      dataFile.print(now.second(), DEC);
      dataFile.println();
      dataFile.println("--------------------------------");

      //wite parameters
      dataFile.print("Temperature: ");
      dataFile.print(DHT.temperature);
      dataFile.println(" C");
      dataFile.println("--------------------------------");
      dataFile.print("Humidity: ");
      dataFile.print(DHT.humidity);
      dataFile.println(" %");
      dataFile.println("--------------------------------");
      dataFile.print("Pressure: ");
      dataFile.print(pressure_event.pressure);
      dataFile.println(" hpa");
      dataFile.println("--------------------------------");
      dataFile.print("Approx Altitude: ");
      dataFile.print(altitude_);
      dataFile.println(" m");
      dataFile.println("--------------------------------");
      dataFile.println("/////////////////////////////////////////////");
      dataFile.println("/////////////////////////////////////////////");
      dataFile.close();

  }
  // if the file isn't open, show this error in serial monitor:
  else {
    Serial.println("error opening datalog.txt");
  }

  delay(3000);
  
}
