#include <OneWire.h> 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//serial object for GPS parser, tx=11 rx=10
SoftwareSerial mySerial(11, 10);
Adafruit_GPS GPS(&mySerial);

//DS18S20 Signal pin on digital 2
int DS18S20_Pin = 2;

//Temperature chip i/o
OneWire ds(DS18S20_Pin);

/*
 * Setup
 * GPS - baud and update rate need to be defined
 * Temperature - none required
 * Barometer -
 * Accelerometer -
 */
void setup()  {
  
  Serial.begin(115200);
  Serial.println("SEG Component test!");

  //GPS functioning baud rate
  GPS.begin(9600);
  
  //1hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}

//timer to handle data loop for gps
uint32_t timer = millis();


/*
 * Loop 
 * GPS - Read and verify nmea sentence, then print every 2 seconds
 * Temperature - No parser so local conversion is required then print asap
 * Barometer -
 * Accelerometer -
 */
void loop() {

  //------ temperature ------//
  
  float temperature = getTemp(); //will take about 750ms to run
  Serial.println(temperature);

  //------ gps ------//
  
  GPS.read();
  
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  //if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  //every 2 seconds
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    printGPS();
  }

  //------ barometer ------//
  
  
}


/*
 * Temperature
 */
float getTemp() {
 //returns the temperature from one DS18S20 in DEG Celsius

 byte data[12];
 byte addr[8];

 if ( !ds.search(addr)) {
   //no more sensors on chain, reset search
   ds.reset_search();
   return -1000;
 }

 if ( OneWire::crc8( addr, 7) != addr[7]) {
   Serial.println("CRC is not valid!");
   return -1000;
 }

 if ( addr[0] != 0x10 && addr[0] != 0x28) {
   Serial.print("Device is not recognized");
   return -1000;
 }

 ds.reset();
 ds.select(addr);
 ds.write(0x44,1); // start conversion, with parasite power on at the end
 
 delay(750); // Wait for temperature conversion to complete

 byte present = ds.reset();
 ds.select(addr);  
 ds.write(0xBE); // Read Scratchpad

 
 for (int i = 0; i < 9; i++) { // we need 9 bytes
  data[i] = ds.read();
 }
 
 ds.reset_search();
 
 byte MSB = data[1];
 byte LSB = data[0];

 float tempRead = ((MSB << 8) | LSB); //using two's compliment
 float TemperatureSum = tempRead / 16;
 
 return TemperatureSum;
 
}


/*
 * GPS
 */
void printGPS() {

  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
  if (GPS.fix) {
    Serial.print("Location: ");  
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", "); 
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Location (in degrees, works with Google Maps): ");
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(", "); 
    Serial.println(GPS.longitudeDegrees, 4);
      
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
  }
    
}

