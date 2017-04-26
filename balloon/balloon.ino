#include <SD.h>
#include <OneWire.h> 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Adding SD card logging of the data (26/04/2017)
// Code adapted from https://www.arduino.cc/en/Tutorial/Datalogger - Public domain

//serial object for GPS parser, tx=11 rx=10
SoftwareSerial mySerial(11, 10);
Adafruit_GPS GPS(&mySerial);

//DS18S20 Signal pin on digital 2
int DS18S20_Pin = 4;

//Temperature chip i/o99890------=-=-----------===------------------------=[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[-
OneWire ds(DS18S20_Pin);

// Accelerometer
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
Quaternion q;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/* Barometer start
 SCP1000 Barometric Pressure Sensor Display

 Shows the output of a Barometric Pressure Sensor on a
 Uses the SPI library. For details on the sensor, see:
 http://www.sparkfun.com/commerce/product_info.php?products_id=8161
 http://www.vti.fi/en/support/obsolete_products/pressure_sensors/

 This sketch adapted from Nathan Seidle's SCP1000 example for PIC:
 http://www.sparkfun.com/datasheets/Sensors/SCP1000-Testing.zip

 Circuit:
 SCP1000 sensor attached to pins 6, 7, 10 - 13:
 DRDY: pin 6
 CSB: pin 7
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13

 created 31 July 2010
 modified 14 August 2010
 by Tom Igoe
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>

//Sensor's memory register addresses:
const int PRESSURE = 0x1F;      //3 most significant bits of pressure
const int PRESSURE_LSB = 0x20;  //16 least significant bits of pressure
const int TEMPERATURE = 0x21;   //16 bit temperature reading
const byte READ = 0b11111100;     // SCP1000's read command
const byte WRITE = 0b00000010;   // SCP1000's write command

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int dataReadyPin = 6;
const int chipSelectPin = 7;
// Barometer End

// SD card start
const int chipSelect = 4;
const char* DATA_LOG_PATH = "datalog.txt";
// SD card end

/*
 * Setup
 * GPS - baud and update rate need to be defined
 * Temperature - none required
 * Barometer -
 * Accelerometer -
 */
void setup()  {
  Serial.begin(115200);
  
  // Check if the SD card is present
 boolean present = SD.begin(chipSelect);
 if (!present){
   Serial.print("SD card not found! chipSelect : " + chipSelect);
 }else{
   Serial.print("SD card found and loaded!");
 }
  
  // Barometer Starts
 // start the SPI library:
  SPI.begin();

  // initalize the  data ready and chip select pins:
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);

  //Configure SCP1000 for low noise configuration:
  writeRegister(0x02, 0x2D);
  writeRegister(0x01, 0x03);
  writeRegister(0x03, 0x02);
  // give the sensor time to set up:
  delay(100);
  // Barometer ends
  
  // Accelerometer Starts
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
  // Accelerometer end
  
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
  
  String dataPacket = "";
  
  // Accelerometer start
  // if programming failed, don't try to do anything
    if (!dmpReady) {



      Serial.print("DMP initialisation failed, aborting");

      while(1) {}
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
    // drop the rest of the queue
    mpu.resetFIFO();  
    // Accelerometer end
  
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
  
  //Select High Resolution Mode
  writeRegister(0x03, 0x0A);

  // don't do anything until the data ready pin is high:
//  if (digitalRead() == HIGH) {
  if (1) {
    //Read the temperature data
    int tempData = readRegister(0x21, 2);

    // convert the temperature to celsius and display it:
    float realTemp = (float)tempData / 20.0;
    Serial.print("Temp[C]=");
    Serial.print(realTemp);


    //Read the pressure data highest 3 bits:
    byte  pressure_data_high = readRegister(0x1F, 1);
    pressure_data_high &= 0b00000111; //you only needs bits 2 to 0

    //Read the pressure data lower 16 bits:
    unsigned int pressure_data_low = readRegister(0x20, 2);
    //combine the two parts into one 19-bit number:
    long pressure = ((pressure_data_high << 16) | pressure_data_low) / 4;

    // display the temperature:
    Serial.println("\tPressure [Pa]=" + String(pressure));
  }
  
  boolean logSuccessful = save(dataPacket);
  if (!logSuccessful){
    Serial.println("Couldn't log to file!");
  }
}

boolean save(String dataPacket){
  File sdCard = SD.open(DATA_LOG_PATH,FILE_WRITE);
  if (sdCard){
    sdCard.println(dataPacket);
    sdCard.close();
    return true;
  }else{
    return false;
  }
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

// https://www.arduino.cc/en/Tutorial/BarometricPressureSensor
//Sends a write command to SCP1000

void writeRegister(byte thisRegister, byte thisValue) {

  // SCP1000 expects the register address in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister | WRITE;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}

//Read from or write to register from the SCP1000:
unsigned int readRegister(byte thisRegister, int bytesToRead) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  Serial.print(thisRegister, BIN);
  Serial.print("\t");
  // SCP1000 expects the register name in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the address and the command into one byte
  byte dataToSend = thisRegister & READ;
  Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // return the result:
  return (result);
}
