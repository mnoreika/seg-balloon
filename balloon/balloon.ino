#include <Wire.h>

// Barometer code adapted from
// https://www.arduino.cc/en/Tutorial/BarometricPressureSensor

#include <OneWire.h> // Used for communications with the temperature sensor
#include <Adafruit_GPS.h> // Used for communications with the GPS
#include <SoftwareSerial.h> // Used for communications with the GPS
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h> // Used for communications with the barometer
#include <SD.h> // Used for saving to the SD card


//serial object for GPS parser, tx=11 rx=10
SoftwareSerial mySerial(11, 10);
Adafruit_GPS GPS(&mySerial);
File dataLogFile;

//DS18S20 Signal pin on digital 2
int DS18S20_Pin = 4;

bool sdCardUsable;

//Temperature chip i/o
OneWire ds(DS18S20_Pin);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
Quaternion q;

// MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

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

/*
 * Setup
 * GPS - baud and update rate need to be defined
 * Temperature - none required
 * Barometer - initialise pins and configure for low noise
 * Accelerometer - initialise i2c, gyros and packet size
 */
void setup()  {
  // Serial.begin(115200);

  // Barometer Starts
  // start the SPI library:
  SPI.begin();
  Wire.begin(8);
  Wire.onRequest(requestEvent);

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

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Initialize device
  mpu.initialize();

  // load and configure the DMP. Returns 0 if it was.
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // Check if the DMP was successfully loaded and configured. devStatus == 0 if it was
    // Turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
//    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  // Accelerometer end

  // Gps start
  //GPS functioning baud rate
  GPS.begin(9600);

  //1hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  // Gps end
  // SD start
  sdCardUsable = SD.begin(4);


  // SD end
  delay(1000);
}

//timer to handle data loop for gps
//uint32_t timer = millis();

/*
 * Loop
 * GPS - Read and verify nmea sentence, then print every 2 seconds
 * Temperature - No parser so local conversion is required then print asap
 * Barometer -
 * Accelerometer -
 */
void loop() {

  String dataPacket;

  //------ accelerometer ------//
  String acceleration = getAcceleration();
  //------ temperature ------//
  float temperature = getTemp(); //will take about 750ms to run
  //------ gps ------//
  String gpsStr = getGps();
  //------ barometer ------//
  writeRegister(0x03, 0x0A);  //Select High Resolution Mode
  // String barometerTemp = getBarometerTemp();
  String barometerPres = getBarometerPres();

  //------ Create the data packet ------//
  char buff[10];
  sprintf(buff, "%f", temperature);

  dataPacket += acceleration;
  dataPacket += String(buff);
  dataPacket += gpsStr;
  dataPacket += barometerPres;

  if (dataLogFile) {
    dataLogFile.println(dataPacket);
  }

  // Wire.write returns false if the data couldn't be sent
  char* dataOut;
  dataPacket.toCharArray(dataOut, dataPacket.length());
  boolean success = Wire.write(dataOut);

//  if(!success){
//    Serial.println("Couldn't send data to transmitter!");
//  }
}

// Get the GPS output
String getGps(){
  GPS.read();

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return "";
  }

  //if millis() or timer wraps around, we'll just reset it
//  if (timer > millis())  timer = millis();

//  //Only return every 2 seconds
//  if (millis() - timer > 2000) {
  //  timer = millis(); // reset the timer
    if (GPS.fix) {
      char buff[10];
      String gpsStr = String("");
      
      sprintf(buff, "%f", GPS.latitudeDegrees);
      gpsStr += String(buff) + ",";
      
      sprintf(buff, "%f", GPS.longitudeDegrees);
      gpsStr += String(buff);
      return gpsStr;
    }else{
      return "0,0";
    }
//  }
}

void writeToFile (String text) {
  if (sdCardUsable){
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile){
      dataFile.println(text);
      dataFile.close();
    }
  }
}

String getAcceleration(){
  String accel = String("");
  char buff[10];
  //only if dmp is working
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    accel += String("w");
    sprintf(buff, "%f", q.w);
    accel += String(buff);
    
    accel += String("x");
    sprintf(buff, "%f", q.x);
    accel += String(buff);
    
    accel += String("y");
    sprintf(buff, "%f", q.y);
    accel += String(buff);
    
    accel += String("z");
    sprintf(buff, "%f", q.z);
    accel += String(buff);

    // drop the rest of the queue
    mpu.resetFIFO();

    return accel;
}

/*
String getBarometerTemp() {
 
 // don't do anything until the data ready pin is high:
 if (digitalRead(dataReadyPin) == HIGH) {
 //Read the temperature data
 int tempData = readRegister(0x21, 2);
 
 // convert the temperature to celsius and display it:
 float realTemp = (float)tempData / 20.0;
 //Serial.print("Temp[C]=");
 //Serial.print(realTemp);
 
 return String(realTemp);
 }
 return
 }
 */
// Get the barometer pressure
String getBarometerPres() {
  // don't do anything until the data ready pin is high:
  if (digitalRead(dataReadyPin) == HIGH) {

    //Read the pressure data highest 3 bits:
    byte  pressure_data_high = readRegister(0x1F, 1);
    pressure_data_high &= 0b00000111; //you only needs bits 2 to 0

    //Read the pressure data lower 16 bits:
    unsigned int pressure_data_low = readRegister(0x20, 2);
    //combine the two parts into one 19-bit number:
    long pressure = ((pressure_data_high << 16) | pressure_data_low) / 4;

    return String(pressure);
  }
  return "NoPressure";
}

// Returns the temperature from one DS18S20 in DEG Celsius
float getTemp() {

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    //Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    //Serial.print("Device is not recognized");
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

// Barometer code adapted from
// https://www.arduino.cc/en/Tutorial/BarometricPressureSensor
// Sends a write command to SCP1000
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
  //Serial.print(thisRegister, BIN);
  //Serial.print("\t");
  // SCP1000 expects the register name in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the address and the command into one byte
  byte dataToSend = thisRegister & READ;
  //Serial.println(thisRegister, BIN);
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

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
// This is used for transmission between ardunios
void requestEvent() {
  Wire.write("seg "); // respond with message of 6 bytes
  // as expected by master
}

