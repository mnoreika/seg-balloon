#include <OneWire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <SD.h>

//serial object for GPS parser, tx=11 rx=10
SoftwareSerial mySerial(11, 10);
Adafruit_GPS GPS(&mySerial);

//DS18S20 Signal pin on digital 2
int DS18S20_Pin = 4;

//Temperature chip i/o
OneWire ds(DS18S20_Pin);

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

// SD card start
const int chipSelect = 4;
const char* DATA_LOG_PATH = "datalog.txt";

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
  Serial.begin(115200);

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

  #if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    ////Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    ////Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

  // Accelerometer end

  //Serial.println("SEG Component test!");

  //GPS functioning baud rate
  GPS.begin(9600);

  //1hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  // Check if the SD card is present
  boolean present = SD.begin(chipSelect);
  if (!present) {
    //Serial.print("SD card not found! chipSelect : " + chipSelect);
  } else{
      //Serial.print("SD card found and loaded!");
  }

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

  //------ accelerometer ------//

  //only if dmp is working
    if (dmpReady) {

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      float dmpW = q.w;
      float dmpX = q.x;
      float dmpY = q.y;
      float dmpZ = q.z;

      // drop the rest of the queue
      mpu.resetFIFO();
    }

  //------ temperature ------//

  float temperature = getTemp(); //will take about 750ms to run
  //Serial.println(temperature);

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

    if (GPS.fix) {
      float lat = GPS.latitudeDegrees;
      float lng = GPS.longitudeDegrees;
    }
  }

  //------ barometer ------//

  //Select High Resolution Mode
  writeRegister(0x03, 0x0A);

  String barometerTemp = getBarometerTemp();
  String barometerPres = getBarometerPres();

  boolean logSuccessful = save(dataPacket);
  if (!logSuccessful){
     //Serial.println("Couldn't log to file!");
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

}

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

    // display the temperature:
    //Serial.println("\tPressure [Pa]=" + String(pressure));

    return String(pressure);

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
