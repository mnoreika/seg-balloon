#include <string.h>
#include <util/crc16.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "OneWire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define RADIOPIN 13

char datastring[80];

//DS18S20 Signal pin on digital 2
int DS18S20_Pin = 4;

//Temperature chip i/o
OneWire ds(DS18S20_Pin);
MPU6050 mpu;
Quaternion q;
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

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

/*
 * Setup
 * Temperature - none required
 * Accelerometer - initialise i2c, gyros and packet size
 */
void setup()  {
  Serial.begin(9600);

  // Accelerometer Starts

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Initialize device
  mpu.initialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // Turn on the DMP, now that it's ready
  mpu.setDMPEnabled(true);

  // Enable Arduino interrupt detection
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  // Accelerometer end
  
  // Transmission setup
  pinMode(RADIOPIN,OUTPUT);
}

/*
 * Loop
 * Temperature - No parser so local conversion is required then print asap
 * Accelerometer -
 */
void loop() {

  String dataPacket;
  // Structure of the packet:
  // x,y,z,temp,checksum

  //------ accelerometer ------//
  dataPacket += getAcceleration();
  
  //------ temperature ------//
  float temperature = getTemp(); //will take about 750ms to run
  char buff[10];
  sprintf(buff, "%d,", (int)(temperature * 1000));
  dataPacket += String(buff);

  // Wire.write returns false if the data couldn't be sent
  char dataOut[64];
  dataPacket.toCharArray(dataOut, dataPacket.length());

  char checksum_str[6];

  strcat(datastring, dataOut);
  Serial.println(dataOut);
  //unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  //sprintf(checksum_str, "*%04X\n", CHECKSUM);
  //strcat(datastring,checksum_str);
  //rtty_txstring (datastring);
  //delay(2000);
}

String getAcceleration(){
  String accel = String("");
  char buff[64];

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // drop the rest of the queue
  mpu.resetFIFO();
  
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

//  sprintf(buff, "%d,", (int)(aaWorld.x * 1000));
  sprintf(buff, "%d,", (int)(q.x * 1000));
  accel += String(buff);
    
  sprintf(buff, "%d,", (int)(aaWorld.y * 1000));
  accel += String(buff);
    
  sprintf(buff, "%d,", (int)(aaWorld.z * 1000));
  accel += String(buff);

  return accel;
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


void rtty_txstring (char * string)
{

  /* Simple function to sent a char at a time to
     ** rtty_txbyte function.
    ** NB Each char is one byte (8 Bits)
    */

  char c;

  c = *string++;

  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}


void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to
    ** rtty_txbit function.
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and
    ** proceded with a 1. 0 = Start bit; 1 = Stop bit
    **
    */

  int i;

  rtty_txbit (0); // Start bit

  // Send bits for for char LSB first

  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1);

    else rtty_txbit(0);

    c = c >> 1;

  }

  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    digitalWrite(RADIOPIN, HIGH);
  }
  else
  {
    // low
    digitalWrite(RADIOPIN, LOW);

  }

  //                  delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
  delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
                            // largest value that will produce an accurate delay is 16383
                            // See : http://arduino.cc/en/Reference/DelayMicroseconds

}

uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}
