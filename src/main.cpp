#include <Arduino.h>
#include <./lib/SparkFunLSM6DSO.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Dps368.h>

#define DATA_RATE     104
#define DATA_PERIOD   1.f/104.f
#define SQRT_2        0.70710678118f
#define SD_CS         22
#define FILENAME      "testcode.csv"
#define BZR           15

LSM6DSO IMU;
fifoData myFifo; //This will hold our FIFO data
int availableBytes = 0;

File dataLog;
uint32_t startTime;
float prevTimeStamp;

Dps368 Dps368PressureSensor = Dps368();

float xGyOfst = 0;
float yGyOfst = 0;
float zGyOfst = 0;

float *xGyro, *yGyro, *zGyro, *xAccel, *yAccel, *zAccel, *timeStamp, *vMag;
void getVectors(int *length);
void clearMem();
void writeToSD(int length);
void gyroCalibrate();

void setup()
{
  Serial.begin(115200);

  SD.begin(SD_CS);
  dataLog = SD.open(FILENAME, O_CREAT | O_TRUNC | O_WRITE);   //create file if doesn't exist, delete all data in it if it exists, and write data to it
  dataLog.close();

  Wire.begin();
  Wire.setClock(400000);  //set i2c bus to 400khz

  IMU.begin();
  IMU.initialize(FIFO_SETTINGS);
  IMU.setFifoMode(FIFO_MODE_DISABLED);
  IMU.setAccelDataRate(DATA_RATE);
  IMU.setGyroDataRate(DATA_RATE);
  IMU.setAccelBatchDataRate(DATA_RATE);
  IMU.setGyroBatchDataRate(DATA_RATE);
  IMU.setAccelRange(32);
  IMU.setGyroRange(2000);

  tone(BZR,2300,250);
  delay(500);
  tone(BZR,2300,250);

  delay(1000);
  gyroCalibrate();    //pre-flight gyro calibration
  delay(100);
  startTime = millis();
  IMU.setFifoMode(FIFO_MODE_CONTINUOUS);
}

int flightEvent = 0;
/*
0: On the pad
1: Motor burn
2: Coast phase --> apogee detect
3: Chute Deploy
4: Landed
*/

void loop() {
  int arraySize;
  getVectors(&arraySize);
  
  if (flightEvent > 0) {
    writeToSD(arraySize);
    Serial.println("writing");
  }
  clearMem();
  delay(50);
}

void getVectors(int *length) {
  int j = 0;
  availableBytes = IMU.getFifoStatus();  //Check for data in FIFO
  //dynamically allocate memory space for each array of int16s using malloc()
  xGyro = static_cast<float *>(malloc(availableBytes * sizeof(float))); yGyro = static_cast<float *>(malloc(availableBytes * sizeof(float))); zGyro = static_cast<float *>(malloc(availableBytes * sizeof(float)));
  xAccel = static_cast<float *>(malloc(availableBytes * sizeof(float))); yAccel = static_cast<float *>(malloc(availableBytes * sizeof(float))); zAccel = static_cast<float *>(malloc(availableBytes * sizeof(float)));
  timeStamp = static_cast<float *>(malloc(availableBytes * sizeof(float)));
  timeStamp[0] = prevTimeStamp;
  if (availableBytes > 5) {
    for(int i = availableBytes; i > 5; i--) {
      myFifo = IMU.fifoRead();
      //do some quick trig to convert diagonal IMU values into straight coordinate values
      if( myFifo.fifoTag == ACCELEROMETER_DATA) {
        xAccel[j/2] = ((float)myFifo.yAccel * SQRT_2 - (float)myFifo.xAccel * SQRT_2)*0.000976f;
        xAccel[j/2] = ((float)myFifo.yAccel * SQRT_2 - (float)myFifo.xAccel * SQRT_2)*0.000976f;
        yAccel[j/2] = ((float)myFifo.yAccel * SQRT_2 + (float)myFifo.xAccel * SQRT_2)*0.000976f;
        zAccel[j/2] = myFifo.zAccel * 0.000976f;
      }

      if( myFifo.fifoTag == GYROSCOPE_DATA) {
        xGyro[j/2] = ((float)myFifo.xGyro) * 0.07f - xGyOfst;
        yGyro[j/2] = ((float)myFifo.yGyro) * 0.07f - yGyOfst;
        zGyro[j/2] = ((float)myFifo.zGyro) * 0.07f - zGyOfst;
      }

      timeStamp[(j/2)+1] = timeStamp[j/2] + (DATA_PERIOD);    //create timestamp based on approximate time per datapoint (batchDataRate of IMU)

      switch(flightEvent) {
        case 0:
          float magnitude = sqrtf((xAccel[j/2]*xAccel[j/2])+(yAccel[j/2]*yAccel[j/2])+(zAccel[j/2]*zAccel[j/2]));
          if (magnitude >= 2.5f) {    //if acceleration passes threshold, set flight status to 1
            flightEvent = 1;
            Serial.println("launch detect!");
          }
        case 1:
          float magnitude = sqrtf((xAccel[j/2]*xAccel[j/2])+(yAccel[j/2]*yAccel[j/2])+(zAccel[j/2]*zAccel[j/2]));
          if (magnitude <= 0) {     //if acceleration drops negative (air drag), set flight status to 2
            flightEvent = 2;
            Serial.println("coasting");
          }
      }

      j++;
    }
    prevTimeStamp = timeStamp[j/2];
  }
  *length = floor(j/2);
}

void clearMem() {
  delete[] xGyro;
  delete[] yGyro;
  delete[] zGyro;
  delete[] xAccel;
  delete[] yAccel;
  delete[] zAccel;
  delete[] timeStamp;
}

void writeToSD(int size) {
  dataLog = SD.open(FILENAME,FILE_WRITE);
  for(int i = 0; i < size; i++) {
    dataLog.print(timeStamp[i],3);
    dataLog.print(",");
    dataLog.print(xGyro[i],3);
    dataLog.print(",");
    dataLog.print(yGyro[i],3);
    dataLog.print(",");
    dataLog.print(zGyro[i],3);
    dataLog.print(",");
    dataLog.print(xAccel[i],3);
    dataLog.print(",");
    dataLog.print(yAccel[i],3);
    dataLog.print(",");
    dataLog.println(zAccel[i],3);
  }
  dataLog.close();
}

void gyroCalibrate() {
  IMU.setFifoMode(FIFO_MODE_CONTINUOUS);
  delay(250);
  int fullLength = 0;
  float xO = 0;
  float yO = 0;
  float zO = 0;
  for (int i=0; i < 30; i++) {
    int length;
    getVectors(&length);
    delay(100);
    for (int i=0; i < length; i++) {
      xO += xGyro[i];
      yO += yGyro[i];
      zO += zGyro[i];
    }
    fullLength += length;
    clearMem();
  }
  xGyOfst = xO/(float)fullLength;
  yGyOfst = yO/(float)fullLength;
  zGyOfst = zO/(float)fullLength;
  IMU.setFifoMode(FIFO_MODE_DISABLED);
  tone(BZR,2300,250);
  delay(100);
  tone(BZR,3000,250);
  delay(100);
  tone(BZR,3700,250);
  prevTimeStamp = 0;    //reset previous timestamp before starting the actual flight code
}