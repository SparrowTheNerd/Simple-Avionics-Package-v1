#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 21
#define RFM95_INT digitalPinToInterrupt(2)
#define RFM95_RST 1

RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct packet {
  float w,i,j,k,vX,vY,vZ,X,Y,Z;
} dataPacket;

void setup() {
  pinMode(23,OUTPUT); digitalWrite(23,HIGH);
  pinMode(22,OUTPUT); digitalWrite(22,HIGH);

  Serial.begin(115200);
  // while (!Serial) delay(1); // Wait for Serial Console (comment out line if no computer)

  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  SPI.begin();
  if (!rf95.init())
    Serial.println("init failed");  
  else {Serial.println("init success");}
  rf95.setFrequency(915.0); //set frequency to 915MHz
  rf95.setTxPower(20,false); //set the transmit power to 20dBm using PA_BOOST
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // You can change the modulation parameters with eg
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
}

void loop() {
 if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[sizeof(dataPacket)];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      memcpy(&dataPacket,buf,sizeof(dataPacket));

      Serial.print(dataPacket.w,5); Serial.print(" \t"); Serial.print(dataPacket.i,5); Serial.print(" \t"); Serial.print(dataPacket.j,5); Serial.print(" \t"); Serial.println(dataPacket.k,5);
      // Serial.print("RSSI: ");
      // Serial.println(rf95.lastRssi(), DEC);

    } else {
      Serial.println("Receive failed");
    }
  }
}