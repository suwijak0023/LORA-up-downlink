//#include "sensor/HDC1080.h"
#include "esp32-hal-adc.h"

//HDC1080 hdc1080;
//
//static void prepareTxFrame( uint8_t port )
//{
//  appDataSize = 4;//AppDataSize max value is 64 ( src/Commissioning.h -> 128 )
//  appData[0] = 0x74;
//  appData[1] = 0x65;
//  appData[2] = 0x73;
//  appData[3] = 0x74;
//}

static void Txup_Send_Button_On( uint8_t port )
{
  num++;
  //  Serial.println();
  Serial.print(num);
  Serial.print(":");
appDataSize = 10;//AppDataSize max value is 64 ( src/Commissioning.h -> 128 )
  appData[0] = 0x42;
  appData[1] = 0x54;
  appData[2] = 0x44;
  appData[3] = 0x65;
  appData[4] = 0x76;
  appData[5] = 0x4F;
  appData[6] = 0x6E;
  appData[7] = 0x65;
  appData[8] = 0x4F;
  appData[9] = 0x6E;
//TEXT TO HEX ONLY  NOT CONVERT NUMBER TO HEX         
}

//static void Tx_Sensor_HDC1080( uint8_t port )
//{
//  pinMode(Vext, OUTPUT);
//  digitalWrite(Vext, LOW);
//  uint16_t batteryVoltage = analogRead(37) * 3.046;
//  hdc1080.begin(0x40);
//  float temperature = (float)(hdc1080.readTemperature());
//  float humidity = (float)(hdc1080.readHumidity());
//  hdc1080.end();
//  digitalWrite(Vext, HIGH);
//  unsigned char *puc;
//
//
//  puc = (unsigned char *)(&temperature);
//  appDataSize = 10;
//  appData[0] = puc[0];
//  appData[1] = puc[1];
//  appData[2] = puc[2];
//  appData[3] = puc[3];
//
//  puc = (unsigned char *)(&humidity);
//  appData[4] = puc[0];
//  appData[5] = puc[1];
//  appData[6] = puc[2];
//  appData[7] = puc[3];
//
//  appData[8] = (uint8_t)(batteryVoltage >> 8);
//  appData[9] = (uint8_t)batteryVoltage;
//
//  Serial.print("T=");
//  Serial.print(temperature);
//  Serial.print("C, RH=");
//  Serial.print(humidity);
//  Serial.print("%,");
//  Serial.print("BatteryVoltage:");
//  Serial.println(batteryVoltage);
//}


//static void Tx_Send_DHT11( uint8_t port )
//{
//    // Prepare upstream data transmission at the next possible time.
//    uint32_t humidity = dht.readHumidity(false) * 100;
//    uint32_t temperature = dht.readTemperature(false) * 100;
//  
//    appDataSize = 4;//AppDataSize max value is 64
//    // Format the data to bytes 
//    appData[0] = highByte(humidity);
//    appData[1] = lowByte(humidity);
//    appData[2] = highByte(temperature);
//    appData[3] = lowByte(temperature);
//  
//}
