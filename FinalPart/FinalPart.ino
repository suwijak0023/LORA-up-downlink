#include <ESP32_LoRaWAN.h>
#include "Arduino.h"
#include "downlink.h"
#include "uplink.h"
#include "button.h"


uint32_t  license[4] = {0x7353D08A, 0xFD73AD45, 0x900DB6F3, 0xD422CE24};

/* OTAA para*/
uint8_t DevEui[] = { 0x0C, 0xBA, 0xBC, 0x47, 0xC4, 0xE9, 0x6C, 0x9A };  //CHIRSTACK
uint8_t AppEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppKey[] = { 0x84, 0x16, 0x54, 0x87, 0x98, 0x36, 0x95, 0x6B, 0x16, 0xA7, 0xE9, 0x3D, 0xCF, 0x00, 0x20, 0xA3 };

/* ABP para*/
uint8_t NwkSKey[] = { 0xE6, 0xE4, 0x39, 0xDE, 0xD0, 0x31, 0x19, 0x82, 0x3F, 0xAB, 0x3B, 0xE0, 0x83, 0x33, 0x7F, 0x23 };
uint8_t AppSKey[] = { 0x31, 0x81, 0xA0, 0xDB, 0x39, 0x3E, 0x4A, 0x43, 0x3B, 0x6B, 0xD8, 0x20, 0x44, 0xC2, 0xE5, 0x0A };
uint32_t DevAddr =  ( uint32_t )0x002A9D49;  //CHIRSTACK

///* OTAA para*/
//uint8_t DevEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0x4F, 0x30 };
//uint8_t AppEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //THE THING
//uint8_t AppKey[] = { 0x20, 0x62, 0x61, 0x61, 0x05, 0xA1, 0x9D, 0x56, 0xA0, 0x47, 0xCE, 0x1D, 0x9A, 0x29, 0x9F, 0x2C };
//
///* ABP para*/
//uint8_t NwkSKey[] = { 0xBF, 0xCC, 0x70, 0x1A, 0x6A, 0x10, 0xE8, 0x2D, 0x8D, 0xC0, 0x15, 0x94, 0x28, 0x46, 0x04, 0x7A };
//uint8_t AppSKey[] = { 0x9F, 0x0E, 0xEB, 0x7D, 0xEB, 0x9B, 0xEF, 0x61, 0x2D, 0xB4, 0xB0, 0x42, 0xB9, 0x20, 0x18, 0xCA };
//uint32_t DevAddr =  ( uint32_t )0x260B17C4;  //THE THING


/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

DeviceClass_t  loraWanClass = CLASS_C;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
  None : print basic info.
  Freq : print Tx and Rx freq, DR info.
  Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
  Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

// Add your initialization code here



void setup()
{

  if (mcuStarted == 0)
  {
    LoRaWAN.displayMcuInit();
  }
  Serial.begin(115200);
  pinMode(button, INPUT_PULLUP);
  while (!Serial);
  SPI.begin(SCK, MISO, MOSI, SS);
  Mcu.init(SS, RST_LoRa, DIO0, DIO1, license);

  adcAttachPin(37);
  analogSetClockDiv(255); // 1338mS

  //  deviceState = DEVICE_STATE_INIT;


#if(LORAWAN_DEVEUI_AUTO)
  LoRaWAN.generateDeveuiByChipID();
#endif
  LoRaWAN.init(loraWanClass, loraWanRegion);

  deviceState = DEVICE_STATE_JOIN;
}
void loop() {
  
  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
#if(LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        LoRaWAN.displayJoining();
        break;
      }
    case DEVICE_STATE_SEND:
      {
//        LoRaWAN.displaySending();
//        Txup_Send_Button_On( appPort );
//       //        prepareTxFrame2( appPort );
        LoRaWAN.send(loraWanClass);
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
//        LoRaWAN.displayAck();
        LoRaWAN.sleep(loraWanClass, debugLevel);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
//        app;
        break;
      }
  }
}

//  if(digitalRead(button)==true){
//    status=!status;
//    Serial.println(status);
////    digitalWrite(led, status);
//    }
//    while(digitalRead(button==true));
//    delay(50);
//    if (status == 1) {
////    digitalWrite(led, HIGH);
////    prepareTxFrame( appPort );
//        prepareTxFrame2( appPort );
//displayInof();
//    delay(50);
//  }
//  else{
////    digitalWrite(led, LOW);
//    }
//  }
