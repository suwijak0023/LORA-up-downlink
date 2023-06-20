#define LEDPin 25
#define LEDredPin 12
#define LockerPin 13

void app(uint8_t data)
{
  lora_printf("data:%d\r\n", data);

  switch (data)
  {
    case 76:
      {
        pinMode(LEDredPin, OUTPUT);
        digitalWrite(LEDredPin, LOW);
        delay(5000);
        pinMode(LEDPin, OUTPUT);
        digitalWrite(LEDPin, HIGH);
        delay(5000);
        pinMode(LockerPin, OUTPUT);
        digitalWrite(LockerPin, HIGH);
        delay(5000);
        
        digitalWrite(LEDredPin, HIGH);
        digitalWrite(LEDPin, LOW);
        digitalWrite(LockerPin, LOW);
        Serial.println("Status : START DEVICE 1");
//        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case 85:
      {
        pinMode(LEDredPin, OUTPUT);
        digitalWrite(LEDredPin, LOW);
        delay(5000);
        pinMode(LEDPin, OUTPUT);
        digitalWrite(LEDPin, HIGH);
        delay(5000);
        pinMode(LockerPin, OUTPUT);
        digitalWrite(LockerPin, HIGH);
        delay(5000);
        
        digitalWrite(LEDredPin, HIGH);
        digitalWrite(LEDPin, LOW);
        digitalWrite(LockerPin, LOW);
        Serial.print("Status : UNLOCK DEVICE 1 ");
        break;
      }
    case 51:
      {
        break;
      }
    default:
      {
        break;
      }
  }
}


//void  downLinkDataHandle(McpsIndication_t *mcpsIndication)
//{
//  lora_printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
//  lora_printf("+REV DATA:");
//    app(mcpsIndication->Buffer[0]);
//
//  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
//  {
//    lora_printf("%02X",mcpsIndication->Buffer[i]);
//  }
//  lora_printf("\r\n");
//}


//full+data
String LoRa_data;
uint16_t num = 0;
bool LoRaDownLink = false;
uint32_t LoRadonwlinkTime;

void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  LoRa_data = "";
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA:");
  app(mcpsIndication->Buffer[0]);
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++)
  {
//    Serial.printf("%d", mcpsIndication->Buffer[i]);
    LoRa_data = LoRa_data + (String)(char)mcpsIndication->Buffer[i];
  }
  lora_printf("\r\n");
  LoRaDownLink = true;
  LoRadonwlinkTime = millis();
  num++;
  Serial.println();
  //  Serial.print(num);
  //  Serial.print(":");
  Serial.println(LoRa_data);

  Display.init();
  Display.clear();
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 22, LoRa_data);
  Display.display();
}
