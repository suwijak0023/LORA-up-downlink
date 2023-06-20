#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char auth[] = "xxxxxxxxx"; //blynk auth code

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "xxx xxxx";
char pass[] = "xxxx";

float lat;
float lon;



// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

//OLED pins to ESP32 0.96OLEDGPIOs 
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16

SSD1306  display(0x3c, 4, 15);

#define SS      18
#define RST     14
#define DI0     26
#define BAND    433.175E6  //915E6

WidgetMap myMap(V1);


BLYNK_WRITE(V0) {
  GpsParam gps(param);

  // Print 6 decimal places for Lat, Lon


  lat = String(gps.getLat(),6).toFloat();
  lon = String(gps.getLon(),6).toFloat();
  
  Serial.print("Lat: ");
  Serial.println(lat,7);
 

  Serial.print("Lon: ");
  Serial.println(lon,7);

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

  Serial.println("LoRa Receiver");

  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);

  }
  Serial.println("LoRa Initial OK!");

  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  Blynk.begin(auth, ssid, pass);

  myMap.clear();
  
}


void loop() {
  String tmp_string, tmp_rssi;
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      //Serial.print((char)LoRa.read());
      tmp_string += (char)LoRa.read();      
    }

    Serial.print(tmp_string);
    
    tmp_rssi = LoRa.packetRssi();
    
    // print RSSI of packet
    
    
    Serial.println("' with RSSI " + tmp_rssi);
    
    //Serial.println(LoRa.packetRssi());

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(10, 0, "Received:");
    display.drawString(10, 15, tmp_string+" RSSI: "+tmp_rssi);
    display.drawString(10, 30, String(lat,7));
    display.drawString(10, 45, String(lon,7));
    // write the buffer to the display
    display.display();

    //BLYNK_WRITE(V0);
    Blynk.syncVirtual(V0);
    myMap.location(1, lat, lon, "value");
    
  }
  tmp_string ="";
  tmp_rssi = "";

  Blynk.run();
  
}
