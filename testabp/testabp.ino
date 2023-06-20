#include <Arduino.h>
#include <stdlib.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xD2, 0x13, 0x33, 0xF4, 0xB4, 0xAA, 0xA0, 0x50, 0xD0, 0x61, 0xAE, 0xD4, 0x8F, 0xDF, 0x95, 0x86 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x8C, 0x3E, 0x03, 0xAC, 0xA4, 0xCC, 0xD5, 0xAC, 0x22, 0xD8, 0x0B, 0xF1, 0xE7, 0x8E, 0x48, 0x7F };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260BE5D2 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

SSD1306  display(0x3c, 4, 15); //heltec
#define SCK     5    // GPIO5  -- heltec
#define MISO    19   // GPIO19 -- heltec
#define MOSI    27   // GPIO27 -- heltec
#define SS      18   // GPIO18 -- heltec
#define RST     14   // GPIO14 -- heltec
#define DI0     26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

//SSD1306  display(0x3c, SDA, SCL); //sparkfun
//#define SCK     14    // GPIO5  -- sparkfun
//#define MISO    12   // GPIO19 -- sparkfun
//#define MOSI    13   // GPIO27 -- sparkfun
//#define SS      16   // GPIO18 -- sparkfun
//#define RST     27   // GPIO14 -- sparkfun
//#define DI0     26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

//#define DIO1 35  // GPIO 33
//#define DIO2 34  // GPIO 32
//#define BAND    915E6  //433.175E6
//#define BAND 433E6 //you can set band here directly,e.g. 868E6,915E6
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = SS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RST,
  .dio = {DI0, 3, 4},
  //  .dio = {DI0, DIO1, DIO2},
};


//#define RedledPin = 12;
//#define GreenledPin = 13;
//#define LOCK  27


void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    //    display.setFont(ArialMT_Plain_16);
    display.drawString(10, 5, "SEND STATUS : FAIL");

    // write the buffer to the display
    display.display();
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    //    display.setFont(ArialMT_Plain_16);
    display.drawString(10, 5, "SEND STATUS : PASS");
    //    display.drawString(10, 20, "Message = " + String(mydata[0]));
    display.drawString(10, 20, "Message = ");

    // write the buffer to the display
    display.display();

    //    delay(3000);
    //    display.clear();
    //    display.setFont(ArialMT_Plain_24);
    //    display.setTextAlignment(TEXT_ALIGN_LEFT);
    //    display.drawString(0, 24, "COMPLETE");
    //    display.display();
    //    display.clear();



  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  pinMode(25, OUTPUT);
  SPI.begin(SCK, MISO, MOSI, SS);

  Serial.println(F("Starting"));

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  Serial.println("LoRa Sender");
  SPI.begin(5, 19, 27, 18);

  Serial.println("LoRa Initial OK!");
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  //    #if defined(CFG_eu433)
//#if defined(CFG_sx1276_radio)

//#if defined(CFG_eu868)
//  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
//  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this

  //  // frequency is not configured here.
//  #elif defined(CFG_as923)
      LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 923600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 923800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 924000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 924200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 924400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 924600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 924500000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

//  #else
//    # error Region not supported
//    #endif
//#elif defined(CFG_us915)
////  //  // NA-US channels 0-71 are configured automatically
////  //  // but only one group of 8 should (a subband) should be active
////  //  // TTN recommends the second sub band, 1 in a zero based count.
////  //  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
//  LMIC_selectSubBand(1);
//#endif
  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}


void loop() {
  //  Serial.print("Sending packet: ");
//  unsigned long now;
//    now = millis();
//    if ((now & 512) != 0) {
//      digitalWrite(13, HIGH);
//    }
//    else {
//      digitalWrite(13, LOW);
//    }
    os_runloop_once();
}
//  os_runloop_once();
  

  //  if (!digitalRead(buttonPin)){
  //    delay(1000);
  //    do_send(&sendjob);

//}
