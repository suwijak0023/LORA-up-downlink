#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x0C, 0xBA, 0xBC, 0x47, 0xC4, 0xE9, 0x6C, 0x9A };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16]={ 0x31, 0x81, 0xA0, 0xDB, 0x39, 0x3E, 0x4A, 0x43, 0x3B, 0x6B, 0xD8, 0x20, 0x44, 0xC2, 0xE5, 0x0A };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t btn_activated[1] = { 0x01};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;
#define SCK     5    // GPIO5  -- heltec
#define MISO    19   // GPIO19 -- heltec
#define MOSI    27   // GPIO27 -- heltec
#define SS      18   // GPIO18 -- heltec
#define RST     14   // GPIO14 -- heltec
#define DI0     26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RST,
    .dio = {DI0, 3, 4},
};

//------ Added ----------------
#define LED_YELLOW 25
#define LED_GREEN  6

#define DHT_PIN 7
#define BTN_PIN 0

// DHT11 or DHT22
#define DHTTYPE DHT11

// Initialize dht
DHT dht(DHT_PIN, DHTTYPE);

int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
//-----------------------------

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
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
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));

              //------ Added ----------------
              if (LMIC.dataLen == 1) {
                uint8_t result = LMIC.frame[LMIC.dataBeg + 0];
                if (result == 0)  {
                  Serial.println("RESULT 0");
                  digitalWrite(LED_YELLOW, LOW);
                  digitalWrite(LED_GREEN, LOW);
                }              
                if (result == 1)  {
                  Serial.println("RESULT 1");
                  digitalWrite(LED_YELLOW, HIGH);
                  digitalWrite(LED_GREEN, LOW);                  
                } 
                if (result == 2)  {
                  Serial.println("RESULT 2");
                  digitalWrite(LED_YELLOW, LOW);
                  digitalWrite(LED_GREEN, HIGH);                     
                } 
                if (result == 3)  {
                  Serial.println("RESULT 3");
                  digitalWrite(LED_YELLOW, HIGH);
                  digitalWrite(LED_GREEN, HIGH);                       
                }                                             
              }
             Serial.println();
             //-----------------------------
            }
                       
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        uint32_t humidity = dht.readHumidity(false) * 100;
        uint32_t temperature = dht.readTemperature(false) * 100;
        
        Serial.println("Humidity: " + String(humidity));
        Serial.println("Temperature: " + String(temperature));
        
        byte payload[4];
        payload[0] = highByte(humidity);
        payload[1] = lowByte(humidity);
        payload[2] = highByte(temperature);
        payload[3] = lowByte(temperature); 
    
        //Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    //------ Added ----------------
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(BTN_PIN, INPUT);

    digitalWrite(BTN_PIN, LOW); 

    dht.begin();
    //-----------------------------
    
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

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    //------ Added ----------------
    // read the state of the button value:
    buttonState = digitalRead(BTN_PIN);

    // compare the buttonState to its previous state
    if (buttonState != lastButtonState) {

        if (buttonState == HIGH) {
            // if the current state is HIGH then the button went from off to on:
            LMIC_setTxData2(1, btn_activated, sizeof(btn_activated), 0);
            Serial.println(F("Button On"));
        } else {
            // if the current state is LOW then the button went from on to off:
            Serial.println(F("Button Off"));
        }
        // Delay a little bit to avoid bouncing
        delay(50);
    }

    // save the current state as the last state, for next time through the loop
    lastButtonState = buttonState;
    //-----------------------------
    
    os_runloop_once();
}
