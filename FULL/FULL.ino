#include <WiFi.h>
#include <WiFiManager.h>
#include <Ticker.h>
Ticker ticker;
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include <DHT.h>

#define LED  12
#define LED  13
#define LOCK  27

#define SW_reset  15

#define SW1  36
int SW1State = 0;
#define SW2  25
int SW2State = 0;
#define SW3  33
int SW3State = 0;

const char* ssid = "Sangtong";
const char* password = "042781418";

//Your IP address or domain name with URL path
const char* serverName = "http://192.168.1.131/esp/esp-outputs-action.php?action=outputs_state&board=1";
//test
const char* serverName2 = "http://192.168.1.131/SENSOR/esp-post-data.php";
//http://sangtongshop.000webhostapp.com/esp-outputs-action.php?action=outputs_state&board=1";
// Update interval time set to 5 seconds
const long interval = 5000;
unsigned long previousMillis = 0;
String outputsState;


String apiKeyValue = "tPmAT5Ab3j7F9";

void tick() {
  digitalWrite(LED, !digitalRead(LED));
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  ticker.attach(0.2, tick);
}

void Wifi_Reset_begin() {
  if (digitalRead(SW_reset) == LOW) {
    Serial.println("Wifi Reset? Pls. waiting 3S..");
    delay(3000);
    if (digitalRead(SW_reset) == LOW) {
      delay(10);
      while (digitalRead(SW_reset) == LOW) {
        digitalWrite(LED, HIGH);
        delay(10);
      }
      Serial.println("WiFi Reset Settings....OK!.");
      WiFiManager wm;
      wm.resetSettings();
      ESP.restart();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  //pinMode(SW_reset, INPUT_PULLUP);

  // Connect to Wi-Fi
  //ticker.attach(1, tick);
  //WiFiManager wm;
  //wm.setAPCallback(configModeCallback);

  //if ( !wm.autoConnect("WiFI_ABC_AP") ) {
  //Serial.println("failed to connect and hit timeout");
  //ESP.restart();
  //delay(1000);
  //}

  //ticker.detach();
  //digitalWrite(LED, LOW);
  //delay(100);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("");
  Serial.println("connected...already..WiFi :)");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void swit() {
  if (digitalRead(SW1) == LOW) {
    //Serial.println("SW1 is Pressed ON");
    SW1State = 1;
  }
  else {
    SW1State = 0;
    //Serial.println("SW1 is NOT pressed DISABLE");
  } //sw2
  if (digitalRead(SW2) == LOW) {
    //Serial.println("SW2 is pressed ON");
    SW2State = 1;
  }
  else {
    SW2State = 0;
    //Serial.println("SW2 is NOT Pressed & DISABLE");
  } //sw3
  if (digitalRead(SW3) == LOW) {
    //Serial.println("SW3 is pressed ON");
    SW3State = 1;
  }
  else {
    SW3State = 0;
    //Serial.println("SW3 is NOT Pressed DISABLE");
  }
}

void loop() {

  //Wifi_Reset_begin();
  unsigned long currentMillis = millis();

  //ver1
  if (currentMillis - previousMillis >= interval) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED ) {
      WiFiClient client;
      HTTPClient http;
      //SEND TO PHP
      http.begin(client, serverName2);
      // Specify content-type header
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // Prepare your HTTP POST request data
      //SW
      swit();
      String httpRequestData = "api_key=" + apiKeyValue + "&value1=" + String(SW1State)
                               + "&value2=" + String(SW2State) + "&value3=" + (SW3State);
      delay(100);
      Serial.print("httpRequestData: ");
      Serial.println(httpRequestData);
      // Send HTTP POST request
      int httpResponseCode = http.POST(httpRequestData);
      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }

      //RECIVE LED
      outputsState = httpGETRequest(serverName);
      Serial.println(outputsState);
      JSONVar myObject = JSON.parse(outputsState);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }

      Serial.print("JSON object = ");
      Serial.println(myObject);

      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys = myObject.keys();

      for (int i = 0; i < keys.length(); i++) {
        JSONVar value = myObject[keys[i]];
        Serial.print("GPIO: ");
        Serial.print(keys[i]);
        Serial.print(" - SET to: ");
        Serial.println(value);
        pinMode(atoi(keys[i]), OUTPUT);
        digitalWrite(atoi(keys[i]), atoi(value));
        delay(100);
      }
      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  }
}
String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(client, serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();
  return payload;
}
