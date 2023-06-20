#include <Arduino.h>
#include <LoRaWan-Arduino.h>
#include <ESP32_LoRaWAN.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include "ArduinoNvs.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
CayenneLPP lpp(51);
static esp_adc_cal_characteristics_t *adc_chars;
#define NO_OF_SAMPLES 16
#define DEFAULT_VREF 1100 
//---- Temperature Sensor
#include "MLX90614.h"

esl::MLX90614 mlx90614;

#include "mpu9250.h"
/* Mpu9250 object */
bfs::Mpu9250 imu;

#include <TinyGPS++.h>
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;
//GPS Configulation
#define GPS_RUN_TIME    3000 //ms (3 sec)
#define GPS_SLEEP_TIME    1000*60*30 //ms (30 mins)
#define GPS_2ND_RUN_TIME  1000*30 //ms (30 sec)
#define GPS_2ND_SLEEP_TIME  1000*60*60*2 //ms (2 hours)

#include "localDatabase.hpp"

//LoRa Configulation
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                     /**< Maximum number of events in the scheduler queue. */

#define LORAWAN_APP_DATA_BUFF_SIZE 64  /**< Size of the data to be transmitted. */
#define LORAWAN_APP_TX_DUTYCYCLE 14400000 /**< Defines the application data transmission duty cycle. 60s, value in [ms]. */
#define APP_TX_DUTYCYCLE_RND 1000    /**< Defines a random delay for application data transmission duty cycle. 1s, value in [ms]. */
#define JOINREQ_NBTRIALS 50        /**< Number of trials for the join request. */

hw_config hwConfig;

#ifdef ESP32
// ESP32 - SX126x pin configuration
int PIN_LORA_RESET = 4;  // LORA RESET
int PIN_LORA_NSS = 4;  // LORA SPI CS
int PIN_LORA_SCLK = 6;   // LORA SPI CLK
int PIN_LORA_MISO = 5;   // LORA SPI MISO
int PIN_LORA_DIO_1 = 10; // LORA DIO_1
int PIN_LORA_BUSY = 3;   // LORA SPI BUSY
int PIN_LORA_MOSI = 7;   // LORA SPI MOSI
int RADIO_TXEN = -1;   // LORA ANTENNA TX ENABLE
int RADIO_RXEN = -1;   // LORA ANTENNA RX ENABLE
#endif

// Foward declaration
static uint8_t lorawan_board_get_battery_level_handler(void);
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void send_lora_frame(void);
static uint32_t timers_init(void);
static void displayInfo(void);

// APP_TIMER_DEF(lora_tx_timer_id);                                              ///< LoRa tranfer timer instance.
TimerEvent_t appTimer;                              ///< LoRa tranfer timer instance.
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];        ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PRIVAT_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {lorawan_board_get_battery_level_handler, BoardGetUniqueId, BoardGetRandomSeed,
                    lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler};


// Check if the board has an LED port defined
#ifndef LED_BUILTIN
#ifdef ESP32
// #define LED_BUILTIN 2
#endif
#endif

uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0x4F, 0x30};

uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t nodeAppKey[16] = {0x20, 0x62, 0x61, 0x61, 0x05, 0xA1, 0x9D, 0x56, 0xA0, 0x47, 0xCE, 0x1D, 0x9A, 0x29, 0x9F, 0x2C};

uint32_t nodeDevAddr = 0x260BE5D2;

uint8_t nodeNwsKey[16] = {0xD2, 0x13, 0x33, 0xF4, 0xB4, 0xAA, 0xA0, 0x50, 0xD0, 0x61, 0xAE, 0xD4, 0x8F, 0xDF, 0x95, 0x86};

uint8_t nodeAppsKey[16] = {0x8C, 0x3E, 0x03, 0xAC, 0xA4, 0xCC, 0xD5, 0xAC, 0x22, 0xD8, 0x0B, 0xF1, 0xE7, 0x8E, 0x48, 0x7F};

TimerHandle_t xTimerAcquisition;

static void setup_temperature_sensor(){
  // initialize the sensor
  mlx90614.init( );
}

static void setup_imu(){
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    //while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    //while(1) {}
  }
}

int nmea0183_checksum(char *nmea_data)
{
    int crc = 0;
    int i;
  uint32_t len = strlen(nmea_data);
    // ignore the first $ sign,  no checksum in sentence
    for (i = 1; i < len; i ++) { // removed the - 3 because no cksum is present
        crc ^= nmea_data[i];
    }

    return crc;
}

void vTimerCallback( TimerHandle_t xTimer )
 {
 const uint32_t ulMaxExpiryCountBeforeStopping = 100;
 uint32_t ulCount;

    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT( xTimer );

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );

    /* Increment the count, then test to see if the timer has expired
    ulMaxExpiryCountBeforeStopping yet. */
    ulCount++;

  add_imu_to_database(0, imu.accel_x_mps2(), imu.accel_y_mps2(), imu.accel_z_mps2(),
     imu.gyro_x_radps(), imu.gyro_y_radps(), imu.gyro_z_radps(),
     imu.mag_x_ut(), imu.mag_y_ut(), imu.mag_z_ut()
     );

    /* If the timer has expired Save to DB. */
    if( ulCount >= ulMaxExpiryCountBeforeStopping )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        vTimerSetTimerID( xTimer, ( void * ) 0 );
    }
    else
    {
       /* Store the incremented count back into the timer's ID field
       so it can be read back again the next time this software timer
       expires. */
       vTimerSetTimerID( xTimer, ( void * ) ulCount );

     
    }
 }

static void setupTimer(){
  xTimerAcquisition = xTimerCreate
                   ( /* Just a text name, not used by the RTOS
                     kernel. */
                     "Timer Acquisition",
                     /* The timer period in ticks, must be
                     greater than 0. */
                     pdMS_TO_TICKS(100),
                     /* The timers will auto-reload themselves
                     when they expire. */
                     pdTRUE,
                     /* The ID is used to store a count of the
                     number of times the timer has expired, which
                     is initialised to 0. */
                     ( void * ) 0,
                     /* Each timer calls the same callback when
                     it expires. */
                     vTimerCallback
                   );

         if( xTimerAcquisition == NULL )
         {
             /* The timer was not created. */
         }
         else
         {
             /* Start the timer.  No block time is specified, and
             even if one was it would be ignored because the RTOS
             scheduler has not yet been started. */
             if( xTimerStart( xTimerAcquisition, 0 ) != pdPASS )
             {
                 /* The timer could not be set into the Active
                 state. */
             }
         }
}

void setup()
{ 
  // Initialize Serial for debug output
  Serial.begin(115200);
  Serial1.begin(GPSBaud, SERIAL_8N1, 8, 9);

  // Initialize NVS
  NVS.begin();
  /*** BLOB ***/

  //Initialize ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);

  //Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

  int rawADC = 0;
  
  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
   rawADC += adc1_get_raw(ADC1_CHANNEL_2);
  }
  rawADC /= NO_OF_SAMPLES;

  uint32_t batt_voltage = esp_adc_cal_raw_to_voltage(rawADC, adc_chars) * 2;
  Serial.printf("Battery Level: %d mV.\n", batt_voltage);

    // read from flash
  size_t blobLength;
    blobLength = NVS.getBlobSize("nodeDeviceEUI");   
    bool res = NVS.getBlob("nodeDeviceEUI", nodeDeviceEUI, sizeof(nodeDeviceEUI));

  blobLength = NVS.getBlobSize("nodeAppEUI");   
    res = NVS.getBlob("nodeAppEUI", nodeAppEUI, sizeof(nodeAppEUI));

  blobLength = NVS.getBlobSize("nodeAppKey");   
    res = NVS.getBlob("nodeAppKey", nodeAppKey, sizeof(nodeAppKey));

  blobLength = NVS.getBlobSize("nodeNwsKey");   
    res = NVS.getBlob("nodeNwsKey", nodeNwsKey, sizeof(nodeNwsKey));

  blobLength = NVS.getBlobSize("nodeAppsKey");   
    res = NVS.getBlob("nodeAppsKey", nodeAppsKey, sizeof(nodeAppsKey));

  nodeDevAddr = NVS.getInt("nodeDevAddr");

  Wire.begin(19, 18);
  
  
  Serial.println("=====================================");
  Serial.println("Kuse Cattle Tracker");
  Serial.println("=====================================");
  
  Serial.printf("DevEui=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", nodeDeviceEUI[0], nodeDeviceEUI[1], nodeDeviceEUI[2], nodeDeviceEUI[3], nodeDeviceEUI[4], nodeDeviceEUI[5], nodeDeviceEUI[6], nodeDeviceEUI[7]);
  Serial.printf("DevAdd=%08X\n", nodeDevAddr);
  Serial.printf("AppEui=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", nodeAppEUI[0], nodeAppEUI[1], nodeAppEUI[2], nodeAppEUI[3], nodeAppEUI[4], nodeAppEUI[5], nodeAppEUI[6], nodeAppEUI[7]);  
  Serial.printf("AppKey=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n\n",
        nodeAppKey[0], nodeAppKey[1], nodeAppKey[2], nodeAppKey[3], nodeAppKey[4], nodeAppKey[5], nodeAppKey[6], nodeAppKey[7],
        nodeAppKey[8], nodeAppKey[9], nodeAppKey[10], nodeAppKey[11], nodeAppKey[12], nodeAppKey[13], nodeAppKey[14], nodeAppKey[15]);
    

  // Serial.print("Checksum for $PMTK225,1,3000,12000,18000,72000 is ");
  // Serial.println(nmea0183_checksum("$PMTK225,1,3000,12000,18000,72000"),HEX);

  // Serial.print("\nChecksum for $PMTK225,2,3000,12000,18000,72000 is ");
  // Serial.println(nmea0183_checksum("$PMTK225,2,3000,12000,18000,72000"),HEX);

  setup_temperature_sensor();
  float ambientTemp = mlx90614.readAmbientTemperature();
  float objectTemp = mlx90614.readObjectTemperature();
  if (mlx90614.is_crc_error() ) {
    Serial.print( "mlx crc error " );
  } 

  Serial.print( "Object1 temp. (To): " );
  Serial.println(objectTemp);
  
  Serial.print( "Ambient temp. (Ta): " );
  Serial.println(ambientTemp);
  setup_imu();

  //Init GPS
  Serial.println("Clear gps data");
  while (Serial1.available() > 0)
    Serial.print(Serial1.read());
  Serial.print("\nSet GPS data: ");
  char command[100] = "";
  sprintf(command, "$PMTK225,1,%d,%d,%d,%d",GPS_RUN_TIME,GPS_SLEEP_TIME,GPS_2ND_RUN_TIME,GPS_2ND_SLEEP_TIME);   
  Serial.printf("%s*%02X\r\n",command,nmea0183_checksum(command));
  Serial1.printf("%s*%02X\r\n",command,nmea0183_checksum(command));
  Serial.println();
  while (Serial1.available() > 0)
    Serial.print(Serial1.read());
  Serial.println();

#ifndef RAK4630
  // Define the HW configuration between MCU and SX126x
  hwConfig.CHIP_TYPE = SX1262_CHIP;     // Example uses an eByte E22 module with an SX1262
  hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
  hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;   // LORA SPI CS
  hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;   // LORA SPI CLK
  hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;   // LORA SPI MISO
  hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
  hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;   // LORA SPI BUSY
  hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;   // LORA SPI MOSI
  hwConfig.RADIO_TXEN = RADIO_TXEN;     // LORA ANTENNA TX ENABLE
  hwConfig.RADIO_RXEN = RADIO_RXEN;     // LORA ANTENNA RX ENABLE
  hwConfig.USE_DIO2_ANT_SWITCH = false;   // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = false;       // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;   // Only Insight ISP4520 module uses DIO3 as antenna control
  hwConfig.USE_LDO = true; // Whether SX126x uses LDO or DCDC power regulator
#endif

  // Initialize Scheduler and timer
  Serial.printf("timers_init\n");
  uint32_t err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
  }

  // Initialize LoRa chip.
  Serial.printf("lora_hardware_init\n");
  err_code = lora_hardware_init(hwConfig);
  if (err_code != 0)
  {
    Serial.printf("lora_hardware_init failed - %d\n", err_code);
  }

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);
  lmh_setNwkSKey(nodeNwsKey);
  lmh_setAppSKey(nodeAppsKey);
  lmh_setDevAddr(nodeDevAddr);

  // Initialize LoRaWan
  Serial.printf("lmh_init\n");
  err_code = lmh_init(&lora_callbacks, lora_param_init, true, CLASS_A, LORAMAC_REGION_AS923);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
  }

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Use either
  // lmh_setSingleChannelGateway
  // or
  // lmh_setSubBandChannels
  //
  // DO NOT USE BOTH OR YOUR COMMUNICATION WILL MOST LIKELY NEVER WORK
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Setup connection to a single channel gateway
  // lmh_setSingleChannelGateway(0, DR_3);

  // For some regions we might need to define the sub band the gateway is listening to
  // This must be called AFTER lmh_init()
  /// \todo This is for Dragino LPS8 gateway. How about other gateways???
  // if (!lmh_setSubBandChannels(1))
  // {
  //  Serial.println("lmh_setSubBandChannels failed. Wrong sub band requested?");
  // }

  
  setup_database();
  // setupTimer();

  // Start Join procedure
  Serial.printf("lmh_join\n");
  lmh_join();
}

void loop()
{
#ifdef ESP8266
  // Handle Radio events
  Radio.IrqProcess();
#endif
while (Serial1.available() > 0)
  if (gps.encode(Serial1.read()))
      displayInfo();

  if (millis() > 10000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true){
      delay(1000);
    }
  }
  // We are on FreeRTOS, give other tasks a chance to run
  // delay(100);
}

/**@brief Get the current battery level
 * @retval value    battery level ( 0: very low, 254: fully charged )
 */
static uint8_t lorawan_board_get_battery_level_handler(void){
  int rawADC = 0;
  
  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
   rawADC += adc1_get_raw(ADC1_CHANNEL_2);
  }
  rawADC /= NO_OF_SAMPLES;

  uint32_t batt_voltage = esp_adc_cal_raw_to_voltage(rawADC, adc_chars) * 2;
  uint8_t batt_percent = map(batt_voltage, 2800, 4200, 0, 254);
  Serial.printf("Battery Level: %d %.\n", batt_percent);

  return batt_percent;
}

/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OVER_THE_AIR_ACTIVATION failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief LoRa function for handling HasJoined event.
*/
static void lorawan_has_joined_handler(void)
{
#if (OVER_THE_AIR_ACTIVATION != 0)
  Serial.println("Network Joined");
#else
  Serial.println("OVER_THE_AIR_ACTIVATION != 0");

#endif
  lmh_class_request(CLASS_A);

  TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
  TimerStart(&appTimer);
}

/**@brief Function for handling LoRaWan received data from Gateway

   @param[in] app_data  Pointer to rx data
*/
static void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

  switch (app_data->port)
  {
  case 3:
    // Port 3 switches the class
    if (app_data->buffsize == 1)
    {
      switch (app_data->buffer[0])
      {
      case 0:
        lmh_class_request(CLASS_A);
        break;

      case 1:
        lmh_class_request(CLASS_B);
        break;

      case 2:
        lmh_class_request(CLASS_C);
        break;

      default:
        break;
      }
    }
    break;

  case LORAWAN_APP_PORT:
    // YOUR_JOB: Take action on received data
    break;

  default:
    break;
  }
}

static void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);

  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = LORAWAN_APP_PORT;
  lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

static void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

bool isSetPeriodic = false;

static void displayInfo()
{
  
  if (gps.location.isValid())
  {
  if(isSetPeriodic){
    isSetPeriodic = true;
  }
    Serial.print(F("Location: ")); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    // Serial.print(F("INVALID"));
  }

  
  if (gps.location.isValid() && gps.date.isValid())
  {
    Serial.print(F("  Date/Time: "));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    // Serial.print(F("INVALID"));
  }

  // Serial.print(F(" "));
  if (gps.location.isValid() && gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    Serial.println();
  }
  else
  {
    // Serial.print(F("INVALID"));
  }

  // Serial.println();
}

static void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    Serial.println("Did not join network, skip sending frame");
    return;
  }

  delay(200);
  float ambientTemp = mlx90614.readAmbientTemperature();
  float objectTemp = mlx90614.readObjectTemperature();
  if (mlx90614.is_crc_error() ) {
    return;
  } 

  // uint32_t i = 0;
  // m_lora_app_data.port = LORAWAN_APP_PORT;
  // m_lora_app_data.buffer[i++] = 'H';
  // m_lora_app_data.buffer[i++] = 'e';
  // m_lora_app_data.buffer[i++] = 'l';
  // m_lora_app_data.buffer[i++] = 'l';
  // m_lora_app_data.buffer[i++] = 'o';
  // m_lora_app_data.buffer[i++] = ' ';
  // m_lora_app_data.buffer[i++] = 'w';
  // m_lora_app_data.buffer[i++] = 'o';
  // m_lora_app_data.buffer[i++] = 'r';
  // m_lora_app_data.buffer[i++] = 'l';
  // m_lora_app_data.buffer[i++] = 'd';
  // m_lora_app_data.buffer[i++] = '!';
  // m_lora_app_data.buffsize = i;

  lpp.reset();

  int rawADC = 0;  
  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
   rawADC += adc1_get_raw(ADC1_CHANNEL_2);
  }
  rawADC /= NO_OF_SAMPLES;
  uint32_t batt_voltage = esp_adc_cal_raw_to_voltage(rawADC, adc_chars) * 2;
  Serial.printf("Battery Level: %d mV.\n", batt_voltage);
  lpp.addAnalogOutput(0, (float)batt_voltage/1000.0f);

  Serial.print( "Object1 temp. (To): " );
  Serial.println(objectTemp);
  lpp.addTemperature(0, objectTemp);
  
  Serial.print( "Ambient temp. (Ta): " );
  Serial.println(ambientTemp);
  lpp.addTemperature(1, ambientTemp);

  if (imu.Read()) {
    Serial.print(imu.new_imu_data()); Serial.print("\t");
    Serial.print(imu.new_mag_data()); Serial.print("\t");
    Serial.print(imu.accel_x_mps2()); Serial.print("\t");
    Serial.print(imu.accel_y_mps2()); Serial.print("\t");
    Serial.print(imu.accel_z_mps2()); Serial.print("\t");
    Serial.print(imu.gyro_x_radps()); Serial.print("\t");
    Serial.print(imu.gyro_y_radps()); Serial.print("\t");
    Serial.print(imu.gyro_z_radps()); Serial.print("\t");
    Serial.print(imu.mag_x_ut()); Serial.print("\t");
    Serial.print(imu.mag_y_ut()); Serial.print("\t");
    Serial.print(imu.mag_z_ut()); Serial.print("\t");
    Serial.print(imu.die_temp_c()); Serial.print("\n");

    lpp.addAccelerometer(0, imu.accel_x_mps2(), imu.accel_y_mps2(), imu.accel_z_mps2());
    lpp.addGyrometer(0, imu.gyro_x_radps(), imu.gyro_y_radps(), imu.gyro_z_radps());
    lpp.addAnalogOutput(100, imu.mag_x_ut());
    lpp.addAnalogOutput(101, imu.mag_y_ut());
    lpp.addAnalogOutput(102, imu.mag_z_ut());
  }

    if (gps.location.isValid())
    {
      Serial.print("GPS Location: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.println(gps.location.lng(), 6);

      lpp.addGPS(0, gps.location.lat(), gps.location.lng(), 0);
    }
    
    

  m_lora_app_data.port = LORAWAN_APP_PORT;
  // m_lora_app_data.buffer = lpp.getBuffer();
  m_lora_app_data.buffsize = lpp.getSize();

  memcpy(m_lora_app_data.buffer, lpp.getBuffer(), lpp.getSize());

  lmh_error_status error = lmh_send(&m_lora_app_data, LMH_CONFIRMED_MSG);

  Serial.print("Debug: ");
  for(uint16_t i=0; i<lpp.getSize(); i++){
    printHex(lpp.getBuffer()[i]);
  }
  Serial.println();
  // lora.sendUplink((char *)lpp.getBuffer(), lpp.getSize(), 0, 1);
  Serial.printf("lmh_send result %d\n", error);

}

/**@brief Function for handling a LoRa tx timer timeout event.
*/
static void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_TX_DUTYCYCLE);
  TimerStart(&appTimer);
  Serial.println("Sending frame");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.

   @details Initializes the timer module. This creates and starts application timers.
*/
static uint32_t timers_init(void)
{
  appTimer.timerNum = 3;
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}
