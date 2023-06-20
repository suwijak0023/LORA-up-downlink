#include "display.h"
//#include "downlink.h"
int button = 0;
//int led = 25;
int status = false;

void buttonin() {

  if (digitalRead(button) == true) {
    status = !status;
    Serial.println(status);

    //          deviceState = DEVICE_STATE_SEND;
    //    LoRaWAN.send(loraWanClass);
    //    Txup_Send_Button_On( appPort );
    //    display_send_complete();
    //    Serial.println("Status : START ");
    //    deviceState = DEVICE_STATE_SLEEP;
    //    LoRaWAN.sleep(loraWanClass, debugLevel);
    //    delay(2000);

    //          deviceState = DEVICE_STATE_CYCLE;
  }
  //          else {
  //            displayInof();
  //            deviceState = DEVICE_STATE_CYCLE;
  //            }
  while (digitalRead(button == true));
  delay(50);

  if (status == 1) {
    LoRaWAN.send(loraWanClass);
    Txup_Send_Button_On( appPort );
    display_send_complete();
    Serial.println("Status : START ");
//    deviceState = DEVICE_STATE_CYCLE;
    status = 0;
    //    delay(50);
  }
  else {
//    displayInof();
    deviceState = DEVICE_STATE_CYCLE;
//    status = 1;
//    app();
  }

}



//void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
//{
//    if (event == TWR_BUTTON_EVENT_CLICK)
//    {
//        click_count++;
//
//        header = HEADER_BUTTON_CLICK;
//
//        twr_scheduler_plan_now(0);
//    }
//    else if (event == TWR_BUTTON_EVENT_HOLD)
//    {
//        hold_count++;
//
//        header = HEADER_BUTTON_HOLD;
//
//        twr_scheduler_plan_now(0);
//    }
//}
