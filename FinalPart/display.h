//#include "SSD1306.h"
//  SSD1306  Display(0x3c, 4, 15, 16);
//  Display.init();
//  Display.setFont(ArialMT_Plain_10);
//  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
//  Display.drawString(128,0,"rssi "+String(ackrssi));
//  Display.setFont(ArialMT_Plain_16);
//  Display.setTextAlignment(TEXT_ALIGN_CENTER);
//  Display.drawString(64, 22, "ACK RECEIVED");

void displayInof()
{
  Display.init();
  Display.clear();
  Display.drawString(64, 22, "ACK RECEIVED");
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 22, "ACK RECEIVED");
  Display.display();
}


void display_send_complete()
{
  Display.init();
  Display.clear();
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 22, "complete");
  Display.display();
}
