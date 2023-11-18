#include <ros.h>
#include <auv_msgs/DisplayScreen.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_CLK 13
#define TFT_MISO 12
#define TFT_MOSI 11
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

#define ILI9341_BLACK       0x0000  ///<   0,   0,   0
#define ILI9341_NAVY        0x000F  ///<   0,   0, 123
#define ILI9341_DARKGREEN   0x03E0  ///<   0, 125,   0
#define ILI9341_DARKCYAN    0x03EF  ///<   0, 125, 123
#define ILI9341_MAROON      0x7800  ///< 123,   0,   0
#define ILI9341_PURPLE      0x780F  ///< 123,   0, 123
#define ILI9341_OLIVE       0x7BE0  ///< 123, 125,   0
#define ILI9341_LIGHTGREY   0xC618  ///< 198, 195, 198
#define ILI9341_DARKGREY    0x7BEF  ///< 123, 125, 123
#define ILI9341_BLUE        0x001F  ///<   0,   0, 255
#define ILI9341_GREEN       0x07E0  ///<   0, 255,   0
#define ILI9341_CYAN        0x07FF  ///<   0, 255, 255
#define ILI9341_RED         0xF800  ///< 255,   0,   0
#define ILI9341_MAGENTA     0xF81F  ///< 255,   0, 255
#define ILI9341_YELLOW      0xFFE0  ///< 255, 255,   0
#define ILI9341_WHITE       0xFFFF  ///< 255, 255, 255
#define ILI9341_ORANGE      0xFD20  ///< 255, 165,   0
#define ILI9341_GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define ILI9341_PINK        0xFC18  ///< 255, 130, 198

ros::NodeHandle nh;

String mission;
float dvl;
float imu;
float depth;



void displayScreenCallback(const auv_msgs::DisplayScreen& msg) {
    mission = msg.mission;
    dvl = msg.dvl;
    imu = msg.imu;
    depth = msg.depth;
}

ros::Subscriber<auv_msgs::DisplayScreen> sub_display_screen("/display_screen", &displayScreenCallback);


void setup() {
  nh.initNode();
  nh.subscribe(sub_display_screen);
	

	Serial.begin(9600);
  tft.begin();
}


void loop() {
    tft.fillScreen(ILI9341_NAVY);
    tft.setCursor(0, 5);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);
    tft.print("Battery: ");
    tft.println("10%");
    tft.print("DVL: ");
    tft.println(dvl);
    tft.println();
    tft.print("IMU: ");
    tft.println(imu);
    tft.print("Depth: ");
    tft.println(depth);
    tft.print("Mission: ");
    tft.println(mission);
    nh.spinOnce();
    delay(5000);
}
