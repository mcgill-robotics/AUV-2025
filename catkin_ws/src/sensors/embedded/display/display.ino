#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
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
#define ILI9341_DARKGREEN   0x03E9  ///<   0, 125,   0
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

String mission = "Gate";
int isDvlActive = 0;
int isImuActive = 0;
int isDepthActive = 0;
int battery = 25;


void dvl_status_cb(const std_msgs::Int32& msg) {
  isDvlActive = msg.data;
}
void depth_status_cb(const std_msgs::Int32& msg) {
  isDepthActive = msg.data;
}
void imu_status_cb(const std_msgs::Int32& msg) {
  isImuActive = msg.data;
}

void mission_cb(const std_msgs::String& msg) {
  mission = msg.data;
}

ros::Subscriber<std_msgs::Int32> sub_dvl_status("/sensors/dvl/status", &dvl_status_cb);
ros::Subscriber<std_msgs::Int32> sub_depth_status("/sensors/depth/status", &depth_status_cb);
ros::Subscriber<std_msgs::Int32> sub_imu_status("/sensors/imu/status", &imu_status_cb);
ros::Subscriber<std_msgs::String> sub_mission("/mission_display", &mission_cb);


void setup() {
  nh.initNode();
  nh.subscribe(sub_dvl_status);
  nh.subscribe(sub_depth_status);
  nh.subscribe(sub_imu_status);
  nh.subscribe(sub_mission);

	Serial.begin(9600);
  tft.begin();
  

}
int i = 0;

void loop() {
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 5);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);

    // Battery
    if (battery > 50) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("Battery: ");
      tft.print(battery);
      tft.println("%\n");
    } else if (battery > 15) {
      tft.setTextColor(ILI9341_YELLOW);
      tft.print("Battery: ");
      tft.print(battery);
      tft.println("%\n");
    } else {
      tft.setTextColor(ILI9341_RED);
      tft.print("Battery: ");
      tft.print(battery);
      tft.println("%\n");
    }
    
    // DVL
    if (isDvlActive) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("DVL: ");
      tft.println("O\n");
    } else {
      tft.setTextColor(ILI9341_RED);
      tft.print("DVL: ");
      tft.println("X\n");
    } 

    // IMU
    if (isImuActive) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("IMU: ");
      tft.println("O\n");
    } else {
      tft.setTextColor(ILI9341_RED);
      tft.print("IMU: ");
      tft.println("X\n");
    } 
    
    // Depth Sensor
    if (isDepthActive) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("Depth: ");
      tft.println("O\n");
    } else {
      tft.setTextColor(ILI9341_RED);
      tft.print("Depth: ");
      tft.println("X\n");
    } tft.setTextColor(ILI9341_WHITE);

    tft.print("Mission: ");
    tft.println(mission);
    tft.println();
    nh.spinOnce();
    delay(1000);
}

