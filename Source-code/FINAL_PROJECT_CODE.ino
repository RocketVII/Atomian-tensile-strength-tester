#include <SPI.h>

//#include <SD.h>

#include "RTClib.h"

#include "Adafruit_ILI9341.h"

#include "Adafruit_VL53L1X.h"

#include <EEPROM.h>

#include "HX711.h"

#include "RTClib.h"
// RTC clock
RTC_DS3231 rtc;
#include <SPI.h>
#include "SdFat.h"
SdFat SD;

#define SD_CS_PIN 44
File myFile;
String File_name;

// boolean to indicate which curve we will use;
boolean DW_CURVE = false;
boolean SS_CURVE = false;
//defining the screen
#define TFT_CS 45
#define TFT_DC 49
#define TFT_RST 8

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// curve variables
double ox, oy;
double xaxis, yaxis;
//a boolean to draw the background for the plot
boolean plotbackground = true;
int dataticks;
// don't change X & Y unless you changed the screen
int x_cursor = 140;
int y_cursor = 130;
/*
 * Distance sensor & Load cell vars
 */

// use any unused pins ( if you use the stemma QT there is no need to connect these pins to anything)
#define IRQ_PIN 4
#define XSHUT_PIN 5
// just needed to set up the Distance sensor
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
double prevdistance = -5;
double prevload = -5;

// load cell set up
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 11;
HX711 scale;

//an array to get the mean
double loadcalval;
double yDistance;
double xaxis_Saved_value;
double yaxis_Saved_value;
double Length_Saved;
double width_Saved;
double Thickness_Saved;
double area_sample;
// the the stress coefficient is just 9.81/(area_sample*10^6*10^-3*10^-3*10^-6) = 9.81/(area_sample)
double stress_coefficient;

//__________________________________________________________
/*
 * The states of the System and the boolean values for the A,B,C,D buttons
 *
 */
enum UI_States {
  UI_START,
  UI_INT,
  UI_MAINMENU,
  UI_SETTINGS,
  UI_CALIBRATE1,
  UI_CALIBRATE2,
  UI_ADJUST_XAXIS,
  UI_ADJUST_YAXIS,
  UI_CHOOSECURVE,
  UI_GET_LENGTH,
  UI_GET_WIDTH,
  UI_GET_THICKNESS,
  UI_STARTTEST,
  UI_DISTANCEWEIGHT,
  UI_STRESSSTRAIN,
  UI_FILENAME
}
UI_State;

// button flags
boolean aflag;
boolean bflag;
boolean cflag;
boolean dflag;
//_________________________________________________________

/*
 * Setting up the keypad using a matrix array
 * intarraypointer keeps track of the location of the last input number and in turn the lenght of the array
 * rowpins and col pins are the input pins to the arduino
 * intputint is the number in int form instead of the char
 * X and Y are the location of the pointers when using the number keypad
 *
 */
#include <Keypad.h>

int inputInts[6];
int intarraypointer;
int inputint;
boolean inputnum = false;

/*
 *  change depending on the input pins you used with the arduino
 *  change the rows and col bytes to 3 if you're using a 3x keypad
 *  must change the way the interface works
 */

// Constants for row and column sizes
const byte ROWS = 4;
const byte COLS = 4;

// Array to represent keys on keypad
char hexaKeys[ROWS][COLS] = {
  {
    '1',
    '2',
    '3',
    'A'
  },
  {
    '4',
    '5',
    '6',
    'B'
  },
  {
    '7',
    '8',
    '9',
    'C'
  },
  {
    '*',
    '0',
    '#',
    'D'
  }
};
//used input pins
byte rowPins[ROWS] = {
  30,
  32,
  34,
  36
};
byte colPins[COLS] = {
  22,
  24,
  26,
  28
};

//sets up the keypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
/*
    //____________________________________________________________________\\
    */

void setup() {
  Serial.begin(115200);
  SPI.begin();
  tft.setTextSize(3);
  tft.setFont();
  tft.setTextColor(ILI9341_WHITE);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //loadcell set up
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  while (!Serial) delay(10);

  /*
   * Distance sensor set up
   */

  Wire.begin();
  if (!vl53.begin(0x29, & Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
      tft.print("VL53L1X sensor Error!");
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(200);

  Serial.println(vl53.getTimingBudget());

  /*
   * SD set up
   */

Serial.print("Initializing SD card...");
if (!SD.begin(SD_CS_PIN)) {
     tft.print("SD Card Error!");
Serial.println("initialization failed!");
while (1)delay(10);
}
Serial.println("Initializing SD card Done");
/*
 * RTC SEt up
 */
  if (! rtc.begin()) {
    tft.print("RTC Error!");
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  Serial.println("Initializing RTC Done");
  //starts the state machine and the input array pointer
  UI_State = UI_START;
  intarraypointer = 0;

  //A simple color changing start up
  // note the yield function is important to use otherwire you're screen won't be cleared
  tft.setTextSize(3);
  tft.setFont();
  tft.setTextColor(ILI9341_WHITE);
  tft.begin();
  tft.setRotation(1);
  logodrawteam();
   delay(3000);
  logodraw();
  delay(3000);

}
/*
 * Main looping function
 */
void loop() {
  if (!UI_DISTANCEWEIGHT || !UI_STRESSSTRAIN) {
    printNumToTFT();
    InterfaceState();
    setflagTofalse();
    delay(100);
  } else {
    printNumToTFT();
    InterfaceState();
    setflagTofalse();
  }

}
// makes sure flags are set to false just incase
void setflagTofalse() {
  aflag = false;
  bflag = false;
  cflag = false;
  dflag = false;
}
/*
_____________________________start of draw functions___________________________________
//_________________________________________________________________________________________________________________________\\

*/
/*draws the main menu
//step 1 fill screen with black
//step 2 draw a white frame
//step 3 draw 3 rectangular white frame
//step 4 draw 3 rectangles organe,blue,red
//step 5 adds the words
*/
void mainmenudraw() {
  tft.setTextSize(3);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(40, 160, 250, 54, 8, ILI9341_RED);
  tft.drawRoundRect(40, 160, 250, 54, 8, ILI9341_WHITE); //Game

  tft.fillRoundRect(40, 90, 250, 54, 8, ILI9341_BLUE); //RGB led
  tft.drawRoundRect(40, 90, 250, 54, 8, ILI9341_WHITE);

  tft.fillRoundRect(40, 20, 250, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(40, 20, 250, 54, 8, ILI9341_WHITE); //Oscilloscope
  tft.setCursor(38, 175);
  tft.print(" C:Start test");
  tft.setCursor(38, 110);
  tft.print(" B:Settings");
  tft.setCursor(38, 38);
  tft.print(" A:Calibrate");
  tft.setTextSize(2);
}
/*draws the settings menu
//step 1 fill screen with black
//step 2 draw a white frame
//step 3 draw 3 rectangular white frame
//step 4 draw 3 rectangles organe,blue,red
//step 5 adds the words
*/
void settingsdraw() {
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(40, 160, 250, 54, 8, ILI9341_RED);
  tft.drawRoundRect(40, 160, 250, 54, 8, ILI9341_WHITE); //Game

  tft.fillRoundRect(40, 90, 250, 54, 8, ILI9341_BLUE); //RGB led
  tft.drawRoundRect(40, 90, 250, 54, 8, ILI9341_WHITE);

  tft.fillRoundRect(40, 20, 250, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(40, 20, 250, 54, 8, ILI9341_WHITE); //Oscilloscope
  tft.setTextSize(2);
  tft.setCursor(38, 38);
  tft.print(" A:Adjust X-axis");
  tft.setCursor(38, 110);
  tft.print(" B:Adjust Y-axis");
  tft.setTextSize(3);
  tft.setCursor(38, 175);
  tft.print(" D:Back");
  tft.setTextSize(2);
}
/*draws the settings menu
//step 1 fill screen with black
//step 2 draw a white frame
//step 3 draw 1 rectangular white frame
//step 4 draw 1 rectangles organ
//step 5 adds the words to it
//step 6 make 3 blue rectangles with white frames and add text to them
//step 7 print the word "Distance" in green in the assigned location
*/
void xaxisdraw() {
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 54, 8, ILI9341_WHITE);

  tft.setCursor(26, 38);
  tft.print("Enter max value in mm");

  tft.fillRoundRect(10, 90, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 90, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 180, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 180, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("A:Done");
  tft.setCursor(20, 145);
  tft.print("#:Del");
  tft.setCursor(20, 190);
  tft.print("D:Back");
  tft.setCursor(170, 100);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("Distance");
  tft.setTextColor(ILI9341_WHITE);
}
/*draws the settings menu
//step 1 fill screen with black
//step 2 draw a white frame
//step 3 draw 1 rectangular white frame
//step 4 draw 1 rectangles organ
//step 5 adds the words to it
//step 6 make 3 blue rectangles with white frames and add text to them
//step 7 print the word "Weight" in green in the assigned location
*/
void yaxisdraw() {
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 54, 8, ILI9341_WHITE);

  tft.setCursor(20, 38);
  tft.print("Enter max value in Grams");

  tft.fillRoundRect(10, 90, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 90, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 180, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 180, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("A:Done");
  tft.setCursor(20, 145);
  tft.print("#:Del");
  tft.setCursor(20, 190);
  tft.print("D:Back");
  tft.setCursor(180, 100);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("Weight");
  tft.setTextColor(ILI9341_WHITE);

}
/*
 * A reused yaxisdraw() with some reformating
 * 1 less button and different text
 */
void calweightdraw() {
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 54, 8, ILI9341_WHITE);

  tft.setCursor(15, 38);
  tft.print("Enter used weight(Grams)");

  tft.fillRoundRect(10, 90, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 90, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("A:Done");
  tft.setCursor(20, 145);
  tft.print("#:Del");
  tft.setCursor(180, 100);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("Weight");
  tft.setTextColor(ILI9341_WHITE);

}
/*
 * A simple text display function with the A: done button
 * Can use some more work but it gets the job done
 */
void caldraw() {
  tft.setTextSize(3);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 320, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 70, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 70, 8, ILI9341_WHITE);

  tft.setCursor(20, 30);
  tft.print("put the loadcell");
  tft.setCursor(20, 60);
  tft.print("in tension");
  tft.setTextSize(2);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 145);
  tft.print("A:Done");
    tft.fillRoundRect(10, 185, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 185, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 195);
  tft.print("D:Back");

}
/*
 * Following the same logic as the prev ones this one draw the start test menu
 */
void starttestdraw() {
  tft.setTextSize(3);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 320, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 70, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 70, 8, ILI9341_WHITE);

  tft.setCursor(20, 30);
  tft.print("put the sample");
  tft.setCursor(20, 60);
  tft.print("in tension");
  tft.setTextSize(2);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 145);
  tft.print("A:Tare");
}
/*
 * Following the same logic as the prev ones this one draw the thickness input menu
 */
void thicknessdraw() {
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 250, 70, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 250, 70, 8, ILI9341_WHITE);
  tft.setCursor(20, 38);
  tft.setTextSize(2);
  tft.print("Enter Thickness in ");
  tft.setCursor(20, 60);
  tft.print("Micro meter:");
  tft.setTextSize(2);
  tft.fillRoundRect(10, 100, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 100, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 145, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 145, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 110);
  tft.print("A:Done");
  tft.setCursor(20, 155);
  tft.print("#:Del");
  tft.setCursor(180, 100);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("Thickness");
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

}
/*
 * Following the same logic as the prev ones this one draw the width input menu
 */
void widthdraw() {
    tft.setTextSize(2);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 54, 8, ILI9341_WHITE);
  tft.setCursor(20, 38);
  tft.print("Enter the Width in mm:");

  tft.fillRoundRect(10, 90, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 90, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("A:Done");
  tft.setCursor(20, 145);
  tft.print("#:Del");
  tft.setCursor(180, 100);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("Width ");
  tft.setTextColor(ILI9341_WHITE);

}
/*
 * Following the same logic as the prev ones this one draw the length input menu
 */
void lengthdraw() {
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 54, 8, ILI9341_WHITE);
  tft.setCursor(20, 38);
  tft.print("Enter the length in mm:");

  tft.fillRoundRect(10, 90, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 90, 100, 40, 8, ILI9341_WHITE);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("A:Done");
  tft.setCursor(20, 145);
  tft.print("#:Del");
  tft.setCursor(180, 100);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("Length ");
  tft.setTextColor(ILI9341_WHITE);

}
/*
 * it's possible to combine these input functions into 1 that takes 2 parameters 
 */

/*
 * Following the same logic as the prev ones this one draw the file name menu
 */

void filenamedraw() {
  tft.setTextSize(2);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  //makes an empty white line rectangle
  tft.drawRoundRect(0, 0, 320, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(10, 20, 300, 70, 8, ILI9341_ORANGE);
  tft.drawRoundRect(10, 20, 300, 70, 8, ILI9341_WHITE);

  tft.setCursor(20, 30);
  tft.print("File name is :");
    tft.setTextSize(1);
  tft.setCursor(20, 60);
  tft.print(File_name);
  tft.setTextSize(2);
  tft.fillRoundRect(10, 135, 100, 40, 8, ILI9341_BLUE);
  tft.drawRoundRect(10, 135, 100, 40, 8, ILI9341_WHITE);
  tft.setCursor(20, 145);
  tft.print("A:Done");
}

/*
 * Following the same logic as the prev ones this one draw the choose curve menu
 */
void choosecurvedraw() {
  tft.setTextSize(2);
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.drawRoundRect(0, 0, 319, 240, 8, ILI9341_WHITE);

  tft.fillRoundRect(40, 160, 250, 54, 8, ILI9341_RED);
  tft.drawRoundRect(40, 160, 250, 54, 8, ILI9341_WHITE); //Game

  tft.fillRoundRect(40, 90, 250, 54, 8, ILI9341_BLUE); //RGB led
  tft.drawRoundRect(40, 90, 250, 54, 8, ILI9341_WHITE);

  tft.fillRoundRect(40, 20, 250, 54, 8, ILI9341_ORANGE);
  tft.drawRoundRect(40, 20, 250, 54, 8, ILI9341_WHITE); //Oscilloscope
  tft.setTextSize(2);
  tft.setCursor(38, 38);
  tft.print(" A:Distance-weight");
  tft.setCursor(38, 110);
  tft.print(" B:Stresss-Strain");
  tft.setTextSize(3);
  tft.setCursor(38, 175);
  tft.print(" D:Back");
  tft.setTextSize(2);
}
/*
 * Draws the Ualbany logo
 */
void logodraw(){
    tft.fillScreen(ILI9341_PURPLE);
  yield();
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(17);
         tft.setCursor(10, 40);
  tft.print("U");

    tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(6);
       tft.setCursor(100, 100);
  tft.print("Albany");
  tft.setTextSize(2);
     tft.setCursor(15, 170);
     tft.setTextColor(ILI9341_ORANGE);
  tft.print("Department of Electrical  and Computer Engineering"); 
  
    tft.setTextColor(ILI9341_WHITE);
}
/*
 * Draws our team logo feel free to remove it 
 */
void logodrawteam(){
    tft.fillScreen(ILI9341_PURPLE);
  yield();
  tft.setTextColor(ILI9341_BLACK,ILI9341_BLACK);
  tft.setTextSize(17);
         tft.setCursor(10, 40);
  tft.print("A");

    tft.setTextColor(ILI9341_ORANGE);
  tft.setTextSize(6);
       tft.setCursor(100, 100);
  tft.print("tom");
    tft.setTextColor(ILI9341_BLUE);
  tft.print("ian");
  tft.setTextSize(3);
     tft.setCursor(15, 170);
     tft.setTextColor(ILI9341_WHITE);
  tft.print("TEAM 3  "); 
  tft.setCursor(15, 200);
  tft.print("(best team)");
  
    tft.setTextColor(ILI9341_WHITE);
}

/*
End of draw functions
//_________________________________________________________________________________________________________________________\\

*/
/*
 * The function that handles the Keypad inputs
 * Used paramters:
 * inputInts[]: an array used to store a max of 8 (we only used 6rn) to store the given inputs to be used later on
 * intarraypointer: will be used for inputInts[i]10^(i-n) where n goes from i to 0 where i is intarray pointer
 * a/b/c/d flags will be set to true through this function
 * the black space will be draws here and the value of intarraypointer will be dropped by 1 to remove the val from the array
 * the location of x will be used as indication for the number of inputs max is x+25*6
 *
 */
void printNumToTFT() {
  //gets the input

  char customKey = customKeypad.getKey();

  if (customKey) {

    // the '#' will be used as Delete
    if (customKey == '#' && x_cursor > 140) {
      intarraypointer--;
      //move back one block
      x_cursor = x_cursor - 25;

      // draw a black block
      tft.fillRoundRect(x_cursor - 2, y_cursor - 2, 40, 40, 4, ILI9341_BLACK);
    } else if (customKey == 'A') {
      aflag = true;
    } else if (customKey == 'B') {
      bflag = true;
    } else if (customKey == 'C') {
      cflag = true;
    } else if (customKey == 'D') {
      dflag = true;
    } else if (x_cursor < 241 && customKey != '#' && customKey != '*' && inputnum) {
      inputint = charToint(customKey);
      if (inputint == -1) {} else {
        inputInts[intarraypointer] = inputint;
        intarraypointer++;
        tft.setTextColor(ILI9341_GREEN);

        tft.setTextSize(3);
        tft.setCursor(x_cursor, y_cursor);
        tft.print(customKey);
        x_cursor = x_cursor + 25;
        tft.setTextColor(ILI9341_WHITE);
      }
    }

    // Print key value to serial monitor
    Serial.print("customKey: ");
    Serial.println(customKey);
    Serial.print("array: ");
    Serial.println(inputInts[intarraypointer - 1]);

  }
}
/*
 * function that takes a char and returns and int returns -1 if it was not a number
 */
int charToint(char key) {
  int val = key - 48;
  if (val < 10) {
    return val;
  } else
    return -1;
}

/*
 * read val and plot curve
 */
void runtest(double xaxis1, double yaxis1, double ox, double oy) {

  double distance = getdistance() - yDistance;
  double load = getload();
  // Serial.println("loadcell is ready");

  //Serial.print("load is: ");
  //Serial.println(load);

  if (load >prevload*0.85) {
    Graph(tft, distance, load, 60, 200, 220, 150, 0, xaxis1, xaxis1/7, 0, yaxis1, yaxis1 / 4, "Distance-Weight curve", "Distance", "Weight", ILI9341_BLUE, ILI9341_RED, ILI9341_YELLOW, ILI9341_WHITE, ILI9341_BLACK, plotbackground);

    //digitalWrite(TFT_CS, HIGH);
    Serial.print(distance);
    Serial.print('\t');
    Serial.println(load);
    prevload = load;
    myFile = SD.open(File_name, FILE_WRITE);
      myFile.print(distance);
      myFile.print(",");
      myFile.println(load);
      myFile.close();
      Serial.println("in graph");
      dataticks++;
  }
  else if (load < prevload*0.4 && dataticks >10){
    aflag = true;
    dataticks = 0;
  }

}
/*
 * draws the stress stain curve 
 */
void runtestStress_strain(double xaxis1, double yaxis1, double ox, double oy) {

  double Strain = (getdistance() - yDistance) / Length_Saved;

  double Stress = getload()*stress_coefficient;

  Serial.println("loadcell is ready");
  if (Stress >prevload*0.85) {
  Graph(tft, Strain, Stress, 60, 200, 220, 150, 0, (xaxis1) / Length_Saved, (xaxis1 / 7) / Length_Saved, 0, (yaxis1)*stress_coefficient, (yaxis1*stress_coefficient) / 4 , "Stress-Strain", "Strain", "Stress", ILI9341_BLUE, ILI9341_RED, ILI9341_YELLOW, ILI9341_WHITE, ILI9341_BLACK, plotbackground);
      myFile = SD.open(File_name, FILE_WRITE);
      myFile.print(Strain);
      myFile.print(",");
      myFile.println(Stress);
      myFile.close();
  Serial.print(Strain);
  Serial.print('\t');
  Serial.println(Stress);
    prevload =  Stress;
  }
    else if (Stress < prevload*0.4 && dataticks >20){
    aflag = true;
    dataticks = 0;
  }

}

/*
 * two functions to get the mean
 */
double getdistance() {
  double for_loop_distance = 0;
  for (int i = 0; i < 4; i++) {
    vl53.dataReady();
    for_loop_distance = for_loop_distance + vl53.distance();

  }
  return for_loop_distance / 4;
}
// the avarage value is enough for the loadcell to imporve the accurcy
double getload() {
  return scale.get_units(4);
}
/*
 * two functions to swap and sort
 */
void swap(double * xp, double * yp) {
  int temp = * xp;
  * xp = * yp;
  * yp = temp;
}
void selectionSort(double arr[]) {
  int i, j, min_idx;

  // One by one move boundary of unsorted subarray
  for (i = 0; i < 11; i++) {

    // Find the minimum element in unsorted array
    min_idx = i;
    for (j = i + 1; j < 11; j++)
      if (arr[j] < arr[min_idx])
        min_idx = j;

    // Swap the found minimum element
    // with the first element
    swap( & arr[min_idx], & arr[i]);
  }
}

/*
 *Credit to https://github.com/KrisKasprzak/Graphing.git for the graphing function
 *
 *
 */

/*
  function to draw a cartesian coordinate system and plot whatever data you want
  just pass x and y and the graph will be drawn
  huge arguement list
  &d name of your display object
  x = x data point
  y = y datapont
  gx = x graph location (lower left)
  gy = y graph location (lower left)
  w = width of graph
  h = height of graph
  xlo = lower bound of x axis
  xhi = upper bound of x asis
  xinc = division of x axis (distance not count)
  ylo = lower bound of y axis
  yhi = upper bound of y asis
  yinc = division of y axis (distance not count)
  title = title of graph
  xlabel = x asis label
  ylabel = y asis label
  gcolor = graph line colors
  acolor = axi ine colors
  pcolor = color of your plotted data
  tcolor = text color
  bcolor = background color
  &redraw = flag to redraw graph on fist call only
*/

void Graph(Adafruit_ILI9341 & d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean & redraw) {

  double ydiv, xdiv;
  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  //static double ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
  //static double oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  double i;
  double temp;
  int rot, newrot;

  if (redraw == true) {

    redraw = false;
    Serial.print("point 1 ox is: ");
    Serial.println(x);
    ox = (x - xlo) * (w) / (xhi - xlo) + gx;
    Serial.print("point 2 ox is: ");
    Serial.println(x);
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
    // draw y scale
    for (i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp = (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        d.drawLine(gx, temp, gx + w, temp, acolor);
      } else {
        d.drawLine(gx, temp, gx + w, temp, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(gx - 40, temp);
      // precision is default Arduino--this could really use some format control
      d.println(i);
    }
    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform

      temp = (i - xlo) * (w) / (xhi - xlo) + gx;
      if (i == 0) {
        d.drawLine(temp, gy, temp, gy - h, acolor);
      } else {
        d.drawLine(temp, gy, temp, gy - h, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(temp, gy + 10);
      // precision is default Arduino--this could really use some format control
      d.println(i);
    }

    //now draw the labels
    d.setTextSize(2);
    d.setTextColor(tcolor, bcolor);
    d.setCursor(gx, gy - h - 30);
    d.println(title);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx, gy + 20);
    d.println(xlabel);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx - 30, gy - h - 10);
    d.println(ylabel);
  }

  //graph drawn now plot the data
  // the entire plotting code are these few lines...
  // recall that ox and oy are initialized as static above
  x = (x - xlo) * (w) / (xhi - xlo) + gx;
  y = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox, oy, x, y, pcolor);
  d.drawLine(ox, oy + 1, x, y + 1, pcolor);
  d.drawLine(ox, oy - 1, x, y - 1, pcolor);
  ox = x;
  oy = y;
  //Serial.print("ox is: ");
  //Serial.println(ox);
  tft.setTextColor(ILI9341_WHITE);

}

/*
  End of graphing functioin
*/

/*
 *
 *
 */

/*
 * The state machine function
 * too long to be explained in a comment
 * Check diagram to understand how it works
 * if that didn't help
 * go back to the hardware&software course :)
 *
 */
void InterfaceState() {
  switch (UI_State) {
  case UI_START:
    UI_State = UI_INT;

    break;
  case UI_INT:
    UI_State = UI_MAINMENU;
    mainmenudraw();
    break;
  case UI_MAINMENU:
    if (aflag) {
      UI_State = UI_CALIBRATE1;
      caldraw();
    } else if (bflag) {
      UI_State = UI_SETTINGS;
      settingsdraw();
    } else if (cflag) {
      UI_State = UI_CHOOSECURVE;
      choosecurvedraw();
    }
    break;
  case UI_CALIBRATE1:
    if (aflag) {
      UI_State = UI_CALIBRATE2;
      scale.set_scale();
      scale.tare();
      inputnum = true;
      calweightdraw();
    } else if (dflag) {
      UI_State = UI_MAINMENU;
      mainmenudraw();
    }
    break;
  case UI_CALIBRATE2:
    if (aflag) {
      calibrateLoadCell();
      UI_State = UI_MAINMENU;
      mainmenudraw();
    }
    break;
  case UI_SETTINGS:
    if (aflag) {
      UI_State = UI_ADJUST_XAXIS;
      x_cursor = 140;
      y_cursor = 130;
      inputnum = true;
      xaxisdraw();
    } else if (bflag) {
      UI_State = UI_ADJUST_YAXIS;
      x_cursor = 140;
      y_cursor = 130;
      inputnum = true;
      yaxisdraw();
    }
    else if (dflag){
      UI_State = UI_MAINMENU;
      mainmenudraw();
    }
    break;
  case UI_ADJUST_XAXIS:
    if (aflag) {
      xaxis_Saved_value = getarrayval();
      EEPROM.put(130, xaxis_Saved_value);
      UI_State = UI_SETTINGS;
      settingsdraw();
      inputnum = false;
    } else if (dflag) {
      UI_State = UI_SETTINGS;
      settingsdraw();
      inputnum = false;
      intarraypointer =0;
    }
    break;
  case UI_ADJUST_YAXIS:
    if (aflag) {
      yaxis_Saved_value = getarrayval();
      EEPROM.put(160, yaxis_Saved_value);
      UI_State = UI_SETTINGS;
      settingsdraw();
      inputnum = false;
    } else if (dflag) {
      UI_State = UI_SETTINGS;
      settingsdraw();
      inputnum = false;
      intarraypointer =0;
    }
    break;
  case UI_CHOOSECURVE:
    if (aflag) {
      starttestdraw();
      DW_CURVE = true;
      SS_CURVE = false;
      UI_State = UI_STARTTEST;
    } else if (bflag) {
      lengthdraw();
      SS_CURVE = true;
      DW_CURVE = false;
      inputnum = true;
      UI_State = UI_GET_LENGTH;
      x_cursor = 140;
      y_cursor = 130;
    } else if (dflag) {
      UI_State = UI_MAINMENU;
      mainmenudraw();

    }
    break;
  case UI_STARTTEST:
    if (aflag && DW_CURVE && !SS_CURVE) {
      EEPROM.get(130, xaxis);
      EEPROM.get(160, yaxis);
      plotbackground = true;
      tft.fillScreen(ILI9341_BLACK);
      yield();
      scale.tare();
      yDistance = getdistance();

      EEPROM.get(0, loadcalval);
      Serial.print("loadcalval is :");
      Serial.println(loadcalval);
      scale.set_scale(loadcalval);
      DateTime now = rtc.now();
      File_name ="Test_Distance_weight_"+String(now.month(),DEC)+"_"+String(now.day(),DEC)+"_"+String(now.year(),DEC)+"_"+String(now.hour(),DEC)+"_"+String(now.minute(),DEC)+"_"+String(now.second(),DEC)+".txt";
      myFile = SD.open(File_name, FILE_WRITE);
      myFile.print("Distance(mm)");
      myFile.print(",");
      myFile.println("Weight (g)");
      myFile.close();


      UI_State = UI_DISTANCEWEIGHT;
    } 
    else if (aflag && !DW_CURVE && SS_CURVE) {
      EEPROM.get(130, xaxis);
      EEPROM.get(160, yaxis);
      plotbackground = true;
      tft.fillScreen(ILI9341_BLACK);
      yield();
      scale.tare();
      yDistance = getdistance();

      EEPROM.get(0, loadcalval);
      Serial.print("loadcalval is :");
      Serial.println(loadcalval);
      scale.set_scale(loadcalval);
      area_sample = width_Saved*Thickness_Saved;
      stress_coefficient = 9.81/(area_sample);
      DateTime now = rtc.now();
      File_name ="Test_Stress_Strain_"+String(now.month(),DEC)+"_"+String(now.day(),DEC)+"_"+String(now.year(),DEC)+"_"+String(now.hour(),DEC)+"_"+String(now.minute(),DEC)+"_"+String(now.second(),DEC)+".txt";
     myFile = SD.open(File_name, FILE_WRITE);
      myFile.print("Strain (-)");
      myFile.print(",");
      myFile.println("Stress (MPA)");
      myFile.close();
      UI_State = UI_STRESSSTRAIN;
    }
    break;
  case UI_GET_LENGTH:
    if (aflag) {
      Length_Saved = getarrayval();
      UI_State = UI_GET_THICKNESS;
      thicknessdraw();
       x_cursor = 140;
      y_cursor = 130;
    }
    break;
  case UI_GET_THICKNESS:
    if (aflag) {
      Thickness_Saved = getarrayval();
      UI_State = UI_GET_WIDTH;
      widthdraw();
      intarraypointer =0;
       x_cursor = 140;
      y_cursor = 130;
    }
    break;
  case UI_GET_WIDTH:
    if (aflag) {
      width_Saved = getarrayval();
      starttestdraw();
      UI_State = UI_STARTTEST;
      intarraypointer =0;
      x_cursor = 140;
      y_cursor = 130;
    }
    break;
  case UI_DISTANCEWEIGHT:
    runtest(xaxis, yaxis, ox, oy);
    if (aflag) {
      filenamedraw();
      UI_State = UI_FILENAME;
            inputnum = false;
    }
    break;
  case UI_STRESSSTRAIN:
    runtestStress_strain(xaxis, yaxis, ox, oy);
    if (aflag) {
      filenamedraw();
      UI_State = UI_FILENAME;
    }
    break;
  case UI_FILENAME:
    if (aflag) {
      mainmenudraw();
      UI_State = UI_MAINMENU;
      DW_CURVE = false;
      SS_CURVE = false;
    }

  }
}
/*
 * end of State machine
 */

/*
Function that does the load cell calibration
by reading raw data and diving it by the actual mass used
to give u the calibration factor

*/
void calibrateLoadCell() {

  double known_mass = getarrayval();
  Serial.print("known value: ");
  Serial.println(known_mass);
  double rawvalue = scale.get_units(10);
  Serial.print("raw value: ");
  Serial.println(rawvalue);
  double calvalue = rawvalue / known_mass;
  Serial.print("calvalue: ");
  Serial.println(calvalue);
  EEPROM.put(0, calvalue);
  // reset the scale to 0
}

/*
Function that gets the value currently stored in the array
*/

double getarrayval(){
  int arrSize = intarraypointer;
  //Serial.print("arrSize: ");
  //Serial.println(arrSize);
  double sum = 0.0;
  for (int i = 0; i < arrSize; i++) {
    /* Serial.println("----------------------"); */

    double tmp = (inputInts[i] * pow(10, arrSize - i - 1));
    sum = sum + tmp;
  }
  //Serial.print("getArrayVal: ");
  Serial.println(sum);
  double result = round(sum);
  //Serial.print("result: ");
  Serial.println(result);
  intarraypointer = 0;
  return result;

}
