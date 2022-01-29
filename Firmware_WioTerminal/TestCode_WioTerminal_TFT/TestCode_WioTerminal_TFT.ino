/* This project
 *
 *
 *
 * Reference:
 * 1. https://www.instructables.com/Irrigation-Level-Assessment-by-Thermal-Imaging-W-T
 * 2. https://github.com/S10143806H/Irrigation-Level-based-on-MLX9064x
 * 
 * Author: Kyle Liu
*/
// Set Terminal via Serial
#define Terminal Serial
#define DEBUG true

#include <Wire.h>

// Include the TFT library
#include <TFT_eSPI.h>

// Define the TFT screen
TFT_eSPI tft;


// 
long TimeFlag[3]; // start, stop of get thermopile, stop of TFT print

//  Define the maximum and minimum temperature values:
uint16_t MinTemp = 21;
uint16_t MaxTemp = 45;

// Define the data holders:
// String MLX90640_data = "";
byte red, green, blue;
float a, b, c, d;


void setup() {
  // Initialize Terminal Monitor
  Terminal.begin(115200);

  // Initialize the I2C
  Wire.begin();
  Wire.setClock(400000);

  // Initialize the TFT screen
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);


  // Get the cutoff points:
  Getabcd();

  // Menu setup:
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Waiting to setup menu.....", 5, 40);
  delay(3000);
  tft.fillScreen(TFT_BLACK); // Clear screen
}

void loop() {  
  // Draw the options menu:
  int start_x = 64-48;
  int start_y = 20;
  int w = 6;
  int h = 6;

  int x = start_x;            // start_x = 64
  int y = start_y + (h * 23); // start_y = 20 h = 6
  uint32_t c = TFT_BLUE;

  draw_menu(start_x, start_y, w, h); // 64, 20, 6, 6

  TimeFlag[0] = millis(); // log startTime
  delay(100);

  TimeFlag[1] = millis(); // log stopReadTime
  for (int i = 0; i < 768; i++)
  {
    // Display a simple image version of the collected data (array) on the screen:
    // Define the color palette:
    float _test= 21+i*0.03;

    c = GetColor(_test);
    // Draw image pixels (rectangles):
    tft.fillRect(x, y, 6, 6, c);
    x = x + 6;
    if (i % 32 == 31)
    {
      x = start_x;
      y = y - h;
    }
  }
  
  TimeFlag[2] = millis(); // log stopPrintTime
  float ReadRate = 1000.0 / (TimeFlag[1] - TimeFlag[0]);
  float ReadpusPrintRate = 1000.0 / (TimeFlag[2] - TimeFlag[0]);

  Terminal.println("\r\n====================");
  Terminal.print("Read rate: ");
  Terminal.print(ReadRate, 2);
  Terminal.println(" Hz");
  Terminal.print("Read plus print rate: ");
  Terminal.print(ReadpusPrintRate, 2);
  Terminal.println(" Hz");


  tft.drawString("Rate.R", start_x+210, start_y);
  tft.drawString(String(ReadRate,2), start_x+240, start_y+10);
  tft.drawString(" Hz", start_x+280, start_y+10);

  tft.drawString("Rate.RP", start_x+210, start_y+20);
  tft.drawString(String(ReadpusPrintRate,2), start_x+240, start_y+30);
  tft.drawString(" Hz", start_x+280, start_y+30);

  delay(1000);
}

void Getabcd()
{
  // Get the cutoff points based on the given maximum and minimum temperature values.
  a = MinTemp + (MaxTemp - MinTemp) * 0.2121;
  b = MinTemp + (MaxTemp - MinTemp) * 0.3182;
  c = MinTemp + (MaxTemp - MinTemp) * 0.4242;
  d = MinTemp + (MaxTemp - MinTemp) * 0.8182;
}

void draw_menu(int start_x, int start_y, int w, int h){
  // Draw the border:
  int offset = 10;
  tft.drawRoundRect(start_x - offset, start_y - offset, (2 * offset) + (w * 32), (2 * offset) + (h * 24), 10, TFT_WHITE);


  // Draw options:
  int x_c = 52;
  int x_s = x_c - 7;
  int y_c = 210;
  int y_s = y_c - 11;
  int sp = 72;
  tft.setTextSize(1);

  /* logo U for status Excessive */
  tft.fillCircle(x_c, y_c, 20, TFT_WHITE);
  tft.drawChar(x_s, y_s, 'U', TFT_BLACK, TFT_WHITE, 3);
  tft.drawString("Excessive", x_c - 25, y_c - 33);

  /* logo L for status Sufficient */
  tft.fillCircle(x_c + sp, y_c, 20, TFT_WHITE);
  tft.drawChar(x_s + sp, y_s, 'L', TFT_BLACK, TFT_WHITE, 3);
  tft.drawString("Sufficient", x_c + sp - 28, y_c - 33);

  /* logo R for status Moderate */
  tft.fillCircle(x_c + (2 * sp), y_c, 20, TFT_WHITE);
  tft.drawChar(x_s + (2 * sp), y_s, 'R', TFT_BLACK, TFT_WHITE, 3);
  tft.drawString("Moderate", x_s + (2 * sp) - 16, y_c - 33);

  /* logo D for status Dry */
  tft.fillCircle(x_c + (3 * sp), y_c, 20, TFT_WHITE);
  tft.drawChar(x_s + (3 * sp), y_s, 'D', TFT_BLACK, TFT_WHITE, 3);
  tft.drawString("Dry", x_c + (3 * sp) - 8, y_c - 33);
}

uint16_t GetColor(float val){
  /*
    equations based on
    http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html
  */

  // Assign colors to the given temperature readings:
  // R:
  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

  // G:
  if ((val > MinTemp) & (val < a)){
    green = constrain(255.0 / (a - MinTemp) * val - (255.0 * MinTemp) / (a - MinTemp), 0, 255);
  }
  else if ((val >= a) & (val <= c)){
    green = 255;
  }
  else if (val > c){
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a)){
    green = 0;
  }

  // B:
  if (val <= b){
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d)){
    blue = 0;
  }
  else if (val > d){
    blue = constrain(240.0 / (MaxTemp - d) * val - (d * 240.0) / (MaxTemp - d), 0, 240);
  }

  // Utilize the built-in color mapping function to get a 5-6-5 color palette (R=5 bits, G=6 bits, B-5 bits):
  return tft.color565(red, green, blue);
}