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

// Include the battery status BQ27441
#include <SparkFunBQ27441.h>
//
const byte BQ27441_address = 0x55;
const unsigned int BATTERY_CAPACITY = 650; // e.g. 850mAh battery
unsigned int bat_soc, bat_volts, bat_fullCapacity, bat_capacity;
int bat_current, bat_power, bat_health;
bool hasBattery=false;

// Include the TFT library
#include <TFT_eSPI.h>

// Define the TFT screen
TFT_eSPI tft;

// IR Array (MLX90640) 
#include "MLX90640_API.h"
#include "MLX9064X_I2C_Driver.h"

const byte MLX90640_address = 0x33;  //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8  //Default shift for MLX90640 in open air

int8_t MLX9064XStatus;
uint16_t eeMLX90640[832];     // 768(32*25)+64
float mlx90640To[768];        // pixel 32*25
uint16_t mlx90640Frame[834];  // 832+2
paramsMLX90640 mlx90640;


// Time flag for calculate Flush rate
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
  tft.setTextSize(2);

  if (isConnected((uint8_t)BQ27441_address) == false) // begin() will return true if communication is successful
  {
    // If communication fails, print an error message and loop forever.
    Terminal.println("Error: Unable to communicate with BQ27441.");
    Terminal.println("  Check wiring and try again.");
    Terminal.println("  (Battery must be plugged into Battery Babysitter!)");

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("Battery Not Initialised!", 5, 40);
  }
  else{
    hasBattery = true;
    Terminal.println("Connected to BQ27441!");
    tft.drawString("BQ27441 online!", 5, 40);
  }
  lipo.setCapacity(BATTERY_CAPACITY);

  if (isConnected((uint8_t)MLX90640_address) == false) {
    Terminal.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("MLX90640 not detected", 5, 0);
    tft.drawString("Please check wiring.", 5, 20);
    while (1)  ;
  }
  else{
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("MLX90640 online!", 5, 0);
  }
  
  //Get device parameters - We only have to do this once
  MLX9064XStatus = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (MLX9064XStatus != 0) {
    Terminal.println("Failed to load system parameters");
  }

  MLX9064XStatus = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (MLX9064XStatus != 0) {
    Terminal.println("Parameter extraction failed");
  }

  //Once params are extracted, we can release eeMLX90640 array

  //Set refresh rate
  //A rate of 0.5Hz takes 4Sec per reading because we have to read two frames to get complete picture
  //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.5Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 1Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 2Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 4Hz effective - Works
  MLX90640_SetRefreshRate(MLX90640_address, 0x04);  //Set rate to 8Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 16Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 32Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 64Hz effective - fails

  // Get the cutoff points:
  Getabcd();

  // Menu setup:
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Waiting to setup menu.....", 5, 60);
  delay(3000);
  tft.fillScreen(TFT_BLACK); // Clear screen
}

void loop() {

  // Draw the options menu:
  int start_x = 64-48;
  int start_y = 20;
  int w = 6;
  int h = 6;

  int x = start_x +31 * 6;      // start_x = 64, 0 is left
  int y = start_y ;             // start_y = 20 h = 6
  uint32_t c = TFT_BLUE;

  draw_menu(start_x, start_y, w, h); // 64, 20, 6, 6

  TimeFlag[0] = millis(); // log startTime

  for (byte x = 0; x < 2; x++) {
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    float tr = Ta - TA_SHIFT;  //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  TimeFlag[1] = millis(); // log stopReadTime
  for (int i = 0; i < 768; i++)
  {
    // Display a simple image version of the collected data (array) on the screen:
    // Define the color palette:

    // float _test= 21+i*0.03;
    // c = GetColor(_test);
    
    c = GetColor(mlx90640To[i]);
    // Draw image pixels (rectangles):
    tft.fillRect(x, y, 6, 6, c);

    x = x - 6;
    if (i % 32 == 31)
    {
      x = start_x + 31 * 6 ;
      y = y + h;
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


  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Rate.R", start_x+210, start_y);
  tft.drawString(String(ReadRate,2), start_x+240, start_y+10);
  tft.drawString(" FPS", start_x+270, start_y+10);

  tft.drawString("Rate.RP", start_x+210, start_y+20);
  tft.drawString(String(ReadpusPrintRate,2), start_x+240, start_y+30);
  tft.drawString(" FPS", start_x+270, start_y+30);

  if(hasBattery){
    showBatteryStatus();
    tft.drawString("SoC. "+String(bat_soc)+" %", start_x+210, start_y+120);
    tft.drawString("Bat.mV "+String(bat_volts)+" mV", start_x+210, start_y+130);
    tft.drawString("Bat.mA "+String(bat_current)+" mA", start_x+210, start_y+140);
  }
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

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected(uint8_t _addr) {
  Wire.beginTransmission(_addr);
  if (Wire.endTransmission() != 0) {
    return (false);  //Sensor did not ACK
  }
  return (true);
}

void showBatteryStatus(){
  bat_soc = lipo.soc();                  // Read state-of-charge (%)
  bat_volts = lipo.voltage();            // Read battery voltage (mV)
  bat_current = lipo.current(AVG);                // Read average current (mA)
  bat_fullCapacity = lipo.capacity(FULL);// Read full capacity (mAh)
  bat_capacity = lipo.capacity(REMAIN);  // Read remaining capacity (mAh)
  bat_power = lipo.power();                       // Read average power draw (mW)
  bat_health = lipo.soh();                        // Read state-of-health (%)

  // // Now print out those values:
  // String toPrint = String(bat_soc) + "% | ";
  // toPrint += String(bat_volts) + " mV | ";
  // toPrint += String(bat_current) + " mA | ";
  // toPrint += String(bat_fullCapacity) + " / ";
  // toPrint += String(bat_capacity) + " mAh | ";
  // toPrint += String(bat_power) + " mW | ";
  // toPrint += String(bat_health) + "%";
  // Terminal.println(toPrint);
}