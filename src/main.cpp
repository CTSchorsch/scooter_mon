#include <Arduino.h>
#include "heltec.h"
#include "oled/OLEDDisplayUi.h"
#include "images.h"
#include "driver/rtc_io.h"
#include "Button2.h"
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include "Wire.h"
#include <LSM303.h>
#include "ArduinoNvs.h"

LSM303 compass;

#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6
#define MY_ID       14
#define MAXBATT                 4000    // The default Lipo is 4200mv when the battery is fully charged.
#define BAT_2_3                 3750    // Point where start light sleep
#define BAT_1_3                 3550    // Point where start light sleep
#define MINBATT                 3200    // The default Lipo is 3200mv when the battery is empty...this WILL be low on the 3.3v rail specs!!!

#define VOLTAGE_DIVIDER         3.20    // Lora has 220k/100k voltage divider so need to reverse that reduction via (220k+100k)/100k on vbat GPIO37 or ADC1_1 (early revs were GPIO13 or ADC2_4 but do NOT use with WiFi.begin())
#define DEFAULT_VREF            1100    // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE            500     // Battery sample rate in ms
#define VBATT_SMOOTH            50      // Number of averages in sample
#define ADC_READ_STABILIZE      5       // in ms (delay from GPIO control and ADC connections times)
#define VBATT_GPIO              21      // Heltec GPIO to toggle VBatt read connection ... WARNING!!! This also connects VEXT to VCC=3.3v so be careful what is on header.  Also, take care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH

uint16_t Sample();
esp_adc_cal_characteristics_t *adc_chars;

extern Heltec_ESP32 Heltec;
OLEDDisplayUi ui( Heltec.display );
Button2 But0 = Button2(GPIO_NUM_0,INPUT_PULLUP,false,true);

struct dataPackage {
		float avgMotorCurrent;
		float avgInputCurrent;
		float dutyCycleNow;
		long rpm;
		float inpVoltage;
		float ampHours;
		float ampHoursCharged;
		long tachometer;
		long tachometerAbs;
		float tempMosfet;
		float tempMotor;
		uint8_t error;
		uint8_t id;
	} vescData;

uint16_t   rx_wd = 0;
bool isConnected = false;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
unsigned int isCalibrated = 0;
uint8_t menuLevel=0;
unsigned long tot_start, tot_end;
bool isRunning=false;

/***************************************************************************/
void calibrateCompass() {
  long now = millis();

  while (now+30000>millis()) {
    compass.read();
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);
    delay(100);
  }
}

/***************************************************************************/
// Poll the proper ADC for VBatt on Heltec Lora 32 with GPIO21 toggled
uint16_t ReadVBatt() {
  uint16_t reading = 666;

  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
  pinMode(ADC1_CHANNEL_1, OPEN_DRAIN);        // ADC GPIO37
  reading = adc1_get_raw(ADC1_CHANNEL_1);
  pinMode(ADC1_CHANNEL_1, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider)
  uint16_t voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);  
  voltage*=VOLTAGE_DIVIDER;
  return voltage;
}

//  Use a buffer to average/sample ADC
uint16_t Sample() {
  static uint8_t i = 0;
  static uint16_t samp[VBATT_SMOOTH];
  static int32_t t = 0;
  static bool f = true;
  if(f){ for(uint8_t c=0;c<VBATT_SMOOTH;c++){ samp[c]=0; } f=false; }   // Initialize the sample array first time
  t -= samp[i];   // doing a rolling recording, so remove the old rolled around value out of total and get ready to put new one in.
  if (t<0) {t = 0;}
  // ADC read
  uint16_t voltage = ReadVBatt();
  samp[i]=voltage;
  #if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("ADC Raw Reading[%d]: %d", i, voltage);
  #endif
  t += samp[i];
  if(++i >= VBATT_SMOOTH) {i=0;}
  uint16_t s = round(((float)t / (float)VBATT_SMOOTH));
  return s;
}

/***************************************************************************/
void batOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
  uint16_t voltage = Sample();
  
  if (voltage < BAT_1_3) {
	  display->drawXbm(108, 0, BAT_width, BAT_height, BAT_EMPTY_bits);
  } else if (voltage < BAT_2_3) {
    display->drawXbm(108, 0, BAT_width, BAT_height, BAT_1_3_bits);
  } else if (voltage < MAXBATT) {
    display->drawXbm(108, 0, BAT_width, BAT_height, BAT_2_3_bits);
  } else {
    display->drawXbm(108, 0, BAT_width, BAT_height, BAT_bits);
  }
}

void connectedOverlay(OLEDDisplay *display, OLEDDisplayUiState *state) {
  if (isConnected) {
    display->drawXbm(0,0, WIFI_width, WIFI_height, WIFI_bits);
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(15,0,(String)vescData.id);
  }
}

OverlayCallback overlays[] = { batOverlay, connectedOverlay };
int overlayCount = 2;

void drawFrame1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  
  float batLevel;
  int rpm = vescData.rpm/24;

  //10s: Max 42V, min 32V -> 10V Hub
  batLevel= ((vescData.inpVoltage-32)/10)*100;

  display->setFont(ArialMT_Plain_24);
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  if (rpm<0) rpm = 0;
  display->drawString(x+80 ,y+20,  (String)rpm);
  display->setFont(ArialMT_Plain_16);
  display->drawString(x+125 ,y+30, "rpm");
  display->setFont(ArialMT_Plain_16);
  display->drawString(x+12 ,y+47, "M");
  display->drawXbm(x+16, y+51, BAT_width, BAT_height, BAT_bits);
  display->drawProgressBar(x+40,y+52,85,6,(uint8_t)(batLevel));
}

void drawFrame2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x,y+16 ,"Tpcb");
  display->drawString(x,y+32 ,"Tmot");
  display->drawString(x,y+48 ,"Vin");
  display->setFont(ArialMT_Plain_16);
  display->drawString(x+30, y+10, (String)vescData.tempMosfet+" °C");
  display->drawString(x+30, y+26, (String)vescData.tempMotor+" °C");
  display->drawString(x+30, y+42, (String)vescData.inpVoltage+" V");

}

void drawFrame3(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  char buff[10];

  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x,y+16 ,"Ibat");
  display->drawString(x,y+32 ,"Imot");
  display->drawString(x,y+48 ,"Pwr");
  sprintf(buff,"%.0f W", vescData.inpVoltage*vescData.avgInputCurrent);
  display->setFont(ArialMT_Plain_16);
  display->drawString(x+30, y+10, (String)vescData.avgInputCurrent+" A");
  display->drawString(x+30, y+26, (String)vescData.avgMotorCurrent+" A");
  display->drawString(x+30, y+42, (String)buff);

}

String mkCompassString(uint16_t course) {
  if (course%5==0) {
    if (course==0) return "N";
    else if (course==90) return "E";
    else if (course==180) return "S";
    else if (course==270) return "W";
    else  return (String)course;
  } else return ".";
}

float mittelWert(float newVal) {
  static float buff[16];
  uint8_t i=0;
  float sum = 0;

  for (i=1; i < 16; i++) {
    sum+=buff[i];
    buff[i-1]=buff[i];
  }
  buff[i-1]=newVal;
  sum+=newVal;

  return sum/16;
}

void drawFrame4(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  int xoff;
  String centerstr;
  int newcompass,realcompass;
  char buff[20];
  long tots = (tot_end-tot_start)/1000;
  
  display->drawHorizontalLine(x+0,y+15,128);
  display->drawVerticalLine(x+64,y+15,5);
  display->drawHorizontalLine(x+0,y+43,128);
  display->drawVerticalLine(x+64,y+38,5);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_16);
  sprintf(buff,"%02d:%02d",(int)tots/60,(int)tots%60);
  display->drawString(x+100,y+48,(String)buff);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x+30,y+54,"> calibrate");
  display->setFont(ArialMT_Plain_16);

  xoff=64;
  realcompass=compass.heading((LSM303::vector<int>){-1, 0, 0});
  newcompass=mittelWert(realcompass);
  if (abs(newcompass-realcompass) > 4) newcompass=realcompass;
  //newcompass=compass.heading();
  centerstr = mkCompassString(newcompass);
  display->drawString(x+xoff, y+21, centerstr);
  xoff+=display->getStringWidth(centerstr)/2;
  //Display rechts auffüllen
  while(xoff < 128) {
    newcompass++;
    if (newcompass > 359) newcompass = 0;
    centerstr = mkCompassString(newcompass);
    xoff+=display->getStringWidth(centerstr)/2;
    display->drawString(x+xoff, y+21, centerstr);
    xoff+=display->getStringWidth(centerstr)/2;
  }
  //Display links auffüllen
  xoff=64;
  realcompass=compass.heading((LSM303::vector<int>){-1, 0, 0});
  newcompass=mittelWert(realcompass);
  if (abs(newcompass-realcompass) > 4) newcompass=realcompass;
  //newcompass=compass.heading();
  centerstr = mkCompassString(newcompass);
  xoff-=display->getStringWidth(centerstr)/2;
  while(xoff > 0) {
    newcompass--;
    if (newcompass < 0) newcompass = 359;
    centerstr = mkCompassString(newcompass);
    xoff-=display->getStringWidth(centerstr)/2;
    display->drawString(x+xoff, y+21, centerstr);
    xoff-=display->getStringWidth(centerstr)/2;
  }
}

void drawFrame5(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  static  long calibStart=millis();

  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(x,y+10 ,"Calibrate Compass");
  display->drawString(x,y+20 ,"Rotate X-Axis");
  compass.read();
  running_min.x = min(running_min.x, compass.m.x);
  running_min.y = min(running_min.y, compass.m.y);
  running_min.z = min(running_min.z, compass.m.z);
  running_max.x = max(running_max.x, compass.m.x);
  running_max.y = max(running_max.y, compass.m.y);
  running_max.z = max(running_max.z, compass.m.z);
  if (calibStart+10000<millis()) {
    display->drawString(x,y+30,"Rotate Y-Axis");
  }
  if (calibStart+20000<millis()) {
    display->drawString(x,y+40,"Rotate Z-Axis");
  }
  if (calibStart+30000<millis()) {
    calibStart=0;
    isCalibrated=1;
    compass.m_min = running_min;
    compass.m_max = running_max;
    NVS.setInt("minx",running_min.x);
    NVS.setInt("miny",running_min.y);
    NVS.setInt("minz",running_min.z);
    NVS.setInt("maxx",running_max.x);
    NVS.setInt("maxy",running_max.y);
    NVS.setInt("maxz",running_max.z);    
    NVS.setInt("calib",1);
    menuLevel--;
    ui.switchToFrame(3); 
  } 
}

FrameCallback frames[] = { drawFrame1, drawFrame2, drawFrame3, drawFrame4, drawFrame5 };
int frameCount = 5;


void onReceive(int packetSize) {
  
  uint8_t data[packetSize];
  uint8_t idx=0;
  while (LoRa.available()) {
    data[idx++] = (uint8_t)LoRa.read();
  }
  Serial.printf("len: %d\n",packetSize);
  if ( ((dataPackage *)data)->id == MY_ID ) {
    rx_wd = 0;
    isConnected = true;
    if (sizeof(dataPackage) == packetSize) {
      memcpy((void *)&vescData,data,sizeof(data));
      if (vescData.rpm > 1000 && !isRunning) {
        tot_start=millis(); 
        tot_end=tot_start;
        isRunning = true;
      } else if (vescData.rpm < 100 && isRunning) {
        tot_end = millis();
        isRunning = false;
      } else if (isRunning) {
        tot_end = millis();
      }
    } else {
      Serial.println("dataPacket size missmatch");
    }
  }
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}
/**************************************************************************************************/
void btnHandler(Button2 &btn)
{
  OLEDDisplayUiState *state = ui.getUiState();

  rx_wd = 0;
  if (menuLevel > 0) return;
  if (btn == But0) {
    if (state->currentFrame < 3)
      ui.nextFrame();
    else 
      ui.transitionToFrame(0);
  }
}

void btnLongHandler(Button2 &btn) 
{
   OLEDDisplayUiState *state = ui.getUiState();

  if (state->currentFrame == 3) {
    menuLevel++;
    isCalibrated = 0;
    NVS.setInt("calib",0);
    ui.switchToFrame(4);
  }
}

/**************************************************************************************************/
void setup() {

  pinMode(21, OUTPUT);
  pinMode(37,OPEN_DRAIN);
  digitalWrite(21,0);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0,0); //1 = High, 0 = Low

  But0.setTapHandler(btnHandler);
  But0.setClickHandler(btnHandler);
  But0.setLongClickHandler(btnLongHandler);
  But0.setLongClickTime(300);

  NVS.begin();

  Heltec.begin(true /*DisplayEnable Enable*/, true /*LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.onReceive(onReceive);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setSpreadingFactor(7);
  //LoRa.setCodingRate4(5);
  LoRa.enableInvertIQ();
  LoRa.receive();

  ui.setTargetFPS(20);
  ui.setFrames(frames, frameCount);
  ui.setOverlays(overlays, overlayCount);
  ui.disableIndicator();
  ui.disableAutoTransition();
  ui.disableAllIndicators();
  ui.init();
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_6);

   for (uint8_t i = 0;i < VBATT_SMOOTH;i++) {
    Sample();
  }
  pinMode(VBATT_GPIO,OUTPUT);
  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize

  Wire1.begin(23,22);
  compass.init();
  compass.enableDefault();
  isCalibrated = NVS.getInt("calib");
  if (isCalibrated) {
    running_min.x = NVS.getInt("minx");
    running_min.y = NVS.getInt("miny");
    running_min.z = NVS.getInt("minz");
    running_max.x = NVS.getInt("maxx");
    running_max.y = NVS.getInt("maxy");
    running_max.z = NVS.getInt("maxz");
    compass.m_min = running_min;
    compass.m_max = running_max;
  } else {
     ui.switchToFrame(4);
  }
}

void loop() {
  int remainingTimeBudget = ui.update();

  But0.loop();
  if (remainingTimeBudget > 0) {
    if (runEvery(200)) {
      compass.read();
      rx_wd++;
      if (rx_wd < 10) {
      } else {
        isConnected = false;
        if (rx_wd > 500) {
          Heltec.VextOFF();
          Heltec.LoRa.sleep();
          Heltec.display->sleep();
          rtc_gpio_deinit(GPIO_NUM_0);
          rtc_gpio_pulldown_en(GPIO_NUM_0);
          esp_deep_sleep_start();
        }
      }

    } else {
      delay(remainingTimeBudget);
    }
	}
}
