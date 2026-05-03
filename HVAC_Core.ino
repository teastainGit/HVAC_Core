#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <M5Unified.h>
#include <M5GFX.h>
#include "Free_Fonts.h"
#include <Wire.h>

#include <esp_now.h>
#include <WiFi.h>
#include "M5_STHS34PF80.h"

#define RightButton 37   //37 is  RHS button
#define MiddleButton 38  //37 is  Mid button
#define LeftButton 39    //37 is  LHS button

Adafruit_BME680 bme;  // I2C
M5GFX Lcd;
String thisfile = "HVAC_Core";  //with TMOS PIR

float setpoint = 26.0;
int angleMin = 10;
int angleMax = 50;

//Vent temp from damper expresses as 4 digits (2450 is 24.5)
int posDisplay;
int posDisplay_glide = 0;
int x = 1;
int vent_glide = 15;
int room_tempAVG_old_intX10;
int room_tempAVG_intX10;
int vent_tempAVG_old_intX10;
int vent_tempAVG_intX10;
int humidityAVG_old_int;
int humidityAVG_int;
int posDisplay_old;
int gas_resistance = 100;
int gas_resistanceAVG_old = 100;
int gas_resistanceAVG = 100;
int TimeoutResets = 0;
unsigned long interrupted = 0;
unsigned long lastReadTempAVG = 0;
unsigned long longtime = 0;
unsigned int data[6];
float tempC = 26;  //ENV Pro out back!
float humidity;
float heater1_hysteresis = -0.2;
float heater2_hysteresis = -0.2;
float HiVent_hysteresis;
float vent_hysteresis = 0;
float room_temp = 25.5;
float vent_temp = 22.5;
float room_temp_read[5] = { 25, 25, 25, 25, 25 };          //array
float vent_temp_read[5] = { 25, 25, 25, 25, 25 };          //array
float humidity_read[5] = { 25, 25, 25, 25, 25 };           //array
int gas_resistance_read[5] = { 100, 100, 100, 100, 100 };  //array;
float VOC = 2;
float room_tempAVG = 24;
float vent_tempAVG = 24;
float humidityAVG;
float offset = 0;
bool hot = false;
bool heat_available = false;
bool ticktock = true;
bool enable = false;
bool warmer = false;
bool cooler = false;
bool Enable_old;
bool Warmer_old;
bool Cooler_old;
bool vent = false;
bool flag;
bool do_once = true;
bool Lampflag = true;
bool HiVent;
bool Heater1_old;
bool Heater2_old;
bool flasher;
bool vent_old;
bool triggered;
bool motion;
bool presence;
bool pres_flag_1shot = false;
bool too_long;
bool Leave;

M5_STHS34PF80 TMOS;
unsigned long trigger_time;
uint8_t motionHysteresis = 0;
int16_t motionVal = 0, presenceVal = 0;
uint16_t motionThresholdVal = 0, precenceThresholdVal = 0;
sths34pf80_gain_mode_t gainMode;


//Stamp-PICO                94:   b9:   7e:   90:   15:   dc
uint8_t VentAddress[] = { 0x94, 0xB9, 0x7E, 0xFF, 0x15, 0xDC };
//Address of Heater Stamp     ac:   0b:   fb:   6f:   2f:   84
uint8_t HeaterAddress[] = { 0xAC, 0xFF, 0xFB, 0x6F, 0x2F, 0x84 };
//Address of Lamp StampC3 MAC: 34:b4:72:12:8e:cc
uint8_t LampAddress[] = { 0x34, 0xB4, 0x72, 0x12, 0xFF, 0xCC };
//Address of Lamp StampS3 MAC: dc:54:75:c8:ad:68
uint8_t Lamp2Address[] = { 0xFF, 0x54, 0x75, 0xC8, 0xAD, 0x68 };
//adresss of MCR 34:b4:72:10:9b:20
uint8_t MCRAddress[] = { 0x34, 0xB4, 0x72, 0x10, 0x9B, 0xFF };

typedef struct VentData_message {
  float value;
} VentData_message;
VentData_message Position;
VentData_message VentTemp;

typedef struct HeaterRelay_message {
  bool Heater1;
  bool Heater2;
} HeaterRelay_message;
HeaterRelay_message Heaters;

typedef struct Lamp_message {  //Original C3
  bool State;
} Lamp_message;
Lamp_message Lamp;

typedef struct Lamp2_message {  //NEW S3
  bool State;
} Lamp2_message;
Lamp2_message Lamp2;

typedef struct MCR_message {  //NEW S3
  bool State;
} MRC_message;
MCR_message MCR;

esp_now_peer_info_t peerInfo;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   setup begins
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  delay(100);
  Wire.begin(21, 22);
  M5.Power.begin();
  delay(100);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    ;
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms

  Serial.println("  ");
  Serial.println("  ");
  Serial.print("This Sketch Filename is '");
  Serial.print(thisfile);
  Serial.println("'");
  Serial.print("setpoint =");
  Serial.println(setpoint, 1);
  Serial.println("  ");
  delay(100);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.println("  ");
  M5.Lcd.print("This Sketch Filename is ");
  M5.Lcd.println("  ");
  delay(100);
  M5.Lcd.setTextColor(CYAN);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.print(thisfile);
  delay(2000);
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@###########################
  M5.Lcd.setTextSize(1);
  M5.Lcd.setBrightness(1);
  M5.Lcd.setFreeFont(FSS12);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(3, 10);
  M5.Lcd.print("Room Temp =");
  M5.Lcd.setCursor(3, 50);
  M5.Lcd.print("Vent Temp =");
  M5.Lcd.setCursor(3, 90);
  M5.Lcd.print("Position  =");
  M5.Lcd.setCursor(3, 130);
  M5.Lcd.print("Humidity  =");

  M5.Lcd.setCursor(3, 170);
  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextColor(CYAN);
  M5.Lcd.print("VOC level =        ");
  //M5.Lcd.println();
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(MiddleButton, INPUT_PULLUP);
  attachInterrupt(RightButton, ISR_Disable, FALLING);
  attachInterrupt(LeftButton, ISR_Warmer, FALLING);
  attachInterrupt(MiddleButton, ISR_Cooler, FALLING);

  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  delay(1000);
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // still in setup!!! !!!!!!!!!!!!!!!!!!!!!!!!!
  // register first peer
  memcpy(peerInfo.peer_addr, VentAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer
  memcpy(peerInfo.peer_addr, HeaterAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // register third peer RH Lamp C3
  memcpy(peerInfo.peer_addr, LampAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // register forth peer, LH Lamp S3
  memcpy(peerInfo.peer_addr, Lamp2Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // register forth peer, MCR C3
  memcpy(peerInfo.peer_addr, MCRAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function called when data is received
  //esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Heaters.Heater1 = false;
  Heaters.Heater2 = false;
  Lamp.State = false;
  Lamp2.State = false;
  MCR.State = false;
  enable = false;
  warmer = false;
  cooler = false;
  LCDprint_PosDisplay();
  LCDprint_StatusBar();
  TMOS.begin(&Wire, STHS34PF80_I2C_ADDRESS, 21, 22);

  TMOS.setTmosODR(STHS34PF80_TMOS_ODR_AT_30Hz);
  TMOS.setPresenceThreshold(0xA0);  // Default value C8!!!!!! or A0 = 160
  TMOS.setTmosSensitivity(0xff);
  TMOS.setMotionThreshold(0xC8);
  TMOS.setPresenceHysteresis(0x32);
  TMOS.setMotionHysteresis(0x32);
  TMOS.setGainMode(STHS34PF80_GAIN_DEFAULT_MODE);
  TMOS.getGainMode(&gainMode);
  TMOS.getMotionThreshold(&motionThresholdVal);
  TMOS.getPresenceThreshold(&precenceThresholdVal);
  TMOS.getMotionHysteresis(&motionHysteresis);
  Position.value = angleMin;
  delay(5000);
  Position.value = angleMax;
}
//####################################################################   loop
void loop() {  //line 150
  M5.update();
  if (Leave) {
    Leave = false;
    Leaving();
  }
  do_once = millis() < interrupted + 100;
  LCDprint_Gas_resistance();
  Scheduler();
  ESP_Heater();
  delay(100);
  ESP_Vent();
  delay(100);
  VentPosCalc();
  TMOSenable();
  enable = triggered;

  ticktock = !ticktock;
  if (ticktock) {
    Serial.println("tick");
  } else {
    Serial.println("tock");
  }

  delay(500);
}

//###################################################################loop
void Scheduler() {
  if (millis() > lastReadTempAVG + 2000) {
    ReadTempAVG();  //sceduled for 2 seconds
  }
  if (Lampflag) {
    ESP_Lamps();
    ESP_MCR();
  }
  if (Enable_old != enable || Warmer_old != warmer || Cooler_old != cooler) {
    LCDprint_StatusBar();
  }
  Enable_old = enable;
  Warmer_old = warmer;
  Cooler_old = cooler;
  if (enable) {
    M5.Lcd.setBrightness(100);
  } else {
    M5.Lcd.setBrightness(1);
  }
}
void ReadTempAVG() {
  if (x > 4) {
    x = 0;
  }
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  tempC = bme.temperature;
  delay(100);
  humidity = bme.humidity;
  delay(100);
  gas_resistance = bme.gas_resistance / 1000;

  room_tempAVG_old_intX10 = room_tempAVG_intX10;
  //in aid of only Print LCD when change to prevent flicker
  room_temp_read[x] = tempC;
  room_tempAVG = (room_temp_read[0]
                  + room_temp_read[1]
                  + room_temp_read[2]
                  + room_temp_read[3]
                  + room_temp_read[4]
                  + room_tempAVG * 8)
                 / 13;
  room_tempAVG_intX10 = room_tempAVG * 10;

  vent_tempAVG_old_intX10 = vent_tempAVG_intX10;
  vent_temp_read[x] = VentTemp.value;  //from Vent Stamp
  vent_tempAVG = (vent_temp_read[0]
                  + vent_temp_read[1]
                  + vent_temp_read[2]
                  + vent_temp_read[3]
                  + vent_temp_read[4]
                  + vent_tempAVG * 8)
                 / 13;
  vent_tempAVG_intX10 = vent_tempAVG * 10;

  humidityAVG_old_int = humidityAVG;
  humidity_read[x] = humidity;
  humidityAVG = (humidity_read[0]
                 + humidity_read[1]
                 + humidity_read[2]
                 + humidity_read[3]
                 + humidity_read[4])
                / 5;
  humidityAVG_int = humidityAVG;

  //gas resisistance average

  gas_resistanceAVG_old = gas_resistanceAVG;
  gas_resistance_read[x] = gas_resistance;
  gas_resistanceAVG = (gas_resistance_read[0]
                       + gas_resistance_read[1]
                       + gas_resistance_read[2]
                       + gas_resistance_read[3]
                       + gas_resistance_read[4])
                      / 5;

  //Convert to reciprocal
  float gas_resistanceFloat = gas_resistanceAVG;
  VOC = (1 / gas_resistanceFloat) * 10000;

  if (room_tempAVG_old_intX10 != room_tempAVG_intX10) {
    LCDprint_RoomTemp();
  }
  if (vent_tempAVG_old_intX10 != vent_tempAVG_intX10) {
    LCDprint_VentTemp();
  }
  if (humidityAVG_old_int != humidityAVG_int) {
    LCDprint_Humidity();
  }

  lastReadTempAVG = millis();
  x++;
}


void VentPosCalc() {  //vent position is open/close dep on state of 'vent' bool !!!!!!!!!!
  hot = (room_tempAVG > ((setpoint - 1) + offset + vent_hysteresis));
  heat_available = ((vent_tempAVG) > room_tempAVG);
  vent_old = vent;
  if (hot && !heat_available) {
    vent = true;
    Position.value = angleMax;
    vent_hysteresis = -0.2;
  }
  if (!hot && heat_available) {
    vent = true;
    Position.value = angleMax;
    vent_hysteresis = 0.2;
  }
  if (!hot && !heat_available) {
    vent = false;
    Position.value = angleMin;
    vent_hysteresis = 0;
  }
  if (hot && heat_available) {
    vent = false;
    Position.value = angleMin;
    vent_hysteresis = 0;
  }

  if (vent != vent_old) {
    LCDprint_PosDisplay();
  }
}

void ESP_Vent() {
  esp_err_t resultVnt = esp_now_send(VentAddress,
                                     (uint8_t *)&Position, sizeof(Position));
  if (resultVnt == ESP_OK) {
  } else {
    Serial.println("Core in ESP_Vent    Error sending the data");
  }
}

void ESP_Heater() {  //******************************* Heaters ************
  Heater1_old = Heaters.Heater1;
  Heater2_old = Heaters.Heater2;
  Heaters.Heater1 = enable
                    && room_tempAVG < (setpoint + offset + heater1_hysteresis)
                    && !HiVent;
  if (Heaters.Heater1) heater1_hysteresis = 0.2;  //confirmed Nov21@7:30
  if (!Heaters.Heater1) heater1_hysteresis = -0.2;
  HiVent = (vent_tempAVG > 40 + HiVent_hysteresis);
  if (HiVent) {
    HiVent_hysteresis = -0.2;
  } else {
    HiVent_hysteresis = 0.2;
  }
  delay(100);
  Heaters.Heater2 = Heaters.Heater1
                    && (room_tempAVG < (setpoint + offset + heater2_hysteresis - 1));

  if (Heaters.Heater2) heater2_hysteresis = 0.2;
  if (!Heaters.Heater2) heater2_hysteresis = -0.2;
  esp_err_t resultHtr = esp_now_send(HeaterAddress,
                                     (uint8_t *)&Heaters, sizeof(Heaters));
  if (resultHtr == ESP_OK) {
  } else {
    Serial.println(" ESP_Heater, Error sending the data");
  }

  if (Heater1_old != Heaters.Heater1 || Heater2_old != Heaters.Heater2) {
    M5.Lcd.setTextColor(RED);  //^^^^#########################################HEATERS
    M5.Lcd.setFreeFont(FSS9);
    M5.Lcd.setCursor(205, 195);
    M5.Lcd.fillRect(198, 192, 137, 22, BLACK);
    if (Heaters.Heater1 && !Heaters.Heater2) {
      M5.Lcd.print(" 1 HEATER");
    }
    if (Heaters.Heater1 && Heaters.Heater2) {
      M5.Lcd.print("2 HEATERS");
    }
  }

  if (!Heaters.Heater1 && !Heaters.Heater2) {
    M5.Lcd.fillRect(198, 192, 137, 22, BLACK);
  }
}  // end ESP_Heater

void ESP_Lamps() {
  Lamp.State = enable;
  esp_err_t resultLamp = esp_now_send(LampAddress,
                                      (uint8_t *)&Lamp, sizeof(Lamp));
  if (resultLamp == ESP_OK) {
  } else {
    Serial.println("Core in ESP_Lamp    Error sending the data");
  }

  Lamp2.State = enable;
  esp_err_t resultLamp2 = esp_now_send(Lamp2Address,
                                       (uint8_t *)&Lamp2, sizeof(Lamp2));
  if (resultLamp2 == ESP_OK) {
  } else {
    Serial.println("Core in ESP_Lamp2    Error sending the data");
  }
}

void ESP_MCR() {
  MCR.State = enable;
  esp_err_t resultMCR = esp_now_send(MCRAddress,
                                     (uint8_t *)&MCR, sizeof(MCR));
  if (resultMCR == ESP_OK) {
  } else {
    Serial.println("Core in ESP_MCR    Error sending the data");
  }
}

void LCDprint_StatusBar() {
  Serial.println("<<<<<<<<<<<<<<<LCDprintStatusBar>>>>>>>>>>>");
  M5.Lcd.setFreeFont(FSS9);  //(origin=x column, y row)(size=x horiz, y vert)
  if (enable) {              //if enable
    M5.Lcd.fillRect(0, 215, 320, 25, BLACK);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(216, 220);
    M5.Lcd.print("ENABLED");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(30, 220);
    M5.Lcd.print("warmer");
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.setCursor(132, 220);
    M5.Lcd.print("cooler");

  } else {                                    //not enable
    M5.Lcd.fillRect(0, 215, 320, 25, BLACK);  //full bar
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.setCursor(225, 220);
    M5.Lcd.print("OFF");
  }
  M5.Lcd.fillRect(0, 215, 210, 25, BLACK);  //2/3 width (two buttons)
  if (enable && warmer) {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(20, 220);
    M5.Lcd.print("WARMER");
  }
  if (enable && !warmer) {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(30, 220);
    M5.Lcd.print("warmer");
  }
  if (enable && cooler) {
    M5.Lcd.setCursor(120, 220);
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.print("COOLER");
  }
  if (enable && !cooler) {
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.setCursor(132, 220);
    M5.Lcd.print("cooler");
  }
}

void LCDprint_RoomTemp() {  //555%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%!!!!!!!
  Serial.println("<<<<<<<<<<<<<<<LCDprint_RoomTemp>>>>>>>>>>>");
  M5.Lcd.setFreeFont(FSS18);
  M5.Lcd.fillRect(230, 4, 80, 32, BLACK);
  M5.Lcd.setCursor(233, 5);
  if (room_tempAVG > setpoint + offset + 1) {
    M5.Lcd.setTextColor(RED);
  } else if (room_tempAVG < setpoint + offset - 1) {
    M5.Lcd.setTextColor(CYAN);
  } else {
    M5.Lcd.setTextColor(GREEN);
  }
  M5.Lcd.print(room_tempAVG, 1);
}

void LCDprint_VentTemp() {
  Serial.println("<<<<<<<<<<<<<<<LCDprint_VentTemp>>>>>>>>>>>");
  M5.Lcd.setFreeFont(FSS18);
  M5.Lcd.fillRect(230, 44, 80, 32, BLACK);
  M5.Lcd.setCursor(234, 45);
  if (vent_tempAVG > setpoint + offset + 1) {
    M5.Lcd.setTextColor(RED);
  } else if (vent_tempAVG < room_tempAVG) {
    M5.Lcd.setTextColor(CYAN);
  }
  M5.Lcd.print(vent_tempAVG, 1);
}

void LCDprint_PosDisplay() {
  //line 274 needs work
  Serial.println("<<<<<<<<<<<<<<<LCDprint_PosDisplay>>>>>>>>>>>");
  M5.Lcd.setFreeFont(FSS12);
  M5.Lcd.fillRect(226, 85, 75, 30, BLACK);
  M5.Lcd.setCursor(233, 90);
  Serial.print("vent = ");
  Serial.println(vent);
  if (vent) {
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.print("OPEN");
  } else {
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.print("SHUT");
  }
}

void LCDprint_Humidity() {
  Serial.println("<<<<<<<<<<<<<<<LCDprint_Humidity>>>>>>>>>>>");
  M5.Lcd.setFreeFont(FSS12);
  M5.Lcd.fillRect(230, 122, 70, 30, BLACK);
  M5.Lcd.setCursor(245, 130);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.print(humidityAVG, 0);
  M5.Lcd.setFreeFont(FSS12);
  M5.Lcd.setCursor(274, 130);
  M5.Lcd.print("%");
}

void LCDprint_Gas_resistance() {
  //Serial.println("<<<<<<<<<<<<<<<LCDprint_Gas_resistance>>>>>>>>>>>");
  M5.Lcd.setCursor(245, 167);
  M5.Lcd.setFreeFont(FSS12);
  if (VOC < 40) {
    M5.Lcd.fillRect(240, 165, 70, 25, BLACK);
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.print(VOC, 0);
  }
  if (VOC >= 40) {
    if (ticktock) {
      M5.Lcd.fillRect(240, 165, 70, 25, BLUE);
      M5.Lcd.setTextColor(RED);
    } else {
      M5.Lcd.fillRect(240, 165, 70, 25, RED);
      M5.Lcd.setTextColor(CYAN);
    }
    M5.Lcd.print(VOC, 0);
  }
}

void Leaving() {
  M5.Lcd.fillRect(0, 213, 320, 25, BLACK);
  M5.Lcd.setFreeFont(FSS12);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setCursor(210, 215);
  M5.Lcd.print("leaving");

  TMOS.resetAlgo();  //TIMEOUT RESET ALGO
  delay(10000);
  Heaters.Heater1 = false;
  Heaters.Heater2 = false;
  Lamp.State = false;
  Lamp2.State = false;
  MCR.State = false;
  triggered = false;
  warmer = false;
  cooler = false;
  offset = 0;
  enable = false;
  Leave = false;
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&VentTemp, incomingData, sizeof(VentTemp));
}

void ISR_Disable() {
  if (do_once) {  //was interrupted and is now low
    interrupted = millis();
    do_once = false;
  }
  Leave = true;
}


void ISR_Warmer() {
  if (!do_once) {  //was interrupted and is now low
    interrupted = millis();
    do_once = true;
    cooler = false;
    if (enable) {  //do things from here
      (warmer = !warmer);
      if (warmer) {
        offset = 1.5;
      } else {
        offset = 0;
      }
    }
  }
}

void ISR_Cooler() {
  if (!do_once) {  //was interrupted and is now low
    interrupted = millis();
    do_once = true;
    warmer = false;
    if (enable) {  //do things from here
      (cooler = !cooler);
      if (cooler) {
        offset = -1;
      } else {
        offset = 0;
      }
    }
  }
}

void TMOSenable() {
  sths34pf80_tmos_drdy_status_t dataReady;
  TMOS.getDataReady(&dataReady);
  delay(200);
  if (dataReady.drdy == 1) {  //begin dataReady.drdy
    sths34pf80_tmos_func_status_t status;
    TMOS.getStatus(&status);
    TMOS.getPresenceValue(&presenceVal);
    Serial.println("    ");
    Serial.printf("PresenceValue:%d\n  ", presenceVal);
    Serial.printf("  status.pres_flag:%d\n  ", status.pres_flag);
    Serial.print("        TimeoutResets= ");
    Serial.println(TimeoutResets);
    Serial.println("    ");

    // Pres on too long timer start-v
    if (status.pres_flag && !pres_flag_1shot) {
      longtime = millis();
      pres_flag_1shot = true;  //one shot flag
    } else if (!status.pres_flag) {
      pres_flag_1shot = false;
    }

    // Timeout reset the algo-v
    too_long = millis() > longtime + 25 * 60 * 1000;  //25 minutes
    if (pres_flag_1shot && too_long) {
      TMOS.resetAlgo();  //RESET ALGO
      pres_flag_1shot = false;
      Serial.println("   ");
      Serial.println("             TimeOut reset the Algo ");
      Serial.println("   ");
      triggered = false;
      delay(2000);
      TimeoutResets++;
      Serial.println("   ");
      longtime = millis();
    }

    // Manual reset the algo-v
    if (Serial.available() > 0) {  //Read Serial.monitor
      // read the incoming byte: reset the algo manually
      int incomingByte = Serial.read();
      Serial.print("I received: ");
      delay(1000);
      Serial.println(incomingByte);
      if (incomingByte == 48) {  //'0' i.e. rest to 0
        TMOS.resetAlgo();
        TimeoutResets = 0;
        Serial.println("           Serial reset the Algo ");
      }
      delay(2000);
    }
    int off_delay;
    if (warmer) {
      //>>>>>>>>>>min>>sec>>ms
      off_delay = 60 * 60 * 1000;
    } else {
      off_delay = 20 * 60 * 1000;
    }

    //after TMOS tests sets enable
    if (status.pres_flag) {
      triggered = true;
      enable = true;
      trigger_time = millis();
    } else if (millis() > trigger_time + off_delay) {
      triggered = false;  // minutes no trigger
      enable = false;
      warmer = false;
      cooler = false;
    }
  }  //within dataReady.drdy
  delay(500);
}
