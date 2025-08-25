#include <U8g2lib.h>
#include <MUIU8g2.h>
// #include <SoftwareSerial.h>
#include <RCSwitch.h>
#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <mDNS.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <PCF8574.h> //https://github.com/RobTillaart/PCF8574
#include <PCF8575.h> //https://github.com/RobTillaart/PCF8574
#include <DFRobotDFPlayerMini.h>
#include <LD2450.h>
#include <EnvironmentCalculations.h>
#include <BME280I2C.h>
#include <Wire.h>

void saveDFPlayerConfig();

float referencePressure = 1018.6; // hPa local QFF (official meteor-station reading)
float outdoorTemp = 4.7;          // °C  measured local outdoor temp.
float barometerAltitude = 1650.3; // meters ... map readings + barometer position
BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_16,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76);

BME280I2C bme(settings);

WiFiClient espClient;
PubSubClient mqttClient(espClient);
RTC_DS3231 rtc;
DateTime now;
StaticJsonDocument<256> configJson;
StaticJsonDocument<256> configDFPlayer;
WebServer server(80);
HTTPUpdateServer httpUpdater;
PCF8574 PCFKeyboard(0x27);
PCF8574 PCFRelay(0x26);
LD2450 ld2450;
#define FPSerial Serial2
/*
  I2C device found at 0x26 PCF 1
  I2C device found at 0x27 PCF 2
  I2C device found at 0x57 EEPROM DS3231
  I2C device found at 0x68 DS3231
  I2C device found at 0x76 BME2800
  pin 32 gas sensor
  pin 34 rain sansor

*/

DFRobotDFPlayerMini myDFPlayer;
// Створюємо об'єкт дисплея для SPI
// U8G2_ST7920_128X64_F_1_HAL_HW_SPI(cs, reset) -- Fast full framebuffer
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* reset=*/U8X8_PIN_NONE);
MUIU8G2 mui;
RCSwitch mySwitch = RCSwitch();

const char *www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";

unsigned long last_msg;
int timeout_mqtt_reconnect = 3000;
unsigned long timer_save_last_track;
unsigned long timer_mqtt_reconnect;
unsigned long read_joystik_timer;
float temp_inside = 28.7;
float temp_outside = -12.3;
float pressure = 758.8;
float humidity = 46;
const char *wifi_modes[] = {"Station", "Access point"};
uint8_t is_redraw = 1;

unsigned long send_mqqt_timer;
#define joystikLeftRightPin 35
#define joystikUpDownPin 33

/*Date & time*/
uint8_t time_hours;
uint8_t time_minutes;
uint8_t time_seconds;
uint8_t day;
uint8_t month;
uint8_t year;

/*DFPlayer*/
const char *equalizer[] = {"Normal", "Pop", "Rock", "Jazz", "Classic", "Bass"};
int last_play_track;
uint16_t eq_index_set;
uint8_t volume_set;
bool random_all_set;
bool loop_set;
bool loop_all_set;
bool reset_dfp;
bool play_pause = false;
int file_count;

/*display mui*/
uint8_t num_value = 0;
uint8_t bar_value = 0;
uint16_t animal_idx = 0;

unsigned long key_timer;
unsigned long menu_show_timer;

void printDate();
uint16_t menu_get_cnt(void *data)
{
  return 5; /* number of menu entries */
}

const char *
menu_get_str(void *data, uint16_t index)
{
  static const char *dfp_menu[] = {
      MUI_21 "Volume",
      MUI_22 "EQ",
      MUI_23 "Loop",
      MUI_24 "Random",
      MUI_25 "Reset DFP",

  };
  return dfp_menu[index];
}
uint16_t selection = 0;
uint8_t mui_hrule(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    u8g2.drawHLine(0, mui_get_y(ui), u8g2.getDisplayWidth());
  }
  return 0;
}

uint16_t equalizer_get_cnt(void *data)
{
  return sizeof(equalizer) / sizeof(*equalizer); /* number of animals */
}

const char *equalizer_get_str(void *data, uint16_t index)
{
  return equalizer[index];
}
uint8_t openCloseDoors(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    uint8_t y = mui_get_y(ui);

    if (mui_IsCursorFocus(ui))
    {
      u8g2.setDrawColor(1);             // белый фон
      u8g2.drawBox(0, y - 10, 128, 12); // выделение
      u8g2.setDrawColor(0);             // чёрный текст
    }
    else
    {
      u8g2.setDrawColor(1); // обычный белый текст
    }

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(5, y, "Open/Close");
    u8g2.setDrawColor(1); // восстановление цвета
    return 1;
  }

  else if (msg == MUIF_MSG_CURSOR_SELECT)
  {
    // кнопка нажата!
    mqttClient.publish("garage/doors_controller/move_doors/move", "1");
    return 1;
  }
  return 0;
}

uint8_t dfp_play_pause(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    uint8_t y = mui_get_y(ui);

    if (mui_IsCursorFocus(ui))
    {
      u8g2.setDrawColor(1);             // белый фон
      u8g2.drawBox(40, y - 10, 60, 12); // выделение
      u8g2.setDrawColor(0);             // чёрный текст
    }
    else
    {
      u8g2.setDrawColor(1); // обычный белый текст
    }

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(45, y, "Pause/play");
    u8g2.setDrawColor(1); // восстановление цвета
    return 1;
  }

  else if (msg == MUIF_MSG_CURSOR_SELECT)
  {
    int state = myDFPlayer.readState();
    if (state == 1)
    {
      // Serial.print("Track number: ");
      // myDFPlayer.readCurrentFileNumber();
      // last_play_track = myDFPlayer.readFileCounts();
      saveDFPlayerConfig();
      // delay(300);
      myDFPlayer.pause();
    }
    else if (state == 2)
    {
      myDFPlayer.start();
    }
    else if (state == 0)
    {
      myDFPlayer.playMp3Folder(last_play_track);
    }
    return 1;
  }
  return 0;
}

uint8_t dfp_stop(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    uint8_t y = mui_get_y(ui);

    if (mui_IsCursorFocus(ui))
    {
      u8g2.setDrawColor(1);             // белый фон
      u8g2.drawBox(40, y - 10, 60, 12); // выделение
      u8g2.setDrawColor(0);             // чёрный текст
    }
    else
    {
      u8g2.setDrawColor(1); // обычный белый текст
    }

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(45, y, "Stop");
    u8g2.setDrawColor(1); // восстановление цвета
    return 1;
  }

  else if (msg == MUIF_MSG_CURSOR_SELECT)
  {
    int state = myDFPlayer.readState();
    if (state == 1)
    {
      saveDFPlayerConfig();
      myDFPlayer.stop();
    }
    return 1;
  }
  return 0;
}
uint8_t dfp_next_track(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    uint8_t y = mui_get_y(ui);

    if (mui_IsCursorFocus(ui))
    {
      u8g2.setDrawColor(1);            // белый фон
      u8g2.drawBox(0, y - 10, 60, 12); // выделение
      u8g2.setDrawColor(0);            // чёрный текст
    }
    else
    {
      u8g2.setDrawColor(1); // обычный белый текст
    }

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(5, y, "Next");
    u8g2.setDrawColor(1); // восстановление цвета
    return 1;
  }

  else if (msg == MUIF_MSG_CURSOR_SELECT)
  {
    myDFPlayer.next();
    // delay(300);
    // last_play_track = myDFPlayer.readCurrentFileNumber();
    return 1;
  }
  return 0;
}
uint8_t dfp_previous_track(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    uint8_t y = mui_get_y(ui);

    if (mui_IsCursorFocus(ui))
    {
      u8g2.setDrawColor(1);             // белый фон
      u8g2.drawBox(70, y - 10, 60, 12); // выделение
      u8g2.setDrawColor(0);             // чёрный текст
    }
    else
    {
      u8g2.setDrawColor(1); // обычный белый текст
    }

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(75, y, "Previous");
    u8g2.setDrawColor(1); // восстановление цвета
    return 1;
  }

  else if (msg == MUIF_MSG_CURSOR_SELECT)
  {
    myDFPlayer.previous();
    // delay(300);
    // last_play_track = myDFPlayer.readCurrentFileNumber();
    return 1;
  }
  return 0;
}

uint8_t dfp_current_file(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    uint8_t y = mui_get_y(ui);
    u8g2.setDrawColor(0);            // чёрный текст
    u8g2.drawBox(0, y - 10, 60, 24); // выделение
    u8g2.setDrawColor(1);            // белый фон

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(5, y);
    u8g2.print(last_play_track);
    u8g2.setCursor(20, y);
    u8g2.print("/");
    u8g2.setCursor(30, y);
    u8g2.print(file_count);
    // u8g2.drawStr(5, y, String(last_play_track.c_str()));
    u8g2.setDrawColor(1); // восстановление цвета
    return 1;
  }
  return 0;
}

uint8_t print_date(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    printDate();
    return 1;
  }
  return 0;
}

muif_t muif_list[] = {
    MUIF_U8G2_FONT_STYLE(0, u8g2_font_helvR08_tr), /* regular font */
    MUIF_U8G2_FONT_STYLE(1, u8g2_font_helvB08_tr), /* bold font */
    MUIF_U8G2_FONT_STYLE(2, u8g2_font_8x13_tr),    /* bold font */

    MUIF_RO("HR", mui_hrule),
    MUIF_U8G2_LABEL(),
    MUIF_RO("GP", mui_u8g2_goto_data),
    MUIF_BUTTON("GC", mui_u8g2_goto_form_w1_pi),

    /*Date & time*/
    MUIF_U8G2_U8_MIN_MAX("HC", &time_hours, 0, 23, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("MC", &time_minutes, 0, 59, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("SC", &time_seconds, 0, 59, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("YD", &year, 0, 99, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("MD", &month, 1, 12, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("DD", &day, 1, 31, mui_u8g2_u8_min_max_wm_mud_pi),

    /*DFP settings*/
    MUIF_VARIABLE("LP", &loop_set, mui_u8g2_u8_chkbox_wm_pi),
    MUIF_VARIABLE("LA", &loop_all_set, mui_u8g2_u8_chkbox_wm_pi),
    MUIF_VARIABLE("RA", &random_all_set, mui_u8g2_u8_chkbox_wm_pi),
    MUIF_U8G2_U8_MIN_MAX_STEP("VS", &volume_set, 0, 30, 2, MUI_MMS_2X_BAR | MUI_MMS_SHOW_VALUE, mui_u8g2_u8_bar_wm_mud_pi),
    MUIF_VARIABLE("RT", &reset_dfp, mui_u8g2_u8_chkbox_wm_pi),
    MUIF_U8G2_U16_LIST("EQ", &eq_index_set, NULL, equalizer_get_str, equalizer_get_cnt, mui_u8g2_u16_list_line_wa_mud_pi),
    MUIF_RO("CT", dfp_current_file),

    /*DFPlayer*/
    MUIF_BUTTON("DP", dfp_play_pause),
    MUIF_BUTTON("DS", dfp_stop),
    MUIF_BUTTON("NT", dfp_next_track),
    MUIF_BUTTON("PT", dfp_previous_track),
    MUIF_RO("PD", print_date),

    /*Garage doors*/
    MUIF_BUTTON("OC", openCloseDoors),

    // MUIF_U8G2_U8_MIN_MAX("NV", &num_value, 0, 99, mui_u8g2_u8_min_max_wm_mse_pi),
    // MUIF_U8G2_U8_MIN_MAX_STEP("NB", &bar_value, 0, 16, 1, MUI_MMS_2X_BAR, mui_u8g2_u8_bar_wm_mse_pf),

    MUIF_U8G2_U16_LIST("ID", &selection, NULL, menu_get_str, menu_get_cnt, mui_u8g2_u16_list_goto_w1_pi),
    /* register custom function to show the data */

    /* a button for the menu... */
    MUIF_BUTTON("GO", mui_u8g2_btn_goto_wm_fi),
    MUIF_BUTTON("G1", mui_u8g2_btn_back_wm_fi)};

fds_t fds_data[] =

    MUI_FORM(1)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Settings")
                MUI_STYLE(0)
                    MUI_XY("HR", 0, 11)
                        MUI_DATA("GP",
                                 MUI_10 "Clock|" MUI_20 "Player|" MUI_30 "Light|" MUI_40 "Open/Close doors")
                            MUI_XYA("GC", 5, 24, 0)
                                MUI_XYA("GC", 5, 36, 1)
                                    MUI_XYA("GC", 5, 48, 2)
                                        MUI_XYA("GC", 5, 60, 3)
    // Clock settings
    MUI_FORM(10)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Clock settings")
                MUI_STYLE(0)
                    MUI_XY("HR", 0, 11)
                        MUI_DATA("GP",
                                 MUI_11 "Set clock|" MUI_12 "Set date")
                            MUI_XYA("GC", 5, 24, 0)
                                MUI_XYA("GC", 5, 36, 1)
    // Set clock
    MUI_FORM(11)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Set clock")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(2)
                        MUI_XY("HC", 30, 35)
                            MUI_LABEL(40, 35, " : ")
                                MUI_XY("MC", 55, 35)
                                    MUI_LABEL(65, 35, " : ")
                                        MUI_XY("SC", 80, 35)
    //     MUI_LABEL(5, 35, "Minutes:")
    //         MUI_LABEL(5, 47, "Seconds:")
    MUI_STYLE(0)
        MUI_XYAT("GO", 114, 60, 10, " Ok ")
    // Set date
    MUI_FORM(12)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Set date")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_LABEL(5, 23, "Day:")
                            MUI_LABEL(5, 35, "Month:")
                                MUI_LABEL(5, 47, "Year:")
                                    MUI_XY("DD", 50, 23)
                                        MUI_XY("MD", 50, 35)
                                            MUI_XY("YD", 50, 47)
                                                MUI_XYAT("GO", 114, 60, 10, " Ok ")
    // Player settings
    MUI_FORM(20)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Player settings")
                MUI_STYLE(0)
                    MUI_XY("HR", 0, 11)
                        MUI_XYA("ID", 5, 24, 0)
                            MUI_XYA("ID", 5, 36, 1)
                                MUI_XYA("ID", 5, 48, 2)
                                    MUI_XYA("ID", 5, 60, 3)
    //
    MUI_FORM(21)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Volume")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_XY("VS", 1, 40)
                            MUI_XYAT("GO", 114, 60, 20, " Ok ")
    //
    MUI_FORM(22)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "EQ select")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_LABEL(5, 24, "EQ:")
                            MUI_XYA("EQ", 30, 24, 50)
                                MUI_XYAT("GO", 114, 60, 20, " Ok ")
    // Set loop
    MUI_FORM(23)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Set loop")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_XYAT("LP", 1, 24, 0, "Loop")
                            MUI_XYAT("LA", 1, 36, 1, "Loop All")
                                MUI_XYAT("GO", 114, 60, 20, " Ok ")
    //
    MUI_FORM(24)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Random")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_XYAT("RA", 1, 24, 0, "Random all")
                            MUI_XYAT("GO", 114, 60, 20, " Ok ")
    //
    MUI_FORM(25)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Reset")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_XYAT("RT", 1, 24, 1, "Reset")
                            MUI_XYAT("GO", 114, 60, 20, " Ok ")
    //
    MUI_FORM(40)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Garage doors")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_XYAT("OC", 10, 24, 1, "Open/Close")
                            MUI_XYT("G1", 20, 60, "Back")
                                MUI_XYAT("GO", 114, 60, 1, " Ok ")
    //
    MUI_FORM(50)
        MUI_STYLE(1)
            MUI_STYLE(0)
                MUI_XY("PD", 10, 12)
                    MUI_XYAT("DP", 10, 24, 1, "Play/pause")
                        MUI_XYAT("NT", 10, 36, 1, "Next")
                            MUI_XYAT("PT", 10, 36, 1, "Previous")
                                MUI_XYAT("DS", 10, 48, 1, "Stop")
                                    MUI_XY("VS", 45, 60)
                                        MUI_XY("CT", 10, 60);
/*-------------------------*/

unsigned long timerFreez;
int mainScreen[] = {1, 0, 0};
void generalScreen();
void playerScreen();
void lightsInfo();
void (*generalScreenFuncs[])() = {generalScreen, playerScreen, lightsInfo};
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String topicPath = "garage/main_controller/";
const char *inTopic[] = {
    "garage/main_controller/dfplayer/",
};

String convertFloatToString(float value, int lenght)
{
  String myString = String(value);
  return myString.substring(0, lenght);
}
//****************CONFIG*********************
void saveConfig()
{

  File file = LittleFS.open("/config.json", "w");
  if (!file)
  {
    // Serial.println("File creation error!");
    return;
  }
  serializeJson(configJson, file);
  file.close();
}

void loadConfig()
{
  File file = LittleFS.open("/config.json", "r");
  if (!file)
  {
    // Serial.println("File \"config.json\" not found!");
  }
  DeserializationError error = deserializeJson(configJson, file);
  file.close();
  if (error)
  {
    // Serial.println("Error load \"config.json\"!");
    return;
  }
}

void saveDFPlayerConfig()
{

  File file = LittleFS.open("/dfplayer_config.json", "w");
  if (!file)
  {
    // Serial.println("File creation error!");
    return;
  }
  configDFPlayer["last_play_track"] = last_play_track;
  configDFPlayer["equaliser"] = eq_index_set;
  configDFPlayer["volume"] = volume_set;
  configDFPlayer["random_all"] = random_all_set;
  configDFPlayer["loop_all"] = loop_all_set;
  configDFPlayer["loop"] = loop_set;
  serializeJson(configDFPlayer, file);
  file.close();
}

void loadDFPlayerConfig()
{
  File file = LittleFS.open("/dfplayer_config.json", "r");
  if (!file)
  {
    // Serial.println("File \"dfplayer_config.json\" not found!");
  }
  DeserializationError error = deserializeJson(configDFPlayer, file);
  file.close();
  if (error)
  {
    // Serial.println("Error load \"dfplayer_config.json\"!");
    return;
  }
  else
  {
    last_play_track = configDFPlayer["last_play_track"].as<int>();
    eq_index_set = configDFPlayer["equaliser"].as<int>();
    volume_set = configDFPlayer["volume"].as<int>();
    random_all_set = configDFPlayer["random_all"].as<int>();
    loop_set = configDFPlayer["loop"].as<int>();
    loop_set = configDFPlayer["loop_all"].as<int>();
    (loop_set) ? myDFPlayer.enableLoop() : myDFPlayer.disableLoop();
    delay(100);
    myDFPlayer.volume(volume_set);
    delay(100);
    myDFPlayer.EQ(eq_index_set);
    delay(100);
  }
}

void requireChangeSetDFP()
{
  if (eq_index_set != configDFPlayer["equaliser"].as<int>())
  {
    myDFPlayer.EQ(eq_index_set);
    saveDFPlayerConfig();
  }
  else if (volume_set != configDFPlayer["volume"].as<int>())
  {
    myDFPlayer.volume(volume_set);
    saveDFPlayerConfig();
  }
  else if (random_all_set != configDFPlayer["random_all"].as<int>())
  {
    if (random_all_set)
    {
      myDFPlayer.randomAll();
    }
    saveDFPlayerConfig();
  }
  else if (loop_set != configDFPlayer["loop"].as<int>())
  {
    (loop_set) ? myDFPlayer.enableLoop() : myDFPlayer.disableLoop();
    saveDFPlayerConfig();
  }
  else if (loop_all_set != configDFPlayer["loop_all"].as<int>())
  {
    (loop_all_set) ? myDFPlayer.enableLoopAll() : myDFPlayer.disableLoopAll();
    saveDFPlayerConfig();
  }

  if (reset_dfp)
    myDFPlayer.reset();
}

void callbackMqtt(char *topic, byte *payload, unsigned int length)
{
  const char *topics = topic;
  size_t topicCount = sizeof(inTopic) / sizeof(inTopic[0]);
  int elem = -1;
  for (size_t i = 0; i < topicCount; i++)
  {
    if (!strcmp(topics, inTopic[i]))
    {
      elem = i;
    }
  }
  char buffer[length + 1];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';
  String str = String(buffer);
  switch (elem)
  {
  case 0:
    saveConfig();
    break;
  default:
  {
    break;
  }
    loadConfig();
  }
}

//****************HTTP*********************
void handleFileRequest(const String &path, const String &contentType)
{
  File file = LittleFS.open(path, "r");
  if (!file)
  {
    server.send(404, "text/plain", "File Not Found");
    return;
  }
  server.streamFile(file, contentType);
  file.close();
}

void handleRoot()
{
  if (!server.authenticate(configJson["login"].as<const char *>(), configJson["pass"].as<const char *>()))
  // Basic Auth Method with Custom realm and Failure Response
  // return server.requestAuthentication(BASIC_AUTH, www_realm, authFailResponse);
  // Digest Auth Method with realm="Login Required" and empty Failure Response
  // return server.requestAuthentication(DIGEST_AUTH);
  // Digest Auth Method with Custom realm and empty Failure Response
  // return server.requestAuthentication(DIGEST_AUTH, www_realm);
  // Digest Auth Method with Custom realm and Failure Response
  {
    return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
  }
  handleFileRequest("/index.html", "text/html");
}

void handleCSS()
{
  if (!server.authenticate(configJson["login"].as<const char *>(), configJson["pass"].as<const char *>()))
  {
    return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
  }
  handleFileRequest("/style.css", "text/css");
}

void handleJS()
{
  if (!server.authenticate(configJson["login"].as<const char *>(), configJson["pass"].as<const char *>()))
  {
    return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
  }
  handleFileRequest("/script.js", "text/javascript");
}

void notFound()
{
  server.send(404, "text/plain", "404: Not Found");
}

void sendData()
{
  if (!server.authenticate(configJson["login"].as<const char *>(), configJson["pass"].as<const char *>()))
  {
    return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
  }
  JsonDocument doc;
  JsonObject jsonDoc = doc.to<JsonObject>();
  jsonDoc["ssid"] = configJson["ssid"].as<const char *>();
  jsonDoc["network_ip"] = configJson["network_ip"].as<const char *>();
  // jsonDoc["wifi_pass"]=config.wifi_pass;
  jsonDoc["mqtt_server"] = configJson["mqtt_server"].as<const char *>();
  jsonDoc["mqtt_port"] = configJson["mqtt_port"].as<int>();
  jsonDoc["mqtt_user"] = configJson["mqtt_user"].as<const char *>();
  // jsonDoc["mqtt_pass"]=config.mqtt_pass;
  jsonDoc["login"] = configJson["login"].as<const char *>();
  // jsonDoc["pass"]=config.pass;
  jsonDoc["clientId"] = configJson["clientId"].as<const char *>();
  jsonDoc["wifi_mode"] = configJson["wifi_mode"].as<int>();

  JsonArray wifi_mode = doc.createNestedArray("wifi_mode");
  for (unsigned int i = 0; i < sizeof(wifi_modes) / sizeof(wifi_modes[0]); i++)
  {
    wifi_mode.add(wifi_modes[i]);
  }
  JsonArray wifi_networks = doc.createNestedArray("wifi_networks");
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++)
  {
    wifi_networks.add(WiFi.SSID(i));
  }
  String jsonResponse;
  serializeJson(jsonDoc, jsonResponse);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(200, "application/json", jsonResponse);
}

void saveWifiSet()
{
  if (server.method() != HTTP_POST)
  {
    server.onNotFound(notFound);
  }
  else
  {
    if (server.arg("ssid").length() > 0)
    {
      configJson["ssid"] = server.arg("ssid");
    }
    if (server.arg("wifi_pass").length() > 0)
    {
      configJson["wifi_pass"] = server.arg("wifi_pass");
    }
    if (server.arg("wifi_mode").length() > 0)
    {
      configJson["wifi_mode"] = (server.arg("wifi_mode") == wifi_modes[0]) ? 1 : 2;
    }
    saveConfig();
    server.send(200, "text/plain", "Wifi config save.");
    ESP.restart();
  }
}

void saveMqttSet()
{
  if (server.method() != HTTP_POST)
  {
    server.onNotFound(notFound);
  }
  else
  {
    if (server.arg("mqtt_server").length() > 0)
    {
      configJson["mqtt_server"] = server.arg("mqtt_server");
    }
    if (server.arg("mqtt_port").length() > 0)
    {
      configJson["mqtt_port"] = server.arg("mqtt_port");
    }
    if (server.arg("mqtt_user").length() > 0)
    {
      configJson["mqtt_user"] = server.arg("mqtt_user");
    }
    if (server.arg("mqtt_pass").length() > 0)
    {
      configJson["mqtt_pass"] = server.arg("mqtt_pass");
    }
    if (server.arg("clientId").length() > 0)
    {
      configJson["clientId"] = server.arg("clientId");
    }
    saveConfig();
    server.send(200, "text/plain", "MQTT config save.");
    ESP.restart();
  }
}

void saveUserSet()
{
  if (server.method() != HTTP_POST)
  {
    server.onNotFound(notFound);
  }
  else
  {
    if (server.arg("login").length() > 0)
    {
      configJson["login"] = server.arg("login");
    }
    if (server.arg("pass").length() > 0)
    {
      configJson["pass"] = server.arg("pass");
    }
    saveConfig();
    server.send(200, "text/plain", "User config save.");
    ESP.restart();
  }
}

//****************DISPLAY*********************
void printTemperatureInside()
{
  u8g2.setColorIndex(1);
  u8g2.drawHLine(2, 45, 7);
  u8g2.drawHLine(2, 51, 7);
  u8g2.drawVLine(2, 45, 6);
  u8g2.drawVLine(8, 45, 6);
  u8g2.drawHLine(4, 48, 9);
  u8g2.drawPixel(5, 47);
  u8g2.drawPixel(5, 49);
  u8g2.setCursor(18, 52);
  u8g2.print(convertFloatToString(temp_inside, 4));
  u8g2.drawStr(43, 46, ".");
  u8g2.drawStr(47, 52, "C");
}

void printTemperatureOutside()
{
  u8g2.setColorIndex(1);
  u8g2.drawHLine(2, 56, 7);
  u8g2.drawHLine(2, 62, 7);
  u8g2.drawVLine(2, 56, 6);
  u8g2.drawVLine(8, 56, 6);
  u8g2.drawHLine(4, 59, 9);
  u8g2.drawPixel(11, 58);
  u8g2.drawPixel(11, 60);
  u8g2.setCursor(18, 63);
  u8g2.print(convertFloatToString(temp_outside, 4));
  u8g2.drawStr(43, 57, ".");
  u8g2.drawStr(47, 63, "C");
}

void printPressure()
{
  u8g2.setColorIndex(1);
  u8g2.setCursor(90, 52);
  u8g2.print(convertFloatToString(humidity, 4) + " %");
}

void printHumidity()
{
  u8g2.setColorIndex(1);
  u8g2.setCursor(90, 63);
  u8g2.print(convertFloatToString(pressure, 3));
  u8g2.setCursor(114, 63);
  u8g2.print("mm");
}

void printDate()
{
  u8g2.setColorIndex(1);
  u8g2.drawBox(0, 0, 128, 12);
  u8g2.setFont(u8g_font_6x10);
  u8g2.setColorIndex(0);
  u8g2.setCursor(65, 9);
  u8g2.print(String(now.day()) + "." + String(now.month()) + "." + String(now.year()));
  u8g2.setCursor(5, 9);
  u8g2.print(daysOfTheWeek[now.dayOfTheWeek()]);
}
void generalScreen()
{
  u8g2.firstPage();
  do
  {
    printDate();
    printTemperatureOutside();
    printTemperatureInside();
    printPressure();
    printHumidity();

    u8g2.setFont(u8g_font_courB24n); // Выбираем шрифт u8g_font_courB24n
    u8g2.setCursor(3, 37);
    u8g2.print(now.hour() / 10); // Выводим старший разряд часов    в позиции   3х43
    u8g2.setCursor(20, 37);
    u8g2.print(now.hour() % 10); // Выводим младший разряд часов    в позиции  20х43 20х43
    u8g2.drawStr(33, 37, ":");   // Выводим двоеточие               в позиции  33х43
    u8g2.setCursor(46, 37);
    u8g2.print(now.minute() / 10); // Выводим старший разряд минут    в позиции  46х43
    u8g2.setCursor(63, 37);
    u8g2.print(now.minute() % 10); // Выводим младший разряд минут    в позиции  63х4363х43
    u8g2.drawStr(76, 37, ":");     // Выводим двоеточие               в позиции  76х43
    u8g2.setCursor(89, 37);
    u8g2.print(now.second() / 10); // Выводим старший разряд секунд   в позиции  89х43
    u8g2.setCursor(106, 37);
    u8g2.print(now.second() % 10); // Выводим младший разряд секунд   в позиции 106х43
  } while (u8g2.nextPage());
}
void playerScreen()
{
  /* update the display content, if the redraw flag is set */
  if (is_redraw)
  {
    u8g2.firstPage();
    do
    {
      printDate();
    } while (u8g2.nextPage());
    is_redraw = 0; /* clear the redraw flag */
  }
}
void lightsInfo()
{
  u8g2.firstPage();
  do
  {
    printDate();
  } while (u8g2.nextPage());
  is_redraw = 0; /* clear the redraw flag */
}
void reconnectMqtt()
{
  if ((int)(millis() - timer_mqtt_reconnect) > timeout_mqtt_reconnect)
  {
    if (!mqttClient.connected())
    {
      if (mqttClient.connect(configJson["clientId"].as<const char *>(), configJson["mqtt_user"].as<const char *>(), configJson["mqtt_pass"].as<const char *>()))
      {
        size_t topicCount = sizeof(inTopic) / sizeof(inTopic[0]);
        for (size_t i = 0; i < topicCount; i++)
        {
          mqttClient.subscribe(inTopic[i]);
        }
      }
    }
    timer_mqtt_reconnect = millis();
  }
}

void connectWiFi(int mode)
{
  if (mode == 1)
  {
    WiFi.mode(WIFI_STA);
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(configJson["ssid"].as<const char *>(), configJson["wifi_pass"].as<const char *>());
      // Serial.print("Connecting to WiFi");
      int tries = 0;
      while (WiFi.status() != WL_CONNECTED && tries < 20)
      { // ждём до 20 секунд
        delay(1000);
        // Serial.print(".");
        tries++;
      }
      // Serial.println();
      if (WiFi.status() == WL_CONNECTED)
      {
        // Serial.println("WiFi connected!");
      }
      else
      {
        // Serial.println("WiFi NOT connected!");
        // Serial.println("WiFi NOT connected! Starting access point.");
        if (WiFi.softAP(configJson["ssid"].as<const char *>(), configJson["wifi_pass"].as<const char *>()))
        {
          // Serial.println("Access point started!");
          // Serial.print("IP adress: ");
          // Serial.println(WiFi.softAPIP());
        }
      }
    }
  }
  else if (mode == 2)
  {
    if (WiFi.softAP(configJson["ssid"].as<const char *>(), configJson["wifi_pass"].as<const char *>()))
    {
      // Serial.println("Access point started!");
      // Serial.print("IP adress: ");
      // Serial.println(WiFi.softAPIP());
    }
    else
    {
      // Serial.println("Faild to start access point!");
    }
  }
}

void printDetail(uint8_t type, int value)
{
  switch (type)
  {
  case TimeOut:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "Time Out!");
    break;
  case WrongStack:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "Stack Wrong!");
    break;
  case DFPlayerCardInserted:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "Card Inserted!");
    break;
  case DFPlayerCardRemoved:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "Card Removed!");
    break;
  case DFPlayerCardOnline:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "Card Online!");
    break;
  case DFPlayerUSBInserted:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "USB Inserted!");
    break;
  case DFPlayerUSBRemoved:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "USB Removed!");
    break;
  case DFPlayerPlayFinished:
    mqttClient.publish("garage/main_controller/dfplayer/detail", "Number:");
    // Serial.print(value);
    mqttClient.publish("garage/main_controller/dfplayer/detail", "Play Finished!");
    break;
  case DFPlayerError:
    switch (value)
    {
    case Busy:
      mqttClient.publish("garage/main_controller/dfplayer/errors", "Card not found");
      break;
    case Sleeping:
      mqttClient.publish("garage/main_controller/dfplayer/errors", "Sleeping");
      break;
    case SerialWrongStack:
      mqttClient.publish("garage/main_controller/dfplayer/errors", "Get Wrong Stack");
      break;
    case CheckSumNotMatch:
      mqttClient.publish("garage/main_controller/dfplayer/errors", "Check Sum Not Match");
      break;
    case FileIndexOut:
      mqttClient.publish("garage/main_controller/dfplayer/errors", "File Index Out of Bound");
      break;
    case FileMismatch:
      mqttClient.publish("garage/main_controller/dfplayer/errors", "Cannot Find File");
      break;
    case Advertise:
      mqttClient.publish("garage/main_controller/dfplayer/errors", "In Advertise");
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

void printBME280Data()
{
  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  float temp_pres = pres * 0.75006;
  mqttClient.publish((topicPath + "sensors/BME280/temperature").c_str(), String(temp).c_str());
  mqttClient.publish((topicPath + "sensors/BME280/pressure").c_str(), String(temp_pres).c_str());

  temp_inside = temp;
  pressure = temp_pres;
  EnvironmentCalculations::AltitudeUnit envAltUnit = EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit envTempUnit = EnvironmentCalculations::TempUnit_Celsius;

  /// To get correct local altitude/height (QNE) the reference Pressure
  ///    should be taken from meteorologic messages (QNH or QFF)
  float altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);

  float dewPoint = EnvironmentCalculations::DewPoint(temp, hum, envTempUnit);

  /// To get correct seaLevel pressure (QNH, QFF)
  ///    the altitude value should be independent on measured pressure.
  /// It is necessary to use fixed altitude point e.g. the altitude of barometer read in a map
  float seaLevel = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, temp, pres, envAltUnit, envTempUnit);

  float absHum = EnvironmentCalculations::AbsoluteHumidity(temp, hum, envTempUnit);

  mqttClient.publish((topicPath + "sensors/BME280/altitude").c_str(), String(altitude).c_str());
  mqttClient.publish((topicPath + "sensors/BME280/sea_level").c_str(), String(seaLevel).c_str());
  float heatIndex = EnvironmentCalculations::HeatIndex(temp, hum, envTempUnit);
  mqttClient.publish((topicPath + "sensors/BME280/heat_index").c_str(), String(heatIndex).c_str());
  ;
}

void setup()
{
  Serial.begin(115200);
  FPSerial.begin(9600);
  if (!LittleFS.begin())
  {
    // Serial.println("LittleFS mount failed!");
  }
  loadConfig();
  connectWiFi(configJson["wifi_mode"].as<int>());
  Wire.begin();
  PCFKeyboard.begin();
  u8g2.begin();
  mui.begin(u8g2, fds_data, muif_list, sizeof(muif_list) / sizeof(muif_t));
  mySwitch.enableReceive(13);
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
  }
  server.on("/", handleRoot);
  server.on("/get_data", sendData);
  server.on("/save_wifi_set", saveWifiSet);
  server.on("/save_mqtt_set", saveMqttSet);
  server.on("/save_user_set", saveUserSet);
  server.on("/style.css", handleCSS);
  server.on("/script.js", handleJS);
  server.onNotFound(notFound);
  httpUpdater.setup(&server, "/update", configJson["login"].as<const char *>(), configJson["pass"].as<const char *>());
  server.begin();
  mqttClient.setServer(configJson["mqtt_server"].as<const char *>(), configJson["mqtt_port"].as<int>());
  mqttClient.setCallback(callbackMqtt);
  PCFRelay.begin();
  pinMode(33, INPUT);
  pinMode(35, INPUT);
  pinMode(13, INPUT);
  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  switch (bme.chipModel())
  {
  case BME280::ChipModel_BME280:
    Serial.println("Found BME280 sensor! Success.");
    break;
  case BME280::ChipModel_BMP280:
    Serial.println("Found BMP280 sensor! No Humidity available.");
    break;
  default:
    Serial.println("Found UNKNOWN sensor! Error!");
  }

  if (!myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true))
  { // Use serial to communicate with mp3.
    while (true)
      ;
  }
  delay(500);
  myDFPlayer.setTimeOut(500); // Set serial communictaion time out 500ms
  loadDFPlayerConfig();
  myDFPlayer.stop();

  //----Set volume----
  // myDFPlayer.volume(volume_set); // Set volume value (0~30).
  // myDFPlayer.volumeUp();   // Volume Up
  // myDFPlayer.volumeDown(); // Volume Down

  //----Set different EQ----
  //  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  //  myDFPlayer.EQ(DFPLAYER_EQ_POP);
  //  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
  //  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
  //  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
  //  myDFPlayer.EQ(DFPLAYER_EQ_BASS);

  //----Set device we use SD as default----
  //  myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  //  myDFPlayer.outputDevice(DFPLAYER_DEVICE_AUX);
  //  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SLEEP);
  //  myDFPlayer.outputDevice(DFPLAYER_DEVICE_FLASH);

  //----Mp3 control----
  // myDFPlayer.sleep();     //sleep
  //  myDFPlayer.reset();     //Reset the module
  myDFPlayer.enableDAC(); // Enable On-chip DAC
  //  myDFPlayer.disableDAC();  //Disable On-chip DAC
  //  myDFPlayer.outputSetting(true, 15); //output setting, enable the output and set the gain to 15
  //----Mp3 play----
  // myDFPlayer.next(); // Play next mp3
  // delay(1000);
  // myDFPlayer.previous(); // Play previous mp3
  // delay(1000);
  // myDFPlayer.play(1); // Play the first mp3
  // delay(1000);
  // myDFPlayer.loop(1); // Loop the first mp3
  // delay(1000);
  // myDFPlayer.pause(); // pause the mp3
  // delay(1000);
  // myDFPlayer.start(); // start the mp3 from the pause
  // delay(1000);
  // myDFPlayer.playFolder(15, 4); // play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  // delay(1000);
  myDFPlayer.enableLoopAll(); // loop all mp3 files.
  // delay(1000);
  // myDFPlayer.disableLoopAll(); // stop loop all mp3 files.
  // delay(1000);
  // myDFPlayer.playMp3Folder(4); // play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
  // delay(1000);
  // myDFPlayer.advertise(3); // advertise specific mp3 in SD:/ADVERT/0003.mp3; File Name(0~65535)
  // delay(1000);
  // myDFPlayer.stopAdvertise(); // stop advertise
  // delay(1000);
  // myDFPlayer.playLargeFolder(2, 999); // play specific mp3 in SD:/02/004.mp3; Folder Name(1~10); File Name(1~1000)
  // delay(1000);
  // myDFPlayer.loopFolder(5); // loop all mp3 files in folder SD:/05.
  // delay(1000);
  // myDFPlayer.randomAll(); // Random play all the mp3.
  // delay(1000);
  // myDFPlayer.enableLoop(); // enable loop.
  // delay(1000);
  // myDFPlayer.disableLoop(); // disable loop.
  // delay(1000);

  //----Read imformation----
  // Serial.println(myDFPlayer.readState()); // read mp3 state
  // Serial.println(myDFPlayer.readVolume());              // read current volume
  // Serial.println(myDFPlayer.readEQ());                  // read EQ setting
  myDFPlayer.readFileCounts();
  file_count = myDFPlayer.readFileCounts();
  // read all file counts in SD card
  // Serial.println(myDFPlayer.readCurrentFileNumber());   // read current play file number
  // Serial.println(myDFPlayer.readFileCountsInFolder(3)); // read file counts in folder SD:/03
  Serial.end();
  ld2450.begin(Serial, false);
}
void loop(void)
{
  if (mui.isFormActive())
  {

    /* update the display content, if the redraw flag is set */
    if (is_redraw)
    {
      u8g2.firstPage();
      do
      {
        mui.draw();
      } while (u8g2.nextPage());
      if (mui.getCurrentFormId() != 50)
      {
        is_redraw = 0; /* clear the redraw flag */
      }
      menu_show_timer = millis();
    }

    if (millis() - menu_show_timer > 30000)
    { // exit menu if timeout 30 sec
      mui.leaveForm();
      return;
    }

    if (millis() - timerFreez > 300)
    {
      if (analogRead(joystikUpDownPin) < 1000)
      { // down

        mui.nextField();
        is_redraw = 1;
      }
      if (analogRead(joystikUpDownPin) > 3000)
      { // down
        {
          mui.prevField();
          is_redraw = 1;
        }
      }
      if (!PCFKeyboard.read(7))
      { // Ok/select
        if (mui.getCurrentFormId() == 11 && mui.getCurrentCursorFocusPosition() == 3)
        { // Set time
          rtc.adjust(DateTime((String(now.year()).substring(0, 2) + String(year)).toInt(), month, day, time_hours, time_minutes, time_seconds));
          mui.gotoForm(10, 0);
          is_redraw = 1;
          // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
        }
        else if (mui.getCurrentFormId() == 12 && mui.getCurrentCursorFocusPosition() == 3)
        {
          rtc.adjust(DateTime((String(now.year()).substring(0, 2) + String(year)).toInt(), month, day, time_hours, time_minutes, time_seconds));
          mui.gotoForm(10, 0);
          is_redraw = 1;
        }
        else
        {
          mui.sendSelect();
          is_redraw = 1;
        }
      }
      if (!PCFKeyboard.read(1))
      { // Home
        if (mui.getCurrentFormId() == 1)
        { // Exit from menu
          mui.leaveForm();
          return;
        }
        else
        {                                                                 // start menu
          if (mui.getCurrentFormId() > 10 && mui.getCurrentFormId() < 20) // return clock settings
          {
            mui.gotoForm(10, 0);
            is_redraw = 1;
          }
          else if (mui.getCurrentFormId() > 20 && mui.getCurrentFormId() < 30) // return player settings
          {
            mui.gotoForm(20, 0);
            is_redraw = 1;
          }
          else if (mui.getCurrentFormId() > 30 && mui.getCurrentFormId() < 40) // return light settings
          {
            mui.gotoForm(30, 0);
            is_redraw = 1;
          }

          else // return main settings
          {
            mui.gotoForm(1, 0);
            is_redraw = 1;
          }

          is_redraw = 1;
        }
      }
      if (analogRead(joystikLeftRightPin) > 3000 && mui.getCurrentFormId() == 50)
      {
        mainScreen[0] = 0;
        mainScreen[1] = 0;
        mainScreen[2] = 1;
        mui.leaveForm();
        return;
      }
      if (analogRead(joystikLeftRightPin) < 1000 && mui.getCurrentFormId() == 50)
      {
        mainScreen[0] = 0;
        mainScreen[1] = 1;
        mainScreen[2] = 0;
        mui.leaveForm();
        return;
      }
      timerFreez = millis();
    }
  }
  else
  {
    if (mainScreen[0] || mainScreen[1] || mainScreen[2])
    {

      for (size_t i = 0; i < sizeof(mainScreen) / sizeof(mainScreen[0]); i++)
      {
        if (mainScreen[i])
        {
          if (i == 0)
            generalScreenFuncs[i]();
          break;
        }
      }

      if (analogRead(joystikLeftRightPin) < 1000 && millis() - timerFreez > 300 && mui.getCurrentFormId() != 50)
      {
        int count = sizeof(mainScreen) / sizeof(mainScreen[0]);

        for (size_t i = 0; i < count; i++)
        {

          if (mainScreen[i])
          {

            if (i)
            {
              mainScreen[i - 1] = 1;
              mainScreen[i] = 0;
            }

            else
            {
              mainScreen[count - 1] = 1;
              mainScreen[i] = 0;
            }
            break;
          }
        }
        timerFreez = millis();
      }

      if (analogRead(joystikLeftRightPin) > 3000 && millis() - timerFreez > 300 && mui.getCurrentFormId() != 50)
      {
        int count = sizeof(mainScreen) / sizeof(mainScreen[0]);

        for (size_t i = 0; i < count; i++)
        {

          if (mainScreen[i])
          {

            if ((i < count - 1))
            {
              mainScreen[i + 1] = 1;
              mainScreen[i] = 0;
            }

            else
            {
              mainScreen[i - i] = 1;
              mainScreen[i] = 0;
            }
            break;
          }
        }
        timerFreez = millis();
      }
    }
  }
  if (!mui.isFormActive() && !PCFKeyboard.read(7) && mainScreen[0])
  { // open menu
    mui.gotoForm(1, 0);
    is_redraw = true;
    timerFreez = millis();
  }
  else if (!mui.isFormActive() && mainScreen[1])
  {
    mui.gotoForm(50, 0);
    is_redraw = true;
    timerFreez = millis();
  }

  now = rtc.now();
  if (!mui.isFormActive())
  {
    time_hours = now.hour();
    time_minutes = now.minute();
    time_seconds = now.second();
    day = now.day();
    month = now.month();
    year = (uint8_t)String(now.year()).substring(2).toInt();
  }

  if (mySwitch.available())
  {
    // Serial.println(mySwitch.getReceivedValue());
    if (mySwitch.getReceivedValue() == 13675425)
    {
      mqttClient.publish("garage/doors_controller/move_doors/move", "1");
    }
    if (mySwitch.getReceivedValue() == 13675426)
    {
    }
    // A 13675425
    // B 13675426

    // C 13675428
    // D 13675432
    mySwitch.resetAvailable();
  }
  if (strlen(configJson["mqtt_server"].as<const char *>()) > 0)
  {
    if (!mqttClient.connected())
    {
      reconnectMqtt();
    }
    mqttClient.loop();
    unsigned long now = millis();
    if (now - last_msg > 10000)
    {
      if (WiFi.getMode() != WIFI_MODE_AP)
      {
        if (WiFi.status() != WL_CONNECTED)
          connectWiFi(configJson["wifi_mode"].as<int>());
        last_msg = now;
        char topic_buffer[100];
        snprintf(topic_buffer, sizeof(topic_buffer), "Clients_IP/%s/IP", configJson["clientId"].as<const char *>());
        char ipStr[16];
        snprintf(ipStr, sizeof(ipStr), "%s", WiFi.localIP().toString().c_str());
        mqttClient.publish(topic_buffer, ipStr);
      }
    }
    if (millis() - send_mqqt_timer > 1000)
    {
      int valid_targets = ld2450.read();

      if (valid_targets > 0)
      {
        int valid = 0;
        for (int i = 0; i < ld2450.getSensorSupportedTargetCount(); i++)

        {
          const LD2450::RadarTarget result_target = ld2450.getTarget(i);
          if (result_target.valid)
          {
            mqttClient.publish("garage/radar/value", "1");
            mqttClient.publish(String("garage/radar/targets/").c_str(), ld2450.getLastTargetMessage().c_str());
            valid = 1;
          }
        }
        if (!valid)
        {
          mqttClient.publish("garage/radar/value", "0");
        }
      }
      printBME280Data();
      send_mqqt_timer = millis();
    }
  }
  server.handleClient();
  requireChangeSetDFP();
  // for (size_t i = 0; i < 8; i++)
  // {
  //   if (!PCFKeyboard.readButton(i) && i != 3)
  //     Serial.println(i);
  // }
  // if (millis() - read_joystik_timer > 1000)
  // {
  //   Serial.println("-----------------------");
  //   Serial.print("33:");
  //   Serial.println(analogRead(33));
  //   Serial.println("-----------------------");
  //   Serial.print("35:");
  //   Serial.println(analogRead(35));
  //   Serial.println("************************");
  //   read_joystik_timer = millis();
  // }
  // static unsigned long timer = millis();

  if (millis() - timer_save_last_track > 2000)
  {
    int state = myDFPlayer.readState();
    if (state == 1)
    {
      myDFPlayer.readCurrentFileNumber();
      // last_play_track = myDFPlayer.readCurrentFileNumber();
      last_play_track = myDFPlayer.readFileCounts();
    }

    timer_save_last_track = millis();
  }
  if (myDFPlayer.available())
  {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); // Print the detail message from DFPlayer to handle different errors and states.
  }
  if (reset_dfp)
  {
    myDFPlayer.reset();
    reset_dfp = 0;
    ESP.restart();
  }
  //   /*
  //   keys:
  //   pin 35-X left=0 right=4095 avarage = 1800-1900
  //   pin 33-Y up=0 down=4095 avarage = 1850-1950

  //                       |A|
  //  |K|              |D|     |B|
  //         |F| |E|       |C|
  //   A=6
  //   B=0
  //   C=5
  //   D=1
  //   E=4
  //   F=2
  //   K=7
  //   */
}
