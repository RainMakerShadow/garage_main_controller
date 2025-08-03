
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

WiFiClient espClient;
PubSubClient mqttClient(espClient);
RTC_DS3231 rtc;
DateTime now;
StaticJsonDocument<256> configJson;
WebServer server(80);
HTTPUpdateServer httpUpdater;
PCF8574 PCF_keyboard(0x27);
PCF8574 PCF_relay(0x26);
#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266)) // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/4, /*tx =*/5);
#define FPSerial softSerial
#else
#define FPSerial Serial2
#endif
// I2C device found at 0x26 PCF 1
// I2C device found at 0x27 PCF 2
// I2C device found at 0x57 EEPROM DS3231
// I2C device found at 0x68 DS3231
// I2C device found at 0x76 BME2800

DFRobotDFPlayerMini myDFPlayer;
// Створюємо об'єкт дисплея для SPI
// U8G2_ST7920_128X64_F_1_HAL_HW_SPI(cs, reset) -- Fast full framebuffer
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* reset=*/U8X8_PIN_NONE);
MUIU8G2 mui;
RCSwitch mySwitch = RCSwitch();

const char *www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";

unsigned long lastMsg;
int timeout_mqtt_reconnect = 3000;
unsigned long timer;
unsigned long readJoystikTimer;
float temp_inside = 28.7;
float temp_outside = -12.3;
float pressure = 46;
float humidity = 758.8;
const char *wifi_modes[] = {"Station", "Access point"};

#define joystikLeftRightPin 35
#define joystikUpDownPin 33
uint8_t timeHours;
uint8_t timeMinutes;
uint8_t timeSeconds;
uint8_t day;
uint8_t month;
uint8_t year;

/*display mui*/

uint8_t num_value = 0;
uint8_t bar_value = 0;
uint16_t animal_idx = 0;

/*
  list of animal names
*/
unsigned long key_timer;
unsigned long menu_show_timer;
const char *animals[] = {"Bird", "Bison", "Cat", "Crow", "Dog", "Elephant", "Fish", "Gnu", "Horse", "Koala", "Lion", "Mouse", "Owl", "Rabbit", "Spider", "Turtle", "Zebra"};

uint16_t animal_name_list_get_cnt(void *data)
{
  return sizeof(animals) / sizeof(*animals); /* number of animals */
}

const char *animal_name_list_get_str(void *data, uint16_t index)
{
  return animals[index];
}

uint8_t mui_hrule(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    u8g2.drawHLine(0, mui_get_y(ui), u8g2.getDisplayWidth());
  }
  return 0;
}

uint8_t show_my_data(mui_t *ui, uint8_t msg)
{
  if (msg == MUIF_MSG_DRAW)
  {
    u8g2_uint_t x = mui_get_x(ui);
    u8g2_uint_t y = mui_get_y(ui);
    u8g2.setCursor(x + 5, y);
    u8g2.print("Num:");
    u8g2.setCursor(x + 50, y);
    u8g2.print(num_value);

    u8g2.setCursor(x + 5, y + 12);
    u8g2.print("Bar:");
    u8g2.setCursor(x + 50, y + 12);
    u8g2.print(bar_value);

    u8g2.setCursor(x + 5, y + 24);
    u8g2.print("Animal:");
    u8g2.setCursor(x + 50, y + 24);
    u8g2.print(animal_idx);
    u8g2.print("=");
    u8g2.print(animals[animal_idx]);
  }
  return 0;
}

muif_t muif_list[] = {
    MUIF_U8G2_FONT_STYLE(0, u8g2_font_helvR08_tr), /* regular font */
    MUIF_U8G2_FONT_STYLE(1, u8g2_font_helvB08_tr), /* bold font */

    MUIF_RO("HR", mui_hrule),
    MUIF_U8G2_LABEL(),
    MUIF_RO("GP", mui_u8g2_goto_data),
    MUIF_BUTTON("GC", mui_u8g2_goto_form_w1_pi),

    MUIF_U8G2_U8_MIN_MAX("HC", &timeHours, 0, 23, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("MC", &timeMinutes, 0, 59, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("SC", &timeSeconds, 0, 59, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("YD", &year, 0, 99, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("MD", &month, 1, 12, mui_u8g2_u8_min_max_wm_mud_pi),
    MUIF_U8G2_U8_MIN_MAX("DD", &day, 1, 31, mui_u8g2_u8_min_max_wm_mud_pi),

    MUIF_U8G2_U8_MIN_MAX("NV", &num_value, 0, 99, mui_u8g2_u8_min_max_wm_mse_pi),
    MUIF_U8G2_U8_MIN_MAX_STEP("NB", &bar_value, 0, 16, 1, MUI_MMS_2X_BAR, mui_u8g2_u8_bar_wm_mse_pf),
    MUIF_U8G2_U16_LIST("NA", &animal_idx, NULL, animal_name_list_get_str, animal_name_list_get_cnt, mui_u8g2_u16_list_line_wa_mse_pi),

    /* register custom function to show the data */
    MUIF_RO("SH", show_my_data),

    /* a button for the menu... */
    MUIF_BUTTON("GO", mui_u8g2_btn_goto_wm_fi)};

fds_t fds_data[] =

    MUI_FORM(1)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Settings")
                MUI_STYLE(0)
                    MUI_XY("HR", 0, 11)
                        MUI_DATA("GP",
                                 MUI_10 "Enter Data|" MUI_12 "Show Data|" MUI_20 "Clock Settings")
                            MUI_XYA("GC", 5, 24, 0)
                                MUI_XYA("GC", 5, 36, 1)
                                    MUI_XYA("GC", 5, 48, 2)
    //
    MUI_FORM(10)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Enter Data")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_LABEL(5, 23, "Num:")
                            MUI_LABEL(5, 35, "Bar:")
                                MUI_LABEL(5, 47, "Animal:")
                                    MUI_XY("NV", 50, 23)
                                        MUI_XY("NB", 50, 35)
                                            MUI_XY("NA", 50, 47)
                                                MUI_XYAT("GO", 114, 60, 1, " Ok ")
    //
    MUI_FORM(12)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Show Data")
                MUI_XY("HR", 0, 11)
                    MUI_STYLE(0)
                        MUI_XY("SH", 0, 23)
                            MUI_XYAT("GO", 114, 60, 1, " Ok ")
    //
    MUI_FORM(20)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Clock settings")
                MUI_STYLE(0)
                    MUI_XY("HR", 0, 11)
                        MUI_DATA("GP",
                                 MUI_21 "Set clock|" MUI_22 "Set date")
                            MUI_XYA("GC", 5, 24, 0)
                                MUI_XYA("GC", 5, 36, 1)
    //
    MUI_FORM(21)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Clock settings")
                MUI_STYLE(0)
                    MUI_XY("HR", 0, 11)
                        MUI_LABEL(5, 23, "Hours:")
                            MUI_LABEL(5, 35, "Minutes:")
                                MUI_LABEL(5, 47, "Seconds:")
                                    MUI_XY("HC", 50, 23)
                                        MUI_XY("MC", 50, 35)
                                            MUI_XY("SC", 50, 47)
                                                MUI_XYAT("G0", 114, 60, 1, " Ok ")
                                                    MUI_GOTO(50, 20, 20, "OK")
    //
    MUI_FORM(22)
        MUI_STYLE(1)
            MUI_LABEL(5, 8, "Date settings")
                MUI_STYLE(0)
                    MUI_XY("HR", 0, 11)
                        MUI_LABEL(5, 23, "Day:")
                            MUI_LABEL(5, 35, "Month:")
                                MUI_LABEL(5, 47, "Year:")
                                    MUI_XY("DD", 50, 23)
                                        MUI_XY("MD", 50, 35)
                                            MUI_XY("YD", 50, 47)
                                                MUI_XYAT("G0", 114, 60, 1, " Ok ");
/*-------------------------*/

unsigned long timerFreez;
int mainScreen[] = {1, 0, 0};
void generalScreen();
void playerScreen();
void lightsInfo();
void (*generalScreenFuncs[])() = {generalScreen, playerScreen, lightsInfo};
int main_menu[] = {0, 0, 0, 0};
const char *text_main_menu[] = {"Clock", "Music", "Light", "Relay"};
int clock_menu[] = {0, 0};
const char *text_clock_menu[] = {"Set time", "Set date"};
int music_menu[] = {};
const char *text_music_menu[] = {"EQ", "Loop", "Random", "Reset"};

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String topicPath = "garden/summer_shower/";
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
    Serial.println("Ошибка создания файла!");
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
    Serial.println("File \"config.json\" not found!");
  }
  DeserializationError error = deserializeJson(configJson, file);
  file.close();
  if (error)
  {
    Serial.println("Error load \"config.json\"!");
    return;
  }
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
  u8g2.print(convertFloatToString(pressure, 4) + " %");
}

void printHumidity()
{
  u8g2.setColorIndex(1);
  u8g2.setCursor(90, 63);
  u8g2.print(convertFloatToString(humidity, 3));
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

  u8g2.firstPage();
  do
  {
    printDate();
  } while (u8g2.nextPage());
}
void lightsInfo()
{
  u8g2.firstPage();
  do
  {
    printDate();
  } while (u8g2.nextPage());
}
void reconnectMqtt()
{
  if ((int)(millis() - timer) > timeout_mqtt_reconnect)
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
    timer = millis();
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
      Serial.print("Connecting to WiFi");
      int tries = 0;
      while (WiFi.status() != WL_CONNECTED && tries < 20)
      { // ждём до 20 секунд
        delay(1000);
        Serial.print(".");
        tries++;
      }
      Serial.println();
      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.println("WiFi connected!");
      }
      else
      {
        Serial.println("WiFi NOT connected!");
        Serial.println("WiFi NOT connected! Starting access point.");
        if (WiFi.softAP(configJson["ssid"].as<const char *>(), configJson["wifi_pass"].as<const char *>()))
        {
          Serial.println("Access point started!");
          Serial.print("IP adress: ");
          Serial.println(WiFi.softAPIP());
        }
      }
    }
  }
  else if (mode == 2)
  {
    if (WiFi.softAP(configJson["ssid"].as<const char *>(), configJson["wifi_pass"].as<const char *>()))
    {
      Serial.println("Access point started!");
      Serial.print("IP adress: ");
      Serial.println(WiFi.softAPIP());
    }
    else
    {
      Serial.println("Faild to start access point!");
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
    Serial.print(value);
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

void setup()
{
  Serial.begin(115200);
  FPSerial.begin(9600);
  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed!");
  }
  loadConfig();
  connectWiFi(configJson["wifi_mode"].as<int>());
  Wire.begin();
  PCF_keyboard.begin();
  u8g2.begin();
  mui.begin(u8g2, fds_data, muif_list, sizeof(muif_list) / sizeof(muif_t));
  // mui.gotoForm(1, 0);
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
  WiFi.printDiag(Serial);
  Serial.println(WiFi.isConnected());
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());
  PCF_relay.begin();
  pinMode(33, INPUT);
  pinMode(35, INPUT);
  pinMode(13, INPUT);

  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true))
  { // Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true)
      ;
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.setTimeOut(500); // Set serial communictaion time out 500ms

  //----Set volume----
  myDFPlayer.volume(25);   // Set volume value (0~30).
  myDFPlayer.volumeUp();   // Volume Up
  myDFPlayer.volumeDown(); // Volume Down

  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
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
  //  myDFPlayer.sleep();     //sleep
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
  // Serial.println(myDFPlayer.readState());               // read mp3 state
  // Serial.println(myDFPlayer.readVolume());              // read current volume
  // Serial.println(myDFPlayer.readEQ());                  // read EQ setting
  // Serial.println(myDFPlayer.readFileCounts());          // read all file counts in SD card
  // Serial.println(myDFPlayer.readCurrentFileNumber());   // read current play file number
  // Serial.println(myDFPlayer.readFileCountsInFolder(3)); // read file counts in folder SD:/03
}
uint8_t is_redraw = 1;
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
      is_redraw = 0; /* clear the redraw flag */
      menu_show_timer = millis();
    }
    if (millis() - menu_show_timer > 30000)
    { // exit menu if timeout 30 sec
      mui.leaveForm();
      return;
    }
    /* handle events */
    // switch (u8g2.getMenuEvent())
    // {
    // case U8X8_MSG_GPIO_MENU_SELECT:
    //   mui.sendSelect();
    //   is_redraw = 1;
    //   break;
    // case U8X8_MSG_GPIO_MENU_NEXT:
    //   mui.nextField();
    //   is_redraw = 1;
    //   break;
    // case U8X8_MSG_GPIO_MENU_PREV:
    //   mui.prevField();
    //   is_redraw = 1;
    //   break;
    // }
    if (millis() - timerFreez > 300)
    {
      if (analogRead(33) < 1800)
      { // down
        mui.nextField();
        is_redraw = 1;
      }
      if (analogRead(33) > 2000)
      { // down
        mui.prevField();
        is_redraw = 1;
      }
      if (!PCF_keyboard.read(7))
      { // Ok/select
        if (mui.getCurrentFormId() == 21 && mui.getCurrentCursorFocusPosition() == 3)
        { // Set time
          rtc.adjust(DateTime((String(now.year()).substring(0, 2) + String(year)).toInt(), month, day, timeHours, timeMinutes, timeSeconds));
          mui.gotoForm(20, 0);
          is_redraw = 1;
          // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
        }
        else if (mui.getCurrentFormId() == 22 && mui.getCurrentCursorFocusPosition() == 3)
        {
          rtc.adjust(DateTime((String(now.year()).substring(0, 2) + String(year)).toInt(), month, day, timeHours, timeMinutes, timeSeconds));
          mui.gotoForm(20, 0);
          is_redraw = 1;
        }
        else
        {
          mui.sendSelect();
          is_redraw = 1;
        }
      }
      if (!PCF_keyboard.read(1))
      { // Home
        if (mui.getCurrentFormId() == 1)
        { // Exit from menu
          mui.leaveForm();
          return;
        }
        else
        { // start menu
          if (mui.getCurrentFormId() > 20 && mui.getCurrentFormId() < 30)
          {
            mui.gotoForm(20, 0);
          }
          else
          {
            mui.gotoForm(1, 0);
          }

          is_redraw = 1;
        }
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
          generalScreenFuncs[i]();
          break;
        }
      }
      if (analogRead(joystikLeftRightPin) <= 1800 && millis() - timerFreez > 300)
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
      if (analogRead(joystikLeftRightPin) >= 2000 && millis() - timerFreez > 300)
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
  if (!mui.isFormActive() && !PCF_keyboard.read(7))
  { // open menu
    mui.gotoForm(1, 0);
    is_redraw = true;
    timerFreez = millis();
  }

  now = rtc.now();
  if (!mui.isFormActive())
  {
    timeHours = now.hour();
    timeMinutes = now.minute();
    timeSeconds = now.second();
    day = now.day();
    month = now.month();
    year = (uint8_t)String(now.year()).substring(2).toInt();
  }

  if (mySwitch.available())
  {
    Serial.println(mySwitch.getReceivedValue());
    if (mySwitch.getReceivedValue() == 13675425)
    {
      mqttClient.publish("garage/doors_controller/move_doors/move", "1");
    }
    if (mySwitch.getReceivedValue() == 13675426)
    {
      // mqttClient.publish("garage/doors_controller/move_doors/close", "1");
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
    if (now - lastMsg > 10000)
    {
      if (WiFi.getMode() != WIFI_MODE_AP)
      {
        if (WiFi.status() != WL_CONNECTED)
          connectWiFi(configJson["wifi_mode"].as<int>());
        lastMsg = now;
        char topicBuffer[100];
        snprintf(topicBuffer, sizeof(topicBuffer), "Clients_IP/%s/IP", configJson["clientId"].as<const char *>());
        char ipStr[16];
        snprintf(ipStr, sizeof(ipStr), "%s", WiFi.localIP().toString().c_str());
        mqttClient.publish(topicBuffer, ipStr);
      }
    }
  }
  server.handleClient();
  // for (size_t i = 0; i < 8; i++)
  // {
  //   if (!PCF_keyboard.readButton(i) && i != 3)
  //     Serial.println(i);
  // }
  // if (millis() - readJoystikTimer > 1000)
  // {
  //   Serial.println("-----------------------");
  //   Serial.print("33:");
  //   Serial.println(analogRead(33));
  //   Serial.println("-----------------------");
  //   Serial.print("35:");
  //   Serial.println(analogRead(35));
  //   Serial.println("************************");
  //   readJoystikTimer = millis();
  // }
  //   static unsigned long timer = millis();

  //   // if (millis() - timer > 3000)
  //   // {
  //   //   timer = millis();
  //   //   myDFPlayer.next(); // Play next mp3 every 3 second.
  //   // }

  //   if (myDFPlayer.available())
  //   {
  //     printDetail(myDFPlayer.readType(), myDFPlayer.read()); // Print the detail message from DFPlayer to handle different errors and states.
  //   }
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
