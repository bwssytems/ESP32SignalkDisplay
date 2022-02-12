/* Includes ------------------------------------------------------------------*/

#if (!defined(ESP32))
#error This code is intended to run only on the ESP32 platform! Please check your Tools->Board setting.
#endif

#include <WiFi.h>
#include <Update.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>

#define ESP_ASYNC_WIFIMANAGER_VERSION_MIN_TARGET "ESPAsync_WiFiManager v1.9.2"

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _ESPASYNC_WIFIMGR_LOGLEVEL_ 3

//#define BASIC_SERIAL_DISPLAY
//#define EPD_DISPLAY
#define TFT_DISPLAY

#ifdef EPD_DISPLAY
#include "EPDDisplayMarineData.h"
#endif

#ifdef TFT_DISPLAY
#include "TFTDisplayMarineData.h"
#endif
// #define M5STICK

#ifdef M5STICK
#include "M5Atom.h"
bool toggleM5color = false;
#endif

WiFiMulti wifiMulti;
// You only need to format the filesystem once
// #define FORMAT_FILESYSTEM true
#define FORMAT_FILESYSTEM false

#define FileFS SPIFFS
#define FS_Name "SPIFFS"

#define LED_BUILTIN 2
#define LED_ON HIGH
#define LED_OFF LOW

#include <SPIFFSEditor.h>

// SSID and PW for Config Portal
String ssid = "SK_DisplayWifi_" + String((uint32_t)ESP.getEfuseMac(), HEX);
String password;

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;
// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES false

// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
//#define USE_STATIC_IP_CONFIG_IN_CP          false

// Use false to disable NTP config. Advisable when using Cellphone, Tablet to access Config Portal.
// See Issue 23: On Android phone ConfigPortal is unresponsive (https://github.com/khoih-prog/ESP_WiFiManager/issues/23)
#define USE_ESP_WIFIMANAGER_NTP false
// Use true to enable CloudFlare NTP service. System can hang if you don't have Internet access while accessing CloudFlare
// See Issue #21: CloudFlare link in the default portal (https://github.com/khoih-prog/ESP_WiFiManager/issues/21)
#define USE_CLOUDFLARE_NTP false

#define USING_CORS_FEATURE false
//////
#define USE_DHCP_IP true
#define USE_CONFIGURABLE_DNS false

#include <ESPAsync_WiFiManager.h> //https://github.com/khoih-prog/ESPAsync_WiFiManager

String host = String(ssid);

#define HTTP_PORT 80

AsyncWebServer server(HTTP_PORT);
// DNSServer dnsServer;
size_t content_len;
int previousProgress;

AsyncEventSource events("/events");

String http_username = "admin";
String http_password = "admin";

String separatorLine = "===============================================================";

const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<h2>OTA bin file selection</h2>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

uint8_t connectMultiWiFi()
{
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS 800L

#define WIFI_MULTI_CONNECT_WAITING_MS 500L

  uint8_t status;

  LOGERROR(F("ConnectMultiWiFi with :"));

  if ((Router_SSID != "") && (Router_Pass != ""))
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass);
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  LOGERROR(F("Connecting MultiWifi..."));

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ((i++ < 20) && (status != WL_CONNECTED))
  {
    status = WiFi.status();

    if (status == WL_CONNECTED)
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if (status == WL_CONNECTED)
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP().toString());
  }
  else
  {
    LOGERROR(F("WiFi not connected"));

    // ESP.restart();
  }

  return status;
}

// format bytes
String formatBytes(size_t bytes)
{
  if (bytes < 1024)
  {
    return String(bytes) + "B";
  }
  else if (bytes < (1024 * 1024))
  {
    return String(bytes / 1024.0) + "KB";
  }
  else if (bytes < (1024 * 1024 * 1024))
  {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
  else
  {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

void toggleLED()
{
  // toggle state
#ifdef M5STICK
  if (toggleM5color)
  {
    M5.dis.drawpix(0, 0xfe0000); // green
  }
  else
  {
    M5.dis.drawpix(0, 0x000000); // off
  }
  toggleM5color = !toggleM5color;
#else
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif
}

void heartBeatPrint()
{
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print(F("H\n")); // H means connected to WiFi
  else
    Serial.print(F("F\n")); // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(F(" "));
  }
}

void check_WiFi()
{
  if ((WiFi.status() != WL_CONNECTED))
  {
    Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
#ifdef M5STICK
    M5.dis.drawpix(0, 0xabfe00); // orange
#endif

    while (!connectMultiWiFi())
    {
      delay(1000);
    }

#ifdef M5STICK
    M5.dis.drawpix(0, 0xfe0000); // green
#endif
  }
}

void check_status()
{
  static ulong checkstatus_timeout = 0;
  static ulong LEDstatus_timeout = 0;
  static ulong checkwifi_timeout = 0;

  static ulong current_millis;

#define WIFICHECK_INTERVAL 1000L

#define HEARTBEAT_INTERVAL 10000L

#define LED_INTERVAL 2000L

  current_millis = millis();

  // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }

  if ((current_millis > LEDstatus_timeout) || (LEDstatus_timeout == 0))
  {
    // Toggle LED at LED_INTERVAL = 2s
    toggleLED();
    LEDstatus_timeout = current_millis + LED_INTERVAL;
  }

  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
  }
}

// Variable to store the HTTP request
String deviceName;

int signalk_read_bytes = 0;
int signalk_write_bytes = 0;

int restartDelayCount = 0;

String signalkServerURI;
bool signalkServerDefined;

const int capacity = JSON_OBJECT_SIZE(2);
const char *PARAM_SIGNALK_URI = "signalkURI";
const char *PARAM_ALARM_DEPTH = "alarmDepth";

bool saveParams = false;

WebSocketsClient signalkWebSocket;

bool wsConnected;
bool wsConnectStarted;
unsigned long myTime;

DynamicJsonDocument signalkData(8192);

void initDisplay();
void displayDepth(float theDepth);

#ifdef EPD_DISPLAY
// Create a new image cache
UBYTE *BackgroundImage;
/* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
UWORD Imagesize = ((EPD_4IN2_WIDTH % 8 == 0) ? (EPD_4IN2_WIDTH / 8) : (EPD_4IN2_WIDTH / 8 + 1)) * EPD_4IN2_HEIGHT;

int begin_x, begin_y, end_x, end_y;
int refresh_counter;
#endif

String _depth;
String _prev_depth;
String _depth_unit_disp;
bool prev_alarm;

float prev_depth;
float alarm_depth;
int counter;
int loop_counter;
int change_counter;
int depth_units;

#define USE_SERIAL Serial
#define FEET 1
#define METRIC 0
#define METER_CONVERSION 3.2808f

#ifdef EPD_DISPLAY
#define REFRESH_RATE 20 // # of display changes to redraw circle

#define DEPTH_LOC_X 36
#define DEPTH_LOC_Y 108

#define UNIT_LOC_X 326
#define UNIT_LOC_Y 234

#define DEPTH_CIRCLE_X 170
#define DEPTH_CIRCLE_Y 150
#define DEPTH_CIRCLE_RADIUS 144
#endif

#ifdef TFT_DISPLAY
#endif

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16)
{
  const uint8_t *src = (const uint8_t *)mem;
  USE_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for (uint32_t i = 0; i < len; i++)
  {
    if (i % cols == 0)
    {
      USE_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
    }
    USE_SERIAL.printf("%02X ", *src);
    src++;
  }
  USE_SERIAL.printf("\n");
}

void signalkWebSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  float new_depth = 0.0f;

  switch (type)
  {
  case WStype_PING:
    USE_SERIAL.printf("[WSc] received ping - %s\n", payload);
    break;
  case WStype_PONG:
    USE_SERIAL.printf("[WSc] received pong - %s\n", payload);
    break;
  case WStype_FRAGMENT_BIN_START:
    USE_SERIAL.printf("[WSc] Fragment start binary length: %u\n", length);
    hexdump(payload, length);
    break;
  case WStype_FRAGMENT:
    USE_SERIAL.printf("[WSc] received fragment text- %s\n", payload);
    break;
  case WStype_FRAGMENT_FIN:
    USE_SERIAL.printf("[WSc] received fragment fin text- %s\n", payload);
    break;
  case WStype_FRAGMENT_TEXT_START:
    USE_SERIAL.printf("[WSc] received fragment start text- %s\n", payload);
    break;
  case WStype_BIN:
    USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
    hexdump(payload, length);
    break;
  case WStype_ERROR:
    USE_SERIAL.printf("[WSc] received error - %s\n", payload);
    break;
  case WStype_DISCONNECTED:
    USE_SERIAL.printf("[WSc] Disconnected!\n");
    wsConnected = false;
    break;
  case WStype_CONNECTED:
    USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);
    wsConnected = true;

    signalkWebSocket.sendTXT("{\"context\":\"*\",\"unsubscribe\":[{\"path\":\"*\"}]}");
    delay(1000);
    signalkWebSocket.sendTXT("{\"context\":\"vessels.self\",\"subscribe\":[{\"path\":\"environment.depth.belowTransducer\"}]}");
    // signalkWebSocket.sendTXT("{\"context\":\"vessels.self\",\"subscribe\":[{\"path\":\"environment.inside.temperature\"}]}");
    break;
  case WStype_TEXT:
    // USE_SERIAL.printf("[WSc] received payload: %s\n", payload);
    counter++;
    signalk_read_bytes = signalk_read_bytes + length;
    DeserializationError err = deserializeJson(signalkData, payload);

    if (err.code() == DeserializationError::Ok)
    {
      JsonObject json = signalkData.as<JsonObject>();
      if (json.containsKey("updates"))
      {
        JsonArray updates = signalkData["updates"];
        for (JsonObject anUpdate : updates)
        {
          JsonArray values = anUpdate["values"];
          for (JsonObject aValue : values)
          {
            String pathName = aValue["path"];
            if (pathName == "environment.depth.belowTransducer")
            {
              new_depth = aValue["value"].as<float>();
            }
          }
        }
      }
    }
    else
    {
      USE_SERIAL.printf("[WSc] Could not decode signalk json: %s - with error: %s\n", payload, err.c_str());
    }
    signalkData.clear();

    if (depth_units == FEET)
    {
      new_depth = new_depth * METER_CONVERSION;
    }

    // Test Alarm by making the depth lower
    /*
    if (counter % 20 == 0 || counter % 21 == 0 || counter % 22 == 0)
    {
      new_depth = 2.8F;
    }
    */
    _depth = String(new_depth, 2);

    if (new_depth != prev_depth)
    {
      USE_SERIAL.printf("[WSc] display depth changed: %s%s - msgs #%d - changed #%d\n", _depth.c_str(), _depth_unit_disp.c_str(), counter, change_counter);
      change_counter++;
      displayDepth(new_depth);
    }
    else
    {
      // USE_SERIAL.printf("[WSc] depth not changed: %s%s - ", _depth.c_str(), _depth_unit_disp.c_str());
    }

    // USE_SERIAL.printf("counters: msgs #%d - changed #%d\n", counter, change_counter);
    prev_depth = new_depth;
    _prev_depth = _depth;
    break;
  }
}

bool restartRequested = false;

// flag for saving data
bool shouldSaveConfig = false;

int _timeStampMillis;
char _timeString[26];
String formatDateNow();

// callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setGlobalConfigParams()
{
}

void checkAndSaveParams()
{
  if (saveParams)
  {
    setGlobalConfigParams();
    String theJson = "{\"signalkURI\":\"" + signalkServerURI + "\",\"alarmDepth\":" + String(alarm_depth, 2) + "}";
    USE_SERIAL.println(theJson + "\r\n");
    //    if (FileFS.begin())
    //    {
    if (FileFS.exists("/config.json"))
    {
      FileFS.remove("/config.json");
    }
    File afile = FileFS.open("/config.json", FILE_WRITE);
    if (!afile)
    {
      Serial.println("Could not open /config.json.");
    }
    else
    {
      if (afile.print(theJson.c_str()))
      {
        Serial.println("File was written");
      }
      else
      {
        Serial.println("File write failed");
      }

      afile.close();
    }
    File root = FileFS.open("/");
    File file = root.openNextFile();

    while (file)
    {
      String fileName = file.name();
      size_t fileSize = file.size();
      Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
      file = root.openNextFile();
    }
    //    }
    //    else
    //    {
    //      Serial.println("Could not open SPIFFS to write params.");
    //    }
    //    FileFS.end();
    saveParams = false;
  }
}

String createHttpContentStart()
{
  return String("<!DOCTYPE html><html lang=\"en\">") +
         String("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">") +
         String("<meta charset=\"utf-8\">") +
         String("<link rel=\"icon\" href=\"data:,\">") +
         String("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}") +
         String(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;") +
         String("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}") +
         String(".button2 {background-color: #77878A;}</style></head>") +
         String("<body><h1>") + deviceName + String(" Device Server</h1>");
}

String createHttpContentEnd()
{
  return String("</body></html>\r\n\r\n");
}

String createRefresh()
{
  return createHttpContentStart() +
         String("<script>window.location.href = \"http://") +
         WiFi.localIP().toString() +
         String("/\"</script>") +
         createHttpContentEnd();
}

void handleRoot(AsyncWebServerRequest *request)
{
  String signalkStatus;
  if (wsConnected)
  {
    signalkStatus = "connected</p>";
  }
  else
  {
    signalkStatus = "none connected</p>";
  }

  String message = createHttpContentStart();
  message += String("<p>SignalK read bytes " + String(signalk_read_bytes) + "</p>");
  message += String("<p>SignalK write bytes " + String(signalk_write_bytes) + "</p>");
  message += String("<H2>ESP32 SignalK Display</H2>");
  message += String("<H3>Depth</H3>");
  message += String("<pre>");
  message += String("\nDepth in ");
  message += String(_depth_unit_disp);
  message += String(": ");
  message += String(_depth);
  message += String("</pre>");
  message += String("<p>SignalK server ");
  message += signalkStatus;
  message += String("<form action=\"/get\">");
  message += String("Signalk URI <input type=\"text\" name=\"");
  message += String(PARAM_SIGNALK_URI);
  message += String("\" value=\"");
  message += String(signalkServerURI);
  message += String("\"><br>");
  message += String("Alarm Depth <input type=\"text\" name=\"");
  message += String("alarmDepth");
  message += String("\" value=\"");
  message += String(alarm_depth);
  message += String("\"><br>");
  message += String("<input type=\"submit\" value=\"Submit\">");
  message += String("</form><br>");
  message += String("<form action=\"/get\">");
  message += String("</form><br>");
  message += String("<p><a href=\"/ota\"><button class=\"button button2\">OTA</button></a></p>");
  message += String("<p><a href=\"/resetbytes\"><button class=\"button button2\">Reset Byte Counter</button></a></p>");
  message += String("<p><a href=\"/restartunit\"><button class=\"button button2\">Restart Device</button></a></p>");
  message += String("<p><a href=\"/resetunit\"><button class=\"button button2\">Reset Device</button></a></p>");
  message += createHttpContentEnd();
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->print(message.c_str());
  request->send(response);
}

void handleOTA(AsyncWebServerRequest *request)
{
  String message = createHttpContentStart();
  message += String(serverIndex);
  message += createHttpContentEnd();
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->print(message.c_str());
  request->send(response);
}

void printProgress(size_t prg, size_t sz)
{
  int currentProgress = (prg * 100) / content_len;
  if(currentProgress >= (previousProgress + 5) || (currentProgress == 1 && previousProgress == 0) || (currentProgress >= 99 && previousProgress < 99)) {
    Serial.printf("Progress: %d%%\n", currentProgress);
    if(currentProgress > 1) {
      previousProgress = currentProgress;
    }
  }
}

void handleUpload(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final)
{
  if (!index)
  {
    previousProgress = 0;
    content_len = request->contentLength();
    int cmd = (filename.indexOf("spiffs") > -1) ? U_SPIFFS : U_FLASH;
    Serial.printf("Update: %s\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
    { // start with max available size
      Update.printError(Serial);
    }
  }

  /* flashing firmware to ESP*/
  if (Update.write(data, len) != len)
  {
    Update.printError(Serial);
  }

  if (final)
  {
    String message = createHttpContentStart();
    message += String("<script>window.location.href = \"http://") + WiFi.localIP().toString() + String("/\"</script>");
    message += "Update Success - Please wait while the device reboots";
    message += createHttpContentEnd();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", message);
    request->send(response);
    delay(10);
    if (Update.end(true))
    { // true to set the size to the current progress
      Serial.printf("Progress: 100%%\n");
      Serial.printf("Update Success \nRebooting...\n");
      Serial.flush();
      ESP.restart();
    }
    else
    {
      Update.printError(Serial);
    }
  }
}

void handleUploadError(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}

void handleGet(AsyncWebServerRequest *request)
{
  String signalk_uri;
  float new_alarm;
  bool dataChanged = false;

  // GET PARAM_SIGNALK_URI value on <ESP_IP>/get?input1=<inputMessage>
  if (request->hasParam(PARAM_SIGNALK_URI))
  {
    signalk_uri = String(request->getParam(PARAM_SIGNALK_URI)->value());
    if (signalk_uri != signalkServerURI)
    {
      dataChanged = true;
      signalkServerURI = signalk_uri;
      signalkServerURI.trim();
      USE_SERIAL.print("Paramater for SignalK URI changed to ");
      USE_SERIAL.println(signalkServerURI);
      signalkServerDefined = true;
    }
  }

  if (request->hasParam(PARAM_ALARM_DEPTH))
  {
    new_alarm = atof((request->getParam(PARAM_ALARM_DEPTH)->value()).c_str());
    if (new_alarm != alarm_depth)
    {
      dataChanged = true;
      alarm_depth = new_alarm;
      if (alarm_depth < 0.5F)
      {
        alarm_depth = 0.5F;
      }
      USE_SERIAL.print("Paramater for alarm depth changed to ");
      USE_SERIAL.println(String(alarm_depth, 2));
    }
  }

  if (dataChanged)
  {
    saveParams = true;
    checkAndSaveParams();
  }

  Serial.println("Submit request sent to your ESP on signalk URI with value: " + request->getParam(PARAM_SIGNALK_URI)->value() + ", data did change: " + dataChanged);
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->print(createRefresh().c_str());
  request->send(response);
}

void handleNotFound(AsyncWebServerRequest *request)
{
  String message = createHttpContentStart();
  message += "Not Found";
  Serial.print(F("NOT_FOUND: "));

  if (request->method() == HTTP_GET)
    Serial.print(F("GET"));
  else if (request->method() == HTTP_POST)
    Serial.print(F("POST"));
  else if (request->method() == HTTP_DELETE)
    Serial.print(F("DELETE"));
  else if (request->method() == HTTP_PUT)
    Serial.print(F("PUT"));
  else if (request->method() == HTTP_PATCH)
    Serial.print(F("PATCH"));
  else if (request->method() == HTTP_HEAD)
    Serial.print(F("HEAD"));
  else if (request->method() == HTTP_OPTIONS)
    Serial.print(F("OPTIONS"));
  else
    Serial.print(F("UNKNOWN"));

  Serial.println(" http://" + request->host() + request->url());

  if (request->contentLength())
  {
    Serial.println("_CONTENT_TYPE: " + request->contentType());
    Serial.println("_CONTENT_LENGTH: " + request->contentLength());
  }

  int headers = request->headers();

  for (int i = 0; i < headers; i++)
  {
    AsyncWebHeader *h = request->getHeader(i);
    Serial.println("_HEADER[" + h->name() + "]: " + h->value());
  }

  int params = request->params();

  for (int i = 0; i < params; i++)
  {
    AsyncWebParameter *p = request->getParam(i);

    if (p->isFile())
    {
      Serial.println("_FILE[" + p->name() + "]: " + p->value() + ", size: " + p->size());
    }
    else if (p->isPost())
    {
      Serial.println("_POST[" + p->name() + "]: " + p->value());
    }
    else
    {
      Serial.println("_GET[" + p->name() + "]: " + p->value());
    }
  }

  message += createHttpContentEnd();
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->print(message.c_str());
  request->send(response);
}

void handleResetBytes(AsyncWebServerRequest *request)
{
  Serial.println("reset SignalK read/write bytes counter");
  signalk_read_bytes = 0;
  signalk_write_bytes = 0;
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->print(createRefresh().c_str());
  request->send(response);
}

void handleRestartUnit(AsyncWebServerRequest *request)
{
  Serial.println("restarting temp sensor device unit");
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->print(createRefresh().c_str());
  request->send(response);
  restartRequested = true;
}

void handleResetUnit(AsyncWebServerRequest *request)
{
  Serial.println("reseting all params temp sensor device unit");
  File reqFile = FileFS.open("/reset.req", "w");
  if (!reqFile)
  {
    Serial.println("failed to open reset.req for writing");
  }
  else
  {
    reqFile.println("reset");
    reqFile.close();
    FileFS.end();
    restartRequested = true;
  }
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  response->print(createRefresh().c_str());
  request->send(response);
}

/* Entry point ----------------------------------------------------------------*/
void setup()
{
  signalkServerDefined = false;
  wsConnectStarted = false;
  // USE_SERIAL.begin(921600);
  USE_SERIAL.begin(115200);

  // Serial.setDebugOutput(true);
  USE_SERIAL.setDebugOutput(true);

  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();

  for (uint8_t t = 4; t > 0; t--)
  {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

// set led pin as output
#ifndef M5STICK
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  restartDelayCount = 0;
  delay(200);

#ifdef M5STICK
  M5.begin(true, false, true);
  delay(50);
  M5.dis.drawpix(0, 0xf0f0f0); // white
#endif
  alarm_depth = 0.0F;
  deviceName = String(ssid);

  Serial.println(deviceName + " SignalK Display Setup Started");
  Serial.print(F("\nStarting Async_ESP32_FSWebServer using "));
  Serial.print(FS_Name);
  Serial.print(F(" on "));
  Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION);

  if (String(ESP_ASYNC_WIFIMANAGER_VERSION) < ESP_ASYNC_WIFIMANAGER_VERSION_MIN_TARGET)
  {
    Serial.print("Warning. Must use this code on Version later than : ");
    Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION_MIN_TARGET);
  }

  Serial.setDebugOutput(false);

  if (FORMAT_FILESYSTEM)
  {
    USE_SERIAL.println("Format file system requested....");
    FileFS.format();
    ESPAsync_WiFiManager ESPAsync_Reset_wifiManager(&server, NULL, "AsyncESP32-FSWebServer");
    ESPAsync_Reset_wifiManager.resetSettings();
    delay(200);
    USE_SERIAL.println("Formatted FileFS and Reset wifi settings, re-program to continue...");
    while (true)
    {
      delay(20000);
    }
  }

  if (FileFS.begin())
  {
    Serial.println("mounted file system");
    File root = FileFS.open("/");
    File file = root.openNextFile();

    while (file)
    {
      String fileName = file.name();
      size_t fileSize = file.size();
      Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
      file = root.openNextFile();
    }

    Serial.println("checking for display unit reset params");
    if (FileFS.exists("/reset.req"))
    {
      Serial.println("Display reset params requested");
      FileFS.remove("/reset.req");
      FileFS.remove("/config.json");
      ESPAsync_WiFiManager ESPAsync_Reset_wifiManager(&server, NULL, "AsyncESP32-FSWebServer");
      delay(200);
      ESPAsync_Reset_wifiManager.resetSettings();
      delay(200);
      ESP.restart();
    }
    else
    {
      Serial.println("No Display Reset, continuing....");
    }

    if (FileFS.exists("/config.json"))
    {
      Serial.println("Found config file.");
      File file = FileFS.open("/config.json");
      if (!file)
      {
        Serial.println("Failed to open /config.json for reading");
      }
      else
      {
        String theParams = file.readString();
        USE_SERIAL.print("The params from config.json: ");
        USE_SERIAL.println(theParams);
        if (theParams.length() > 0)
        {
          DynamicJsonDocument configData(128);

          const char *json = theParams.c_str();

          deserializeJson(configData, json);

          signalkServerURI = configData["signalkURI"].as<String>();
          signalkServerURI.trim();

          USE_SERIAL.print("SignalK Server is ");
          USE_SERIAL.println(signalkServerURI);
          if (signalkServerURI != NULL)
          {
            signalkServerDefined = true;
          }

          alarm_depth = configData["alarmDepth"].as<float>();
          USE_SERIAL.println("alarm depth is " + String(alarm_depth, 2));
          if (alarm_depth == 0.0F)
          {
            alarm_depth = 0.5F;
          }
        }
        else
        {
          USE_SERIAL.println("No Display params in file...");
          signalkServerDefined = false;
        }
        file.close();
      }
    }
    else
    {
      alarm_depth = 0.5F;
      Serial.println("No Display config, continuing with defaults...");
    }
  }
  else
  {
    Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));
    signalkServerURI = "";
    signalkServerDefined = false;

    if (!FileFS.begin())
    {
      // prevents debug info from the library to hide err message.
      delay(100);

#if USE_LITTLEFS
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
      Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif

      while (true)
      {
        delay(1);
      }
    }
  }
  Serial.println();
  // end read
  //  FileFS.end();

#ifdef M5STICK
  M5.dis.drawpix(0, 0xabfe00); // orange
#endif

#ifndef M5STICK
  digitalWrite(LED_BUILTIN, LED_ON);
#endif

  DNSServer dnsServer;

  ESPAsync_WiFiManager ESPAsync_wifiManager(&server, &dnsServer, host.c_str());

  ESPAsync_wifiManager.setMinimumSignalQuality(-1);

  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESPAsync_wifiManager.setConfigPortalChannel(0);
  //////

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESPAsync_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

  // We can't use WiFi.SSID() in ESP32as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();

  // Remove this line if you do not want to see WiFi password printed
  Serial.println("ESP Self-Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  // SSID to uppercase
  ssid.toUpperCase();
  password = "signalkdev1.";
  if (Router_SSID == "0")
  {
    Router_SSID = "";
    Router_Pass = "";
    // Starts an access point
    if (!ESPAsync_wifiManager.startConfigPortal((const char *)ssid.c_str(), password.c_str()))
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    else
      Serial.println(F("WiFi connected...yeey :)"));
  }
  else
  {
    Serial.println("Connecting wifimulti with SSID: " + Router_SSID + " w/ len: " + Router_SSID.length() + " and PWD: " + Router_Pass);
    connectMultiWiFi();
  }

  /*use mdns for host name resolution*/
  if (!MDNS.begin(ssid.c_str()))
  {
    Serial.println("Error setting up MDNS responder!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  // SERVER INIT
  events.onConnect([](AsyncEventSourceClient *client)
                   { client->send("hello!", NULL, millis(), 1000); });

  server.addHandler(&events);

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(ESP.getFreeHeap())); });

  server.addHandler(new SPIFFSEditor(FileFS, http_username, http_password));

  server.on("/", handleRoot);
  server.on("/ota", handleOTA);
  server.on(
      "/update", HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data,
         size_t len, bool final)
      { handleUpload(request, filename, index, data, len, final); });
  server.on("/resetbytes", handleResetBytes);
  server.on("/restartunit", handleRestartUnit);
  server.on("/resetunit", handleResetUnit);
  server.on("/get", handleGet);
  server.onNotFound(handleNotFound);
  server.begin();
  Update.onProgress(printProgress);
  Serial.print(F("HTTP server started @ "));
  Serial.println(WiFi.localIP().toString());

  Serial.println(separatorLine);
  Serial.print("Open http://");
  Serial.print(WiFi.localIP().toString());
  Serial.println("/edit to see the file browser");
  Serial.println("Using username = " + http_username + " and password = " + http_password);
  Serial.println(separatorLine);

  digitalWrite(LED_BUILTIN, LED_OFF);

#ifdef M5STICK
  M5.dis.drawpix(0, 0x0000f0); // blue
  delay(3000);                 // wait a second

  M5.dis.drawpix(0, 0xfefe00); // yellow
#endif

  wsConnected = false;
  _depth = String("  0.00");
  _prev_depth = String("  0.00");
  prev_depth = 0.0f;
  counter = 0;
  loop_counter = 0;
  change_counter = 0;
  depth_units = FEET;
  prev_alarm = false;

  if (depth_units == FEET)
  {
    _depth_unit_disp = "FT";
  }
  else
  {
    _depth_unit_disp = "M";
  }

  initDisplay();
}

/* The main loop -------------------------------------------------------------*/
void loop()
{
  if (wsConnectStarted)
  {
    signalkWebSocket.loop();
  }
  else if (!wsConnectStarted && signalkServerDefined)
  {
    USE_SERIAL.print("Connecting to websocket on port 3000 at address: ");
    USE_SERIAL.println(signalkServerURI);
    // server address, port and URL
    signalkWebSocket.begin(signalkServerURI, 3000, "/signalk/v1/stream?subscribe=none");
    // signalkWebSocket.begin("10.0.0.3", 3000, "/signalk/v1/stream?subscribe=none");

    // event handler
    signalkWebSocket.onEvent(signalkWebSocketEvent);

    // use HTTP Basic Authorization this is optional remove if not needed
    // signalkWebSocket.setAuthorization("user", "Password");

    // try ever 5000 again if connection has failed
    signalkWebSocket.setReconnectInterval(5000);
    wsConnectStarted = true;
  }
  else
  {
    USE_SERIAL.println("No SignalK Server defined, waiting for definition from web gui....");
    delay(2000);
  }
  loop_counter++;
  if (loop_counter >= 200000)
  {
    myTime = millis();

    USE_SERIAL.print("[");
    USE_SERIAL.print(myTime);
    USE_SERIAL.print("] Websocket checked another 200k - conn status ");
    USE_SERIAL.println(wsConnected);
    loop_counter = 0;
  }
}

#ifdef EPD_DISPLAY
void initDisplay()
{
  refresh_counter = 0;

  USE_SERIAL.printf("Initializing EPD E-Ink Display\r\n");
  DEV_Module_Init();
  DEV_Delay_ms(1000);
  USE_SERIAL.printf("e-Paper Init and Clear...\r\n");
  EPD_4IN2_Init();
  DEV_Delay_ms(1000);
  EPD_4IN2_Clear();
  DEV_Delay_ms(1000);

  if ((BackgroundImage = (UBYTE *)malloc(Imagesize)) == NULL)
  {
    USE_SERIAL.printf("Failed to apply for background memory...\r\n");
    while (1)
      ;
  }
  USE_SERIAL.printf("Paint_NewImage\r\n");
  Paint_NewImage(BackgroundImage, EPD_4IN2_WIDTH, EPD_4IN2_HEIGHT, 0, WHITE);
  // Paint_Clear(WHITE);
  Paint_ClearWindows(0, 0, EPD_4IN2_WIDTH, EPD_4IN2_HEIGHT, WHITE);
  Paint_DrawCircle(DEPTH_CIRCLE_X, DEPTH_CIRCLE_Y, DEPTH_CIRCLE_RADIUS, BLACK, DOT_PIXEL_3X3, DRAW_FILL_EMPTY);
  Paint_DrawString_EN(UNIT_LOC_X, UNIT_LOC_Y, _depth_unit_disp.c_str(), &Font48, BLACK, WHITE);

  Paint_DrawDepthPoint(DEPTH_LOC_X, DEPTH_LOC_Y, &Font72, BLACK, WHITE);

  Paint_DrawDepth(DEPTH_LOC_X, DEPTH_LOC_Y, _depth.c_str(), &Font72, BLACK, WHITE);
  EPD_4IN2_Display(BackgroundImage);
}

void displayDepth(float theDepth)
{
  refresh_counter++;
  if (refresh_counter >= REFRESH_RATE)
  {
    USE_SERIAL.println("[EPD_DISPLAY] Refresh counter reached...");
    refresh_counter = 0;
    begin_x = (DEPTH_CIRCLE_X - DEPTH_CIRCLE_RADIUS) - 3;
    begin_y = (DEPTH_CIRCLE_Y - DEPTH_CIRCLE_RADIUS) - 3;
    end_x = (DEPTH_CIRCLE_X + DEPTH_CIRCLE_RADIUS) + 3;
    end_y = (DEPTH_CIRCLE_Y + DEPTH_CIRCLE_RADIUS) + 3;
    // begin_x = 0;
    // begin_y = 0;
    // end_x = EPD_4IN2_WIDTH;
    // end_y = EPD_4IN2_HEIGHT;
    Paint_ClearWindows(begin_x, begin_y, end_x, end_y, WHITE);
    Paint_ClearWindows(UNIT_LOC_X, UNIT_LOC_Y, UNIT_LOC_X + (Font48.Width * 2), UNIT_LOC_Y + Font48.Height, WHITE);
    Paint_DrawString_EN(UNIT_LOC_X, UNIT_LOC_Y, _depth_unit_disp.c_str(), &Font48, BLACK, WHITE);
    Paint_DrawCircle(DEPTH_CIRCLE_X, DEPTH_CIRCLE_Y, DEPTH_CIRCLE_RADIUS, BLACK, DOT_PIXEL_3X3, DRAW_FILL_EMPTY);
    Paint_DrawDepthPoint(DEPTH_LOC_X, DEPTH_LOC_Y, &Font72, BLACK, WHITE);
    Paint_DrawDepth(DEPTH_LOC_X, DEPTH_LOC_Y, theDepth, &Font72, BLACK, WHITE);
    EPD_4IN2_PartialDisplay(begin_x, begin_y, end_x, end_y, BackgroundImage);
    EPD_4IN2_PartialDisplay(UNIT_LOC_X, UNIT_LOC_Y, UNIT_LOC_X + (Font48.Width * 2), UNIT_LOC_Y + Font48.Height, BackgroundImage);
  }
  else
  {
    begin_x = DEPTH_LOC_X;
    begin_y = DEPTH_LOC_Y;
    end_x = DEPTH_LOC_X + (Font72.Width * 5);
    end_y = DEPTH_LOC_Y + Font72.Height;
    Paint_ClearWindows(begin_x, begin_y, DEPTH_LOC_X + (Font72.Width * 3), end_y, WHITE);
    Paint_ClearWindows(DEPTH_LOC_X + (Font72.Width / 2) + (Font72.Width * 3), begin_y, DEPTH_LOC_X + (Font72.Width / 2) + (Font72.Width * 2), end_y, WHITE);
    Paint_DrawDepth(DEPTH_LOC_X, DEPTH_LOC_Y, _depth.c_str(), &Font72, BLACK, WHITE);
    EPD_4IN2_PartialDisplay(begin_x, begin_y, DEPTH_LOC_X + (Font72.Width * 3), end_y, BackgroundImage);
    EPD_4IN2_PartialDisplay(DEPTH_LOC_X + (Font72.Width / 2) + (Font72.Width * 3), begin_y, DEPTH_LOC_X + (Font72.Width / 2) + (Font72.Width * 2), end_y, BackgroundImage);
  }
}
#endif

#ifdef BASIC_SERIAL_DISPLAY
void initDisplay()
{
  USE_SERIAL.println("Using Basic Serial Display");
}

void displayDepth(float theDepth)
{
}
#endif

#ifdef TFT_DISPLAY
void initDisplay()
{
  USE_SERIAL.println("Using TFT Display");
  TFT_InitDisplay(_depth_unit_disp);
}

void displayDepth(float theDepth)
{
  if (theDepth <= alarm_depth && !prev_alarm)
  {
    prev_alarm = true;
  }
  else if (theDepth > alarm_depth && prev_alarm)
  {
    prev_alarm = false;
  }

  TFT_DrawDepth(theDepth, prev_alarm);
}
#endif