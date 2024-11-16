#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ArtnetWifi.h>  // https://github.com/rstephan/ArtnetWifi
#include <ArduinoJson.h>

#include "rgb_led.h"
#include "webinterface.h"

#define MIN(x,y) (x<y ? x : y)
#define MAX(x,y) (x>y ? x : y)

#define ENABLE_UART
#define ENABLE_WEBINTERFACE
#define ENABLE_MDNS

#ifdef ENABLE_UART
#define USE_SERIAL_BREAK
/* UART for DMX output */
#define SEROUT_UART 1
#define DMX_BREAK 92
#define DMX_MAB 12
#endif // ENABLE_UART

const char* host = "ARTNET";
const char* version = __DATE__ " / " __TIME__;

Config config;
WebServer server(80);
ArtnetWifi artnet;
WiFiManager wifiManager;

long tic_loop = 0, tic_fps = 0, tic_packet = 0, tic_web = 0;
unsigned long packetCounter = 0, frameCounter = 0, last_packet_received = 0;
float fps = 0;

struct {
  uint16_t universe;
  uint16_t length;
  uint8_t sequence;
#ifdef ENABLE_UART
  uint8_t uart_data[512];
#endif
} global;

#ifdef ENABLE_ARDUINO_OTA
#include <ArduinoOTA.h>
bool arduinoOtaStarted = false;
unsigned int last_ota_progress = 0;
#endif

#ifdef ENABLE_UART
long tic_uart = 0;
unsigned long uartCounter;
#endif

void onDmxPacket(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data) {
  unsigned long now = millis();
  if (now - last_packet_received > 1000) {
    Serial.print("Received DMX data\n");
  }
  last_packet_received = now;

  if ((millis() - tic_fps) > 1000 && frameCounter > 100) {
    Serial.print("packetCounter = ");
    Serial.print(packetCounter++);
    fps = 1000 * frameCounter / (millis() - tic_fps);
    tic_fps = last_packet_received;
    frameCounter = 0;
    Serial.print(", FPS = ");             Serial.print(fps);
    Serial.print(", length = ");          Serial.print(length);
    Serial.print(", sequence = ");        Serial.print(sequence);
    Serial.print(", universe = ");        Serial.print(universe);
    Serial.print(", config.universe = "); Serial.print(universe);
    Serial.println();
  }

  if (universe == config.universe) {
    global.universe = universe;
    global.sequence = sequence;
    #ifdef ENABLE_UART
    if (length <= 512) global.length = length;
    for (int i = 0; i < global.length; i++) {
      global.uart_data[i] = data[i];
    }
    #endif
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Setup starting");

  ledInit();

  #ifdef ENABLE_UART
  Serial1.begin(250000, SERIAL_8N2);
  #endif

  global.universe = 0;
  global.sequence = 0;
  global.length = 512;

  #ifdef ENABLE_UART
  for (int i = 0; i < 512; i++) global.uart_data[i] = 0;
  #endif

  SPIFFS.begin(true);

  if (loadConfig()) {
    ledYellow();
    delay(1000);
  } else {
    ledRed();
    delay(1000);
  }

  wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  wifiManager.autoConnect(host);
  Serial.println("connected");

  if (WiFi.status() != WL_CONNECTED) ledRed();

  WiFi.hostname(host);
  wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));

  #ifdef STANDALONE_PASSWORD
  wifiManager.autoConnect(host, STANDALONE_PASSWORD);
  #else
  wifiManager.autoConnect(host);
  #endif
  Serial.println("connected");

  if (WiFi.status() == WL_CONNECTED) ledGreen();

  #ifdef ENABLE_ARDUINO_OTA
  Serial.println("Initializing Arduino OTA");
  ArduinoOTA.setHostname(host);
  ArduinoOTA.setPassword(ARDUINO_OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    allBlack();
    digitalWrite(LED_B, ON);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    allBlack();
    digitalWrite(LED_R, ON);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    analogWrite(LED_B, 4096 - (20 * millis()) % 4096);
    if (progress != last_ota_progress) {
      Serial.printf("OTA Progress: %u%%\n", (progress / (total / 100)));
      last_ota_progress = progress;
    }
  });
  ArduinoOTA.onEnd([]() {
    allBlack();
    digitalWrite(LED_G, ON);
    delay(500);
    allBlack();
  });
  Serial.println("Arduino OTA init complete");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Starting Arduino OTA (setup)");
    ArduinoOTA.begin();
    arduinoOtaStarted = true;
  }
  #endif

  #ifdef ENABLE_WEBINTERFACE

  server.onNotFound(handleNotFound);

  server.on("/", HTTP_GET, []() {
    tic_web = millis();
    handleRedirect("/index.html");
  });

  server.on("/defaults", HTTP_GET, []() {
    tic_web = millis();
    Serial.println("handleDefaults");
    handleStaticFile("/reload_success.html");
    defaultConfig();
    saveConfig();
    server.close();
    server.stop();
    ESP.restart();
  });

  server.on("/reconnect", HTTP_GET, []() {
    tic_web = millis();
    Serial.println("handleReconnect");
    handleStaticFile("/reload_success.html");
    ledRed();
    server.close();
    server.stop();
    delay(5000);
    WiFiManager wifiManager;
    wifiManager.resetSettings();
    wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    wifiManager.startConfigPortal(host);
    Serial.println("connected");
    server.begin();
    if (WiFi.status() == WL_CONNECTED) ledGreen();
  });

  server.on("/restart", HTTP_GET, []() {
    tic_web = millis();
    Serial.println("handleRestart");
    handleStaticFile("/reload_success.html");
    ledRed();
    server.close();
    server.stop();
    SPIFFS.end();
    delay(5000);
    ESP.restart();
  });

  server.on("/dir", HTTP_GET, [] {
    tic_web = millis();
    handleDirList();
  });

  server.on("/json", HTTP_PUT, [] {
    tic_web = millis();
    handleJSON();
  });

  server.on("/json", HTTP_POST, [] {
    tic_web = millis();
    handleJSON();
  });

  server.on("/json", HTTP_GET, [] {
    tic_web = millis();
    DynamicJsonDocument root(300);
    N_CONFIG_TO_JSON(universe, "universe");
    N_CONFIG_TO_JSON(channels, "channels");
    N_CONFIG_TO_JSON(delay, "delay");
    root["version"] = version;
    root["uptime"]  = long(millis() / 1000);
    root["packets"] = packetCounter;
    root["fps"]     = fps;
    String str;
    serializeJson(root, str);
    server.send(200, "application/json", str);
  });

  server.on("/update", HTTP_GET, [] {
    tic_web = millis();
    handleStaticFile("/update.html");
  });

  server.on("/update", HTTP_POST, handleUpdate1, handleUpdate2);

  // start the web server
  server.begin();
  #endif 

  #ifdef ENABLE_MDNS
  MDNS.begin(host);
  MDNS.addService("http", "tcp", 80);
  #endif

  artnet.begin();
  artnet.setArtDmxCallback(onDmxPacket);

  tic_loop = millis();
  tic_packet = millis();
  tic_fps = millis();
  tic_web = 0;

  #ifdef ENABLE_UART
  tic_uart = 0;
  #endif

  Serial.println("Setup done");
} // setup

void loop() {
  long now = millis();
  if (now - last_packet_received > 1000) {
    wifiManager.process();
    #ifdef ENABLE_ARDUINO_OTA
    if (WiFi.status() == WL_CONNECTED && !arduinoOtaStarted) {
      Serial.println("Starting Arduino OTA (loop)");
      ArduinoOTA.begin();
          arduinoOtaStarted = true; // remember that it started
    }
    ArduinoOTA.handle();
    #endif
  }
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    ledRed();
    delay(10);
    #ifndef ENABLE_STANDALONE
    return;
    #endif
  }

  if ((millis() - tic_web) < 5000) {
    // give feedback that the webinterface is active
    ledBlue();
    delay(25);
  } else {
    ledGreen();
    artnet.read();

    // this section gets executed at a maximum rate of around 40Hz
    if ((millis() - tic_loop) > config.delay) {
      long now = millis();
      tic_loop = now;
      frameCounter++;

      #ifdef ENABLE_UART
      #ifdef USE_SERIAL_BREAK
      Serial1.flush();
      Serial1.begin(90000, SERIAL_8N2);
      while (Serial1.available()) Serial1.read();
      // send the break as a "slow" byte
      Serial1.write(0);
      // switch back to the original baud rate
      Serial1.flush();
      Serial1.begin(250000, SERIAL_8N2);
      while (Serial1.available()) Serial1.read();
      #else
      SET_PERI_REG_MASK(UART_CONF0(SEROUT_UART), UART_TXD_BRK);
      delayMicroseconds(DMX_BREAK);
      CLEAR_PERI_REG_MASK(UART_CONF0(SEROUT_UART), UART_TXD_BRK);
      delayMicroseconds(DMX_MAB);
      #endif

      Serial1.write(0); // Start-Byte
      // send out the value of the selected channels (up to 512)
      for (int i = 0; i < MIN(global.length, config.channels); i++) {
        Serial1.write(global.uart_data[i]);
      }

      uartCounter++;
      if ((now - tic_uart) > 1000 && uartCounter > 100) {
        // don't estimate the FPS too frequently
        float pps = (1000.0 * uartCounter) / (now - tic_uart);
        tic_uart = now;
        uartCounter = 0;
        Serial.printf("UART: %.1f p/s\n", pps);
      }
      #endif
    }
  }

  #ifdef WITH_TEST_CODE
  testCode();
  #endif
}

#ifdef WITH_TEST_CODE
void testCode() {
  long now = millis();
  uint8_t x = (now / 60) % 240;
  if (x > 120) {
    x = 240 - x;
  }

  #ifdef ENABLE_UART
  global.uart_data[1] = x; // x: 0 - 170
  global.uart_data[2] = 0; // x fine
  global.uart_data[3] = x; // y: 0: -horz. 120: vert, 240: +horz
  global.uart_data[4] = 0; // y fine
  global.uart_data[5] = 30; // color wheel: red
  global.uart_data[6] = 0; // pattern
  global.uart_data[7] = 0; // strobe
  global.uart_data[8] = 150; // brightness
  #endif // ENABLE_UART
}
#endif // WITH_TEST_CODE
