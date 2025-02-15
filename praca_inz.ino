#include <Arduino.h>
#include <ESP8266WiFi.h>         // https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager
#include <ArtnetWifi.h>          // https://github.com/rstephan/ArtnetWifi
#include <ArduinoJson.h>

#include "rgb_led.h"
#include "webinterface.h"

#define MIN(x,y) (x<y ? x : y)
#define MAX(x,y) (x>y ? x : y)

// Piny dla przekaźnika_1 i przekaźnika_2
const int ledPin = D3;    
const int relayPin = D6;  

// Piny dla dimera
#define ZERO_CROSS_PIN D7     
#define DIMMER_PIN D8         
 
// Zmienne przekaźnika_1 i przekaźnika_2
bool ledState = false;    // Stan przekaźnika_1
bool relayState = false;  // Stan przekaźnik_2
volatile int brightness = 128; // Jasność dimmera (0-255)
 
 // Pin w WEMOS D1 do podłoczenia do 485 
#define ENABLE_UART //D4


//#define ENABLE_STANDALONE
//#define STANDALONE_PASSWORD "WIFI_secret"

// Włącz OTA (programowanie bezprzewodowe w Arduino GUI, nie przez serwer WWW)
//#define ENABLE_ARDUINO_OTA
//#define ARDUINO_OTA_PASSWORD "OTA_WI_FI"

// włącznie interfejsu webowego do ustawienia kanału 
#define ENABLE_WEBINTERFACE

// włączenie multicast dns
#define ENABLE_MDNS



#ifdef ENABLE_UART
#include "c_types.h"
#include "eagle_soc.h"
#include "uart_register.h"

// istnieją dwie różne implementacje dla przerwy, jedna wykorzystująca szereg, druga wykorzystująca czasy niskiego poziomu; obie powinny działać
#define USE_SERIAL_BREAK

// UART for DMX output 
#define SEROUT_UART 1
// Minimalne czasy DMX zgodnie z E1.11 
#define DMX_BREAK 92
#define DMX_MAB 12
#endif // ENABLE_UART

const char* host = "ART-NET";
const char* version = __DATE__ " / " __TIME__;

Config config;
ESP8266WebServer server(80);
ArtnetWifi artnet;
WiFiManager wifiManager;

//śledź czas wywołań funkcji
long tic_loop = 0, tic_fps = 0, tic_packet = 0, tic_web = 0;
unsigned long packetCounter = 0, frameCounter = 0, last_packet_received = 0;
float fps = 0;

// Globalny bufor z jednym wszechświatem Artnet

struct  {
  uint16_t universe;
  uint16_t length;
  uint8_t sequence;
#ifdef ENABLE_UART
  uint8_t uart_data[512];
#endif

} global;

#ifdef ENABLE_ARDUINO_OTA
#include <ArduinoOTA.h>

// Śledź, czy OTA zostało uruchomione
bool arduinoOtaStarted = false;
unsigned int last_ota_progress = 0;
#endif

#ifdef ENABLE_UART
long tic_uart = 0;
unsigned long uartCounter;
#endif



// Ta funkcja będzie wywoływana dla każdego otrzymanego pakietu UDP
void onDmxPacket(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t * data) {

  unsigned long now = millis();
  if (now - last_packet_received > 1000) {
    Serial.print("Received DMX data\n");
  }
  last_packet_received = now;

  // pokazuj informacje zwrotne raz na sekundę
  if ((millis() - tic_fps) > 1000 && frameCounter > 100) {
    Serial.print("packetCounter = ");
    Serial.print(packetCounter++);

   // nie sprawdzaj FPS zbyt często
    fps = 1000 * frameCounter / (millis() - tic_fps);
    tic_fps = last_packet_received;
    frameCounter = 0;
    Serial.print(", FPS = ");             Serial.print(fps);
    // pokaż także pewne informacje diagnostyczne
    Serial.print(", length = ");          Serial.print(length);
    Serial.print(", sequence = ");        Serial.print(sequence);
    Serial.print(", universe = ");        Serial.print(universe);
    Serial.print(", config.universe = "); Serial.print(universe);
    Serial.println();
  }

  if (universe == config.universe) {
    // kopiuj dane z pakietu UDP
    global.universe = universe;
    global.sequence = sequence;

#ifdef ENABLE_UART
    if (length <= 512)
      global.length = length;
    for (int i = 0; i < global.length; i++) {
      global.uart_data[i] = data[i];
    }
#endif
  }
} // onDmxpacket

//dimmer
// Funkcja obsługująca przerwanie przy przejściu przez zero
void IRAM_ATTR zeroCrossInterrupt() {
  int delayTime = map(brightness, 0, 255, 0, 1000);
  delayMicroseconds(delayTime);
  digitalWrite(DIMMER_PIN, HIGH);
  delayMicroseconds(10); // Krótki impuls
  digitalWrite(DIMMER_PIN, LOW);
}


void setup() {

 // Serial0 służy do celów debugowania
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Setup starting");

// skonfiguruj trzy piny wyjściowe dla diody LED stanu RGB
  ledInit();

#ifdef ENABLE_UART
// skonfiguruj trzy piny wyjściowe dla diody LED stanu RGB
  Serial1.begin(250000, SERIAL_8N2);
#endif

  global.universe = 0;
  global.sequence = 0;
  global.length = 512;

#ifdef ENABLE_UART
  for (int i = 0; i < 512; i++)
    global.uart_data[i] = 0;
#endif

// System plików SPIFFS zawiera kod HTML i JavaScript dla interfejsu internetowego
  SPIFFS.begin();

  if (loadConfig()) {
    ledYellow();
    delay(1000);
  }
  else {
    ledRed();
    delay(1000);
  }

  WiFiManager wifiManager;
  wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  wifiManager.autoConnect(host);
  Serial.println("connected");

  if (WiFi.status() != WL_CONNECTED)
    ledRed();

#ifdef ENABLE_STANDALONE
  Serial.println("Starting WiFiManager (non-blocking mode)");
  wifiManager.setConfigPortalBlocking(false);
#else
  Serial.println("Starting WiFiManager (blocking mode)");
#endif

  WiFi.hostname(host);
  wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));

#ifdef STANDALONE_PASSWORD
  wifiManager.autoConnect(host, STANDALONE_PASSWORD);
#else
  wifiManager.autoConnect(host);
#endif
  Serial.println("connected");

  if (WiFi.status() == WL_CONNECTED)
    ledGreen();

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
  ArduinoOTA.onEnd([]()   {
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

// obsługuje wszystkie identyfikatory URI, które można rozwiązać do pliku w systemie plików SPIFFS
  server.onNotFound(handleNotFound);


//?
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
    if (WiFi.status() == WL_CONNECTED)
      ledGreen();
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

  // start serwera
  server.begin();

#endif // ifdef ENABLE_WEBINTERFACE

#ifdef ENABLE_MDNS
  // ogłoś nazwę hosta i serwer WWW przez zeroconf
  MDNS.begin(host);
  MDNS.addService("http", "tcp", 80);
#endif

  artnet.begin();
  artnet.setArtDmxCallback(onDmxPacket);

  // zainicjuj wszystkie timery
  tic_loop   = millis();
  tic_packet = millis();
  tic_fps    = millis();
  tic_web    = 0;

#ifdef ENABLE_UART
  tic_uart    = 0;
#endif


  Serial.println("Setup done");
  

  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  pinMode(DIMMER_PIN, OUTPUT);

  digitalWrite(ledPin, LOW);
  digitalWrite(relayPin, LOW);
  digitalWrite(DIMMER_PIN, LOW);

attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), zeroCrossInterrupt, FALLING);



    // Endpoint do sterowania przekaźnikiem_1
    server.on("/toggle_led", []() {
      ledState = !ledState; // Zmień stan przekaźnika_1
      digitalWrite(ledPin, ledState ? HIGH : LOW);
      server.send(200, "text/plain", ledState ? "ON" : "OFF");
      Serial.println(ledState ? "LED ON" : "LED OFF");
    });
  
    // Endpoint do sterowania przekaźnikiem_2
    server.on("/toggle_relay", []() {
      relayState = !relayState; // Zmień stan przekaźnika_2
      digitalWrite(relayPin, relayState ? HIGH : LOW);
      server.send(200, "text/plain", relayState ? "ON" : "OFF");
      Serial.println(relayState ? "Relay ON" : "Relay OFF");
    });

    
  server.on("/set_brightness", []() {
  if (server.hasArg("value")) {
    brightness = server.arg("value").toInt();
    server.send(200, "text/plain", "Brightness set to " + String(brightness));
  } else {
    server.send(400, "text/plain", "Bad Request");
       
  }
});

server.on("/status", []() {
  String status = "{\"led\":\"" + String(ledState ? "ON" : "OFF") +
                  "\",\"relay\":\"" + String(relayState ? "ON" : "OFF") +
                  "\",\"dimmer\":\"" + String(brightness) + "\"}";
  server.send(200, "application/json", status);
});


analogWrite(DIMMER_PIN, brightness);  


}
 // setup


void loop() {
// obsługuj żądania wifiManager i arduinoOTA tylko wtedy, gdy nie otrzymujesz nowych danych DMX
  long now = millis();
  if (now - last_packet_received > 1000) {
    wifiManager.process();
#ifdef ENABLE_ARDUINO_OTA
    if (WiFi.status() == WL_CONNECTED && !arduinoOtaStarted) {
      Serial.println("Starting Arduino OTA (loop)");
      ArduinoOTA.begin();
      arduinoOtaStarted = true; 
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
// przekaż informację zwrotną, że interfejs sieciowy jest aktywny
    ledBlue();
    delay(25);
  }
  else  {
    ledGreen();
    artnet.read();

    // ta sekcja jest wykonywana z maksymalną częstotliwością około 40 Hz
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
      // send break using low-level code
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
// loop



#ifdef WITH_TEST_CODE
void testCode() {
  long now = millis();
  uint8_t x = (now / 60) % 240;
  if (x > 120) {
    x = 240 - x;
  }

  //Serial.printf("x: %d\n", x);
#ifdef ENABLE_UART
  global.uart_data[1] =   x;// x 0 - 170
  global.uart_data[2] =   0;// x fine
  global.uart_data[3] =   x;// y: 0: -horz. 120: vert, 240: +horz
  global.uart_data[4] =   0;// y fine
  global.uart_data[5] =  30;// color wheel: red
  global.uart_data[6] =   0;// pattern
  global.uart_data[7] =   0;// strobe
  global.uart_data[8] = 150; // brightness
#endif // ENABLE_UART
}
#endif // WITH_TEST_CODE
