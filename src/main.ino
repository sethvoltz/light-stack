// TODO: Convert from .ino to .cpp with header .h

// =--------------------------------------------------------------------------------= Libraries =--=

#include <limits.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <OneButton.h>


// =----------------------------------------------------------------------------= Configuration =--=

// Wifi
#define SETUP_AP_NAME                 "Setup Light Stack"
#define SETUP_AP_PASSWORD             "setuplightstack"
#define MDNS_PREFIX                   "light-stack-"
#define MDNS_SERVICE_NAME             "light-stack"
#define WIFI_HOTSPOT_TIMEOUT          180 // Seconds before hotspot ends and attempts reconnect

// MQTT
#define MQTT_ONLINE_MESSAGE           "online"
#define MQTT_OFFLINE_MESSAGE          "offline"
#define MQTT_RECONNECT_DELAY          3000 // ms Delay between initial connection attempts
#define MQTT_ROOT                     "light-stack"
#define DEFAULT_MQTT_SERVER           ""
#define MQTT_SERVER_LENGTH            64
#define DEFAULT_MQTT_PORT             "1883"
#define MQTT_PORT_LENGTH              6
#define DEFAULT_MQTT_USERNAME         ""
#define MQTT_USERNAME_LENGTH          32
#define DEFAULT_MQTT_PASSWORD         ""
#define MQTT_PASSWORD_LENGTH          32
#define MQTT_MESSAGE_BUFFER_SIZE      3072

// OTA
#define OTA_USERNAME                  "update"
#define OTA_PASSWORD_LENGTH           32
#define DEFAULT_OTA_PASSWORD          "light-stack"

// Buttons
#define MAIN_BUTTON_PIN               22

// Display
#define LED_GREEN_PIN                 18
#define LED_AMBER_PIN                 19
#define LED_RED_PIN                   23
#define PATTERN_FRAME_MAX             32


// =--------------------------------------------------------------------------= Data Structures =--=

/* Examples for `stack_pattern currentUserPattern = ...`
  Static Green    - {{{false, false, true, ULONG_MAX}}, 1, ULONG_MAX, 0}
  Blink Amber     - {{{false, true, false, 500}, {false, false, false, 500}}, 2, ULONG_MAX, 0}
  Chase           - {{{true, false, false, 200}, {false, true, false, 200}, {false, false, true, 200}}, 3, ULONG_MAX, 0}
  3xBlink -> Off  - {{{false, false, true, 120}, {false, false, false, 120}}, 2, 720, 0} // preset 0 is off
*/

struct pattern_frame {
  bool red_state;
  bool amber_state;
  bool green_state;
  unsigned long delay_ms; // Use ULONG_MAX for indefinite
};

struct stack_pattern {
  pattern_frame frames[PATTERN_FRAME_MAX];
  uint8_t num_frames; // Count of frames, up to PATTERN_FRAME_MAX
  unsigned long delay_ms; // Use ULONG_MAX for indefinite
  uint8_t next_preset; // Ignored if delay_ms is ULONG_MAX
};

struct stack_state {
  uint8_t current_frame; // Array index for stack_pattern.frames
  unsigned long last_frame_at;
  unsigned long pattern_start_at;
};

struct mqtt_settings {
  char server[MQTT_SERVER_LENGTH];
  char port[MQTT_PORT_LENGTH];
  char username[MQTT_USERNAME_LENGTH];
  char password[MQTT_PASSWORD_LENGTH];
};


// =---------------------------------------------------------------------------------= Programs =--=

void setProgram(uint8_t program);
void programOff(bool init);
void programWifiConnecting(bool init);
void programWifiHotspot(bool init);
void programWifiError(bool init);
void programMqttConnecting(bool init);
void programMqttError(bool init);
void programUser(bool init);

void (*renderFunc[])(bool init) {
  programOff,
  programWifiConnecting,
  programWifiHotspot,
  programWifiError,
  programMqttConnecting,
  programMqttError,
  programUser
};
#define PROGRAM_COUNT (sizeof(renderFunc) / sizeof(renderFunc[0]))

const char *programNames[] = {
  "off",
  "wifi-connecting",
  "wifi-hotspot",
  "wifi-error",
  "mqtt-connecting",
  "mqtt-error",
  "user"
};

enum programs {
  PROGRAM_OFF,
  PROGRAM_WIFI_CONNECTING,
  PROGRAM_WIFI_HOTSPOT,
  PROGRAM_WIFI_ERROR,
  PROGRAM_MQTT_CONNECTING,
  PROGRAM_MQTT_ERROR,
  PROGRAM_USER
};


// =----------------------------------------------------------------------------------= Presets =--=

const stack_pattern presetList[] = {
  {{{false, false, false, ULONG_MAX}}, 1, ULONG_MAX, 0},
  {{{false, false, true, ULONG_MAX}}, 1, ULONG_MAX, 0},
  {{{false, false, true, 1000}, {false, false, false, 1000}}, 2, ULONG_MAX, 0},
  {{{false, true, false, ULONG_MAX}}, 1, ULONG_MAX, 0},
  {{{false, true, false, 1000}, {false, false, false, 1000}}, 2, ULONG_MAX, 0},
  {{{true, false, false, ULONG_MAX}}, 1, ULONG_MAX, 0},
  {{{true, false, false, 1000}, {false, false, false, 1000}}, 2, ULONG_MAX, 0},
  {{{true, false, false, 350}, {false, true, false, 350}, {false, false, true, 350}}, 3, ULONG_MAX, 0},
  {{{true, false, false, 200}, {false, true, false, 200}, {false, false, true, 200}, {false, true, false, 200}}, 4, ULONG_MAX, 0},
  {{{true, true, true, ULONG_MAX}}, 1, ULONG_MAX, 0}
};
#define PRESET_COUNT (sizeof(presetList) / sizeof(presetList[0]))

const char *presetNames[] = {
  "off",
  "green",
  "green-blink",
  "amber",
  "amber-blink",
  "red",
  "red-blink",
  "chase",
  "cylon",
  "all"
};

enum presets {
  PRESET_OFF,
  PRESET_GREEN,
  PRESET_GREEN_BLINK,
  PRESET_AMBER,
  PRESET_AMBER_BLINK,
  PRESET_RED,
  PRESET_RED_BLINK,
  PRESET_CHASE,
  PRESET_CYLON,
  PRESET_ALL
};


// =----------------------------------------------------------------------------------= Globals =--=

// Program
uint8_t currentProgram = 0;

// Patterns & Preset
stack_pattern currentUserPattern = presetList[PRESET_OFF];

// Save data flag for setup config
bool shouldSaveConfig = false;

// HTTP
AsyncWebServer webServer(80);
// TODO: Allow certificate to be configured in wifiManager
// Amazon Root CA
const char* rootCA = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n" \
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n" \
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n" \
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n" \
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n" \
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n" \
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n" \
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n" \
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n" \
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n" \
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n" \
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n" \
"rqXRfboQnoZsG4q5WTP468SQvvG5\n" \
"-----END CERTIFICATE-----\n";

// Client ID
char clientId[9]; // Don't forget one byte for the terminating NULL...

// OTA
char otaPassword[OTA_PASSWORD_LENGTH] = DEFAULT_OTA_PASSWORD;

// MQTT
mqtt_settings mqttSettings = {
  DEFAULT_MQTT_SERVER,
  DEFAULT_MQTT_PORT,
  DEFAULT_MQTT_USERNAME,
  DEFAULT_MQTT_PASSWORD
};
int lastReconnectAttempt = 0;

// Wifi
WiFiClientSecure wifiClient;
WiFiManager wifiManager;
WiFiManagerParameter config_mqtt_server("server", "MQTT Server", mqttSettings.server, MQTT_SERVER_LENGTH);
WiFiManagerParameter config_mqtt_port("port", "MQTT Port", mqttSettings.port, MQTT_PORT_LENGTH);
WiFiManagerParameter config_mqtt_username("username", "MQTT Username", mqttSettings.username, MQTT_USERNAME_LENGTH);
WiFiManagerParameter config_mqtt_password("password", "MQTT Password", mqttSettings.password, MQTT_PASSWORD_LENGTH);
WiFiManagerParameter config_ota_password("otaPassword", "OTA Password", otaPassword, OTA_PASSWORD_LENGTH);
bool wifiFeaturesEnabled = false;
PubSubClient mqttClient(wifiClient);

// Buttons
OneButton mainButton(MAIN_BUTTON_PIN, true);


// =--------------------------------------------------------------------------------= Utilities =--=

// The ESP32 doesn't have a Chip ID the way that the ESP8266 does, so this makes one out of the low
// bits of the wifi MAC address. High bits are are left available in case more characters are needed
void setupClientId() {
  uint64_t mac = ESP.getEfuseMac();
  // uint32_t hi = mac >> 32;
  uint32_t lo = mac;
  snprintf(clientId, 9, "%08x", lo);
}

// On ESP32 this must be run before the Wifi is started at they read from the same analog pins and
// it will both throw errors at runtime as well as mess up entropy gathering.
void setupRandom() {
  uint32_t seed;

  // Random works best with a seed that can use 31 bits analogRead on a unconnected pin tends
  // toward less than four bits
  seed = analogRead(0);
  delay(1);

  for (int shifts = 3; shifts < 31; shifts += 3) {
    seed ^= analogRead(0) << shifts;
    delay(1);
  }

  randomSeed(seed);
}


// =---------------------------------------------------------------------------= MQTT Utilities =--=

// Turn a topic suffix into a full MQTT topic within the defined namespace
String makeTopic(String suffix, bool all = false) {
  if (all) {
    return String(String(MQTT_ROOT) + "/all/" + suffix);
  }
  return String(String(MQTT_ROOT) + "/" + clientId + "/" + suffix);
}

String makeTopic(String suffix, String newClientId) {
  return String(String(MQTT_ROOT) + "/" + newClientId + "/" + suffix);
}

// Check if a given topic containes the suffix and is for this or all nodes
bool topicMatch(String topic, String suffix) {
  return topic.equals(makeTopic(suffix)) || topic.equals(makeTopic(suffix, true));
}


// =----------------------------------------------------------------------------------= Display =--=

void setLights(pattern_frame frame) {
  // Serial.printf("Light State: (%s) (%s) (%s)\n", frame.red_state ? "R" : "_", frame.amber_state ? "A" : "_", frame.green_state ? "G" : "_");
  digitalWrite(LED_RED_PIN, frame.red_state);
  digitalWrite(LED_AMBER_PIN, frame.amber_state);
  digitalWrite(LED_GREEN_PIN, frame.green_state);

  // Send current state
  StaticJsonDocument<200> messageJson;
  messageJson["red"] = frame.red_state;
  messageJson["amber"] = frame.amber_state;
  messageJson["green"] = frame.green_state;

  char message[200];
  size_t messageLength = serializeJson(messageJson, message);
  mqttClient.publish(makeTopic("state").c_str(), message, messageLength);
}

void runPattern(stack_pattern pattern, stack_state& state, bool init = false) {
  unsigned long now = millis();
  
  if (init) {
    state = {0, now, now};
    setLights(pattern.frames[0]);
    return; // Return early to avoid infinite loop from malformed pattern delay
  }

  pattern_frame frame = pattern.frames[state.current_frame];
  unsigned long frameDelay = frame.delay_ms;
  unsigned long patternDelay = pattern.delay_ms;

  // Check for whether to move to the next pattern
  if (frameDelay != ULONG_MAX && now - state.last_frame_at > frameDelay) {
    state.current_frame++;
    if (state.current_frame >= pattern.num_frames) {
      state.current_frame = 0;
    }

    state.last_frame_at = now;

    setLights(pattern.frames[state.current_frame]);
  }

  // Check whether to change pattern
  if (patternDelay != ULONG_MAX && now - state.pattern_start_at > patternDelay) {
    setPreset(isValidPresetId(pattern.next_preset) ? pattern.next_preset : PRESET_OFF);
  }
}

void programOff(bool init) {
  static stack_state state;
  const stack_pattern pattern = {{{false, false, false, ULONG_MAX}}, 1, ULONG_MAX, 0};
  runPattern(pattern, state, init);
}

void programWifiConnecting(bool init) {
  static stack_state state;
  const stack_pattern pattern = {{{true, false, false, 350}, {false, true, false, 350}, {false, false, true, 350}}, 3, ULONG_MAX, 0};
  runPattern(pattern, state, init);
}

void programWifiHotspot(bool init) {
  static stack_state state;
  const stack_pattern pattern = {{{false, true, false, 350}, {false, false, true, 350}}, 2, ULONG_MAX, 0};
  runPattern(pattern, state, init);
}

void programWifiError(bool init) {
  static stack_state state;
  const stack_pattern pattern = {{{true, true, true, 200}, {false, true, true, 200}}, 2, ULONG_MAX, 0};
  runPattern(pattern, state, init);
}

void programMqttConnecting(bool init) {
  static stack_state state;
  const stack_pattern pattern = {{{false, true, true, 500}, {false, true, false, 500}}, 2, ULONG_MAX, 0};
  runPattern(pattern, state, init);
}

void programMqttError(bool init) {
  static stack_state state;
  const stack_pattern pattern = {{{true, true, false, 200}, {false, true, false, 200}}, 2, ULONG_MAX, 0};
  runPattern(pattern, state, init);
}

void programUser(bool init) {
  static stack_state state;
  runPattern(currentUserPattern, state, init);
}


// =-------------------------------------------------------------------------------------= MQTT =--=

void setupMQTT() {
  // TODO: Get CA certs working
  // wifiClient.setCACert(rootCA);
  wifiClient.setInsecure();

  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }

  int port = atoi(mqttSettings.port);
  mqttClient.setServer(mqttSettings.server, port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(MQTT_MESSAGE_BUFFER_SIZE);
  lastReconnectAttempt = 0;
}

void loopMQTT() {
  mqttClient.loop();
}

boolean mqttConnect() {
  Serial.printf(
    "Connecting to MQTT Server: %s@%s:%s\n",
    mqttSettings.username,
    mqttSettings.server,
    mqttSettings.port
  );
  setProgram(PROGRAM_MQTT_CONNECTING);

  if (mqttClient.connect(
    clientId,
    mqttSettings.username,
    mqttSettings.password,
    makeTopic("identity").c_str(), // Last Will & Testament
    1,
    true,
    MQTT_OFFLINE_MESSAGE
  )) {
    Serial.println("MQTT Connected");
    setProgram(PROGRAM_USER);

    sendIdentity();

    // Subscribe to topics
    mqttClient.subscribe(makeTopic("identify", true).c_str());
    mqttClient.subscribe(makeTopic("preset").c_str());
    mqttClient.subscribe(makeTopic("preset", true).c_str());
    mqttClient.subscribe(makeTopic("definition").c_str());
    mqttClient.subscribe(makeTopic("definition", true).c_str());
  } else {
    Serial.print("MQTT Connection Failure, rc=");
    Serial.println(mqttClient.state());
    setProgram(PROGRAM_MQTT_ERROR);
  }

  return mqttClient.connected();
}

void mqttReconnect() {
  unsigned long now = millis();

  if (now - lastReconnectAttempt > MQTT_RECONNECT_DELAY) {
    lastReconnectAttempt = now;
    if (mqttConnect()) {
      lastReconnectAttempt = 0;
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("] ");
  payload[length] = 0;
  String message((char *)payload);
  Serial.println(message);

  if (topicMatch(topic, "identify")) {
    Serial.println("COMMAND: Send Identity");
    sendIdentity();
  } else if (topicMatch(topic, "preset")) {
    Serial.println("COMMAND: Set Preset");
    setPreset(message);
  } else if (topicMatch(topic, "definition")) {
    Serial.println("COMMAND: Set Definition");
    setDefinition(message);
  }
}

void sendIdentity() {
  mqttClient.publish(
    makeTopic("identity").c_str(),
    MQTT_ONLINE_MESSAGE,
    true
  );
}


// =----------------------------------------------------------------------------------= Buttons =--=

void buttonClick() {
  Serial.println("Button Click");
}

void buttonLongPress() {
  Serial.println("Button Long Press Start");
  finalizeWifi(wifiManager.startConfigPortal(captivePortalWifiName().c_str(), SETUP_AP_PASSWORD));
}

void setupButton() {
  pinMode(MAIN_BUTTON_PIN, INPUT_PULLUP);

  mainButton.attachClick(buttonClick);
  mainButton.attachLongPressStart(buttonLongPress);
}

void loopButton() {
  mainButton.tick();
}


// =-------------------------------------------------------------------------------------= WIFI =--=

// Finishing steps for wifi
void finalizeWifi(boolean connectStatus) {
  if (connectStatus) {
    wifiFeaturesEnabled = true;

    if (WiFi.status() == WL_CONNECTED) {
      setupMdns();
    }
  } else {
    wifiFeaturesEnabled = false;
    Serial.println("Failed to connect to wifi and hit timeout.");
    setProgram(PROGRAM_WIFI_ERROR);
  }
}

// Gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager) {
  setProgram(PROGRAM_WIFI_HOTSPOT);

  Serial.print("Entered config mode... ");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

// Callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Saving config values");

  // Read updated parameters
  strcpy(mqttSettings.server, config_mqtt_server.getValue());
  strcpy(mqttSettings.port, config_mqtt_port.getValue());
  strcpy(mqttSettings.username, config_mqtt_username.getValue());
  strcpy(mqttSettings.password, config_mqtt_password.getValue());
  strcpy(otaPassword, config_ota_password.getValue());

  Serial.println("Saving config...");
  DynamicJsonDocument jsonDocument(1000);
  jsonDocument["mqtt_server"] = mqttSettings.server;
  jsonDocument["mqtt_port"] = mqttSettings.port;
  jsonDocument["mqtt_username"] = mqttSettings.username;
  jsonDocument["mqtt_password"] = mqttSettings.password;
  jsonDocument["ota_password"] = otaPassword;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
  }

  serializeJson(jsonDocument, configFile);
  configFile.close();

  // Potentially new values, update MQTT
  setupMQTT();
}

String captivePortalWifiName() {
  return String((String(SETUP_AP_NAME) + " - " + clientId));
}

void setupWifi() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  wifiManager.addParameter(&config_mqtt_server);
  wifiManager.addParameter(&config_mqtt_port);
  wifiManager.addParameter(&config_mqtt_username);
  wifiManager.addParameter(&config_mqtt_password);
  wifiManager.addParameter(&config_ota_password);

  wifiManager.setConfigPortalTimeout(WIFI_HOTSPOT_TIMEOUT);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setDarkMode(true);

  setProgram(PROGRAM_WIFI_CONNECTING);
  finalizeWifi(wifiManager.autoConnect(captivePortalWifiName().c_str(), SETUP_AP_PASSWORD));
}

void loopWifi() {
  wifiManager.process();
}

void setupMdns() {
  if(!MDNS.begin(String((String(MDNS_PREFIX) + clientId)).c_str())) {
    Serial.println("Error starting mDNS");
    return;
  }

  MDNS.addService("http", "tcp", 80);
  MDNS.addService(MDNS_SERVICE_NAME, "tcp", 80);
}

// =------------------------------------------------------------------------------= File System =--=

void setupFileSystem() {
  // Clean FS, for testing
  // SPIFFS.format();

  // Read configuration from FS json
  Serial.println("Mounting FS...");

  if (SPIFFS.begin(true)) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      // file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open("/config.json", "r");

      if (configFile) {
        Serial.println("Opened config file");

        DynamicJsonDocument settingsDocument(1000);
        DeserializationError error = deserializeJson(settingsDocument, configFile);

        if (!error) {
          Serial.println("\nParsed json");

          strcpy(mqttSettings.server, settingsDocument["mqtt_server"]);
          strcpy(mqttSettings.port, settingsDocument["mqtt_port"]);
          strcpy(mqttSettings.username, settingsDocument["mqtt_username"]);
          strcpy(mqttSettings.password, settingsDocument["mqtt_password"]);
          strcpy(otaPassword, settingsDocument["ota_password"]);
        } else {
          Serial.println("Failed to load json config");
        }
      }
    }
  } else {
    Serial.println("Failed to mount FS");
  }
}


// =----------------------------------------------------------------------------------= Display =--=

void setupDisplay() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_AMBER_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  digitalWrite(LED_RED_PIN, 0);
  digitalWrite(LED_AMBER_PIN, 0);
  digitalWrite(LED_GREEN_PIN, 0);
}

void loopDisplay(bool init = false) {
  (*renderFunc[currentProgram])(init);
}

bool isValidProgramId(uint8_t program) {
  return program >= 0 && program < PROGRAM_COUNT;
}

void setProgram(uint8_t program) {
  if (isValidProgramId(program) && program != currentProgram) {
    currentProgram = program;
    Serial.printf("Setting program to %s\n", programNames[program]);
    loopDisplay(true);

    // Current program name
    mqttClient.publish(makeTopic("program").c_str(), programNames[program]);
  }
}

void setProgram(String programName) {
  for (uint8_t program = 0; program < PROGRAM_COUNT; program++) {
    if (programName.equals(programNames[program])) {
      setProgram(program);
      break;
    }
  }
}

void setPattern(stack_pattern pattern) {
  currentUserPattern = pattern;
  loopDisplay(true);

  // Encode and send current pattern
  DynamicJsonDocument messageJson(MQTT_MESSAGE_BUFFER_SIZE);
  JsonArray frames = messageJson.createNestedArray("frames");
  for (size_t index = 0; index < pattern.num_frames; index++) {
    JsonObject frame = frames.createNestedObject();
    frame["red"] = pattern.frames[index].red_state;
    frame["amber"] = pattern.frames[index].amber_state;
    frame["green"] = pattern.frames[index].green_state;
    if (pattern.frames[index].delay_ms != ULONG_MAX) {
      frame["delay"] = pattern.frames[index].delay_ms;
    }
  }
  
  if (pattern.delay_ms != ULONG_MAX) {
    messageJson["delay"] = pattern.delay_ms;
    messageJson["next_preset"] = presetNames[pattern.next_preset];
  }
  
  char message[MQTT_MESSAGE_BUFFER_SIZE];
  size_t messageLength = serializeJson(messageJson, message);
  mqttClient.publish(makeTopic("pattern").c_str(), message, messageLength);
}

bool isValidPresetId(uint8_t preset) {
  return preset >= 0 && preset < PRESET_COUNT;
}

int getPresetId(const char *presetName) {
  for (uint8_t preset = 0; preset < PRESET_COUNT; preset++) {
    if (strcmp(presetName, presetNames[preset]) == 0) {
      return preset;
    }
  }
  return -1;
}

void setPreset(uint8_t preset) {
  if (isValidPresetId(preset)) {
    Serial.printf("Setting preset to %s\n", presetNames[preset]);
    setPattern(presetList[preset]);
  }
}

void setPreset(String presetName) {
  int preset = getPresetId(presetName.c_str());
  if (preset >= 0) setPreset(preset);
}

void setDefinition(String definition) {
  Serial.println("New Definition:");
  Serial.println(definition);

  DynamicJsonDocument doc(MQTT_MESSAGE_BUFFER_SIZE);
  DeserializationError error = deserializeJson(doc, definition);

  if (error) {
    Serial.print("Error deserializing JSON: ");
    Serial.println(error.c_str());
    return;
  }

  stack_pattern pattern;

  if (!doc.containsKey("frames") || doc["frames"].as<JsonArray>().size() == 0) {
    Serial.println("Frames definition missing or empty! Cancelling.");
    return;
  }
  
  uint8_t index = 0;
  for (JsonObject frame : doc["frames"].as<JsonArray>()) {
    pattern.frames[index].red_state = frame["red"];
    pattern.frames[index].amber_state = frame["amber"];
    pattern.frames[index].green_state = frame["green"];

    if (!frame.containsKey("delay") || frame["delay"] == -1) {
      pattern.frames[index].delay_ms = ULONG_MAX;  
    } else {
      pattern.frames[index].delay_ms = frame["delay"];
    }
    index++;

    if (index > PATTERN_FRAME_MAX) {
      Serial.println("Too many pattern frames! Cancelling.");
      return;
    }
  }

  pattern.num_frames = index;
  if (!doc.containsKey("delay") || doc["delay"] == -1) {
    pattern.delay_ms = ULONG_MAX;
    pattern.next_preset = 0;
  } else {
    pattern.delay_ms = doc["delay"];

    if (!doc.containsKey("next_preset")) {
      Serial.println("Missing next preset! Cancelling.");
      return;
    }

    int preset = getPresetId(doc["next_preset"]);
    if (preset < 0) {
      Serial.println("Unknown preset! Cancelling.");
      return;
    }
    pattern.next_preset = preset;
  }

  setPattern(pattern);
}


/*=-------------------------------------------------------------------------------------= OTA =--=*/

void setupOTA() {
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Got to /update for firmware updates.");
  });
  AsyncElegantOTA.begin(&webServer, OTA_USERNAME, otaPassword);
  webServer.begin();
  Serial.println("HTTP server started.");
}


/*=----------------------------------------------------------------------------= Main Runtime =--=*/

void setup() {
  Serial.begin(115200);

  setupClientId();
  setupRandom();
  setupFileSystem();
  setupButton();
  setupDisplay();
  setupWifi();
  setupOTA();
  setupMQTT();
}

void loop() {
  loopButton();
  loopDisplay();
  loopWifi();

  if (wifiFeaturesEnabled) {
    if (mqttClient.connected()) {
      loopMQTT();
    } else {
      mqttReconnect();
    }
  }
}
