/* =------------------------------------------------------------------------------= Description =--=

The Light Stack firmware is meant to drive a simple industrial-style light stack with three lights,
red, orange/amber, and green and expose those controls over MQTT.

The controls are exposed in two ways: preset programs to set all the lights to the correct state or
animation, and a manual control that sets the program to "custom" and allows for the lights to be
set explicitely. Also exposed are fetch commands to publish the current state of the light stack.

The firmware, being designed for the ESP32 chipset, also exposes wifi hotspot setup and
configuration for MQTT server, port and credentials.

The display runs on the concept of a program, which is a hard coded runtime of actions for the LED
stack. Within this, there are programs meant to control what is going on with the system itself
(e.g. wifi, connections, errors, etc) and the actual user-driven pattern from the network. To ease
usage of the light stack, there are also a set of preset patterns. For the user-driven program, a
second set of parameters is able to be provided to declare a sequence of light states with delays,
and if they go to another preset after a given time. The presets are just hard coded definitions of
these actions, which are loaded in at the time the user sets them, overriding any active set, and
the user may also pass a full definition, instead of calling on a preset.

=-----------------------------------------------------------------------------------------------= */

// =--------------------------------------------------------------------------------= Libraries =--=

#include <limits.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <OneButton.h>


// =----------------------------------------------------------------------------= Configuration =--=

// Wifi
#define SETUP_AP_NAME                 "Setup Light Stack"
#define SETUP_AP_PASSWORD             "setuplightstack"
#define WIFI_HOTSPOT_TIMEOUT          180 // Seconds before hotspot ends and attempts reconnect

// MQTT
#define MAX_CONNECTION_ATTEMPTS       3 // Number of attempts before fallback
#define SHORT_CONNECTION_DELAY        3000 // ms Delay between initial connection attempts
#define LONG_CONNECTION_DELAY         120000 // ms Delay between attempts after max attempts
#define CONNECTING_BLINK_DELAY        500
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

// Buttons
#define MAIN_BUTTON_PIN               26

// Display
#define LED_GREEN_PIN                 18
#define LED_AMBER_PIN                 19
#define LED_RED_PIN                   23
#define PATTERN_ELEMENT_MAX           32


// =--------------------------------------------------------------------------= Data Structures =--=

/* Examples for `stack_pattern currentUserPattern = ...`
  Static Green    - {{{false, false, true, ULONG_MAX}}, 1, ULONG_MAX, 0}
  Blink Amber     - {{{false, true, false, 500}, {false, false, false, 500}}, 2, ULONG_MAX, 0}
  Chase           - {{{true, false, false, 200}, {false, true, false, 200}, {false, false, true, 200}}, 3, ULONG_MAX, 0}
  3xBlink -> Off  - {{{false, false, true, 120}, {false, false, false, 120}}, 2, 720, 0} // preset 0 is off
*/

struct pattern_element {
  bool red_state;
  bool amber_state;
  bool green_state;
  unsigned long delay_ms; // Use ULONG_MAX for indefinite
};

struct stack_pattern {
  pattern_element elements[PATTERN_ELEMENT_MAX];
  uint8_t num_elements; // Count of elements, up to PATTERN_ELEMENT_MAX
  unsigned long delay_ms; // Use ULONG_MAX for indefinite
  uint8_t next_preset; // Ignored if delay_ms is ULONG_MAX
};

struct stack_state {
  uint8_t current_element; // Array index for stack_pattern.elements
  unsigned long last_element_at;
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
char client_id[13]; // Don't forget one byte for the terminating NULL...

// MQTT
mqtt_settings mqttSettings = {
  DEFAULT_MQTT_SERVER,
  DEFAULT_MQTT_PORT,
  DEFAULT_MQTT_USERNAME,
  DEFAULT_MQTT_PASSWORD
};
int connectionAttempts = 0;

// Wifi
WiFiClientSecure wifiClient;
WiFiManager wifiManager;
WiFiManagerParameter config_mqtt_server("server", "MQTT Server", mqttSettings.server, MQTT_SERVER_LENGTH);
WiFiManagerParameter config_mqtt_port("port", "MQTT Port", mqttSettings.port, MQTT_PORT_LENGTH);
WiFiManagerParameter config_mqtt_username("username", "MQTT Username", mqttSettings.username, MQTT_USERNAME_LENGTH);
WiFiManagerParameter config_mqtt_password("password", "MQTT Password", mqttSettings.password, MQTT_PASSWORD_LENGTH);
bool wifiFeaturesEnabled = false;
PubSubClient mqttClient(wifiClient);

// Buttons
OneButton mainButton(MAIN_BUTTON_PIN, true);


// =--------------------------------------------------------------------------------= Utilities =--=

void setupClientId() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t hi = mac >> 32;
  uint32_t lo = mac;
  snprintf(client_id, 13, "%04x%08x", hi, lo);
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


// =----------------------------------------------------------------------------------= Display =--=

void setLights(pattern_element element) {
  Serial.printf("Light State: (%s) (%s) (%s)\n", element.red_state ? "R" : "_", element.amber_state ? "A" : "_", element.green_state ? "G" : "_");
  digitalWrite(LED_RED_PIN, element.red_state);
  digitalWrite(LED_AMBER_PIN, element.amber_state);
  digitalWrite(LED_GREEN_PIN, element.green_state);
}

void runPattern(stack_pattern pattern, stack_state& state, bool init = false) {
  unsigned long now = millis();
  
  if (init) {
    state = {0, now, now};
    setLights(pattern.elements[0]);
    return; // Return early to avoid infinite loop from malformed pattern delay
  }

  pattern_element element = pattern.elements[state.current_element];
  unsigned long elementDelay = element.delay_ms;
  unsigned long patternDelay = pattern.delay_ms;

  // Check for whether to move to the next pattern
  if (elementDelay != ULONG_MAX && now - state.last_element_at > elementDelay) {
    state.current_element++;
    if (state.current_element >= pattern.num_elements) {
      state.current_element = 0;
    }

    state.last_element_at = now;

    setLights(pattern.elements[state.current_element]);
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


// =---------------------------------------------------------------------------= MQTT Utilities =--=

// Turn a topic suffix into a full MQTT topic for the centerpiece namespace
String makeTopic(String suffix, bool all = false) {
  if (all) {
    return String(String(MQTT_ROOT) + "/all/" + suffix);
  }
  return String(String(MQTT_ROOT) + "/" + client_id + "/" + suffix);
}

String makeTopic(String suffix, String newClientId) {
  return String(String(MQTT_ROOT) + "/" + newClientId + "/" + suffix);
}

// Check if a given topic containes the suffix and is for this or all nodes
bool topicMatch(String topic, String suffix) {
  return topic.equals(makeTopic(suffix)) || topic.equals(makeTopic(suffix, true));
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
  connectionAttempts = 0;
}

void loopMQTT() {
  mqttClient.loop();
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void mqttConnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.printf("MQTT Server: %s@%s:%s\n", mqttSettings.username, mqttSettings.server, mqttSettings.port);
    Serial.print("Attempting MQTT connection... ");
    setProgram(PROGRAM_MQTT_CONNECTING);

    // Attempt to connect
    if (mqttClient.connect(client_id, mqttSettings.username, mqttSettings.password)) {
      Serial.println("connected");
      setProgram(PROGRAM_USER);

      sendIdentity();

      // Subscribe to topics
      mqttClient.subscribe(makeTopic("identify", true).c_str());
      mqttClient.subscribe(makeTopic("preset").c_str());
      mqttClient.subscribe(makeTopic("preset", true).c_str());
      mqttClient.subscribe(makeTopic("definition").c_str());
      mqttClient.subscribe(makeTopic("definition", true).c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 3 seconds");

      // TODO: This should really be non-blocking
      // https://github.com/knolleary/pubsubclient/tree/master/examples/mqtt_reconnect_nonblocking
      // Wait a few seconds before retrying
      delay(3000);
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
  mqttClient.publish(makeTopic("identity").c_str(), "online");
}


// =----------------------------------------------------------------------------------= Buttons =--=

void buttonClick() {
  Serial.println("Button Click");
}

void buttonLongPress() {
  Serial.println("Button Long Press Start");
  String setupAPName(String(SETUP_AP_NAME) + " " + client_id);
  wifiManager.startConfigPortal(setupAPName.c_str(), SETUP_AP_PASSWORD);
  finalizeWifi();
}

void setupButton() {
  pinMode(MAIN_BUTTON_PIN, INPUT_PULLUP);

  mainButton.attachClick(buttonClick);
  // mainButton.attachLongPressStart(buttonLongPress);
}

void loopButton() {
  mainButton.tick();
}


// =-------------------------------------------------------------------------------------= WIFI =--=

// Finishing steps for wifi
void finalizeWifi() {
  if (WiFi.status() != WL_CONNECTED){
    wifiFeaturesEnabled = false;
    Serial.print("Failed to connect to wifi. ");
    setProgram(PROGRAM_WIFI_ERROR);
  } else {
    wifiFeaturesEnabled = true;
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

  Serial.println("Saving config...");
  DynamicJsonDocument jsonDocument(1000);
  jsonDocument["mqtt_server"] = mqttSettings.server;
  jsonDocument["mqtt_port"] = mqttSettings.port;
  jsonDocument["mqtt_username"] = mqttSettings.username;
  jsonDocument["mqtt_password"] = mqttSettings.password;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
  }

  serializeJson(jsonDocument, configFile);
  configFile.close();

  // Potentially new values, update MQTT
  setupMQTT();
}

void setupWifi() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  wifiManager.addParameter(&config_mqtt_server);
  wifiManager.addParameter(&config_mqtt_port);
  wifiManager.addParameter(&config_mqtt_username);
  wifiManager.addParameter(&config_mqtt_password);

  wifiManager.setConfigPortalTimeout(WIFI_HOTSPOT_TIMEOUT);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  String setupAPName(String(SETUP_AP_NAME) + " " + client_id);
  if (wifiManager.autoConnect(setupAPName.c_str(), SETUP_AP_PASSWORD)) {
    finalizeWifi();
  } else {
    Serial.println("Unable to connect. Womp womp");
  }
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
    currentUserPattern = presetList[preset];
    loopDisplay(true);
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

  uint8_t index = 0;
  for (JsonObject element : doc["elements"].as<JsonArray>()) {
    pattern.elements[index].red_state = element["red"];
    pattern.elements[index].amber_state = element["amber"];
    pattern.elements[index].green_state = element["green"];
    pattern.elements[index].delay_ms = element["delay"] == -1 ? ULONG_MAX : element["delay"];
    index++;

    if (index > PATTERN_ELEMENT_MAX) {
      Serial.println("Too many pattern elements! Cancelling.");
      return;
    }
  }

  pattern.num_elements = index;
  if (doc["delay"] == -1) {
    pattern.delay_ms = ULONG_MAX;
    pattern.next_preset = 0;
  } else {
    pattern.delay_ms = doc["delay"];
    int preset = getPresetId(doc["next_preset"]);
    if (preset < 0) {
      Serial.println("Unknown preset! Cancelling.");
      return;
    }
    pattern.next_preset = preset;
  }

  currentUserPattern = pattern;
  loopDisplay(true);
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
  setupMQTT();
}

void loop() {
  wifiManager.process();

  if (wifiFeaturesEnabled) {
    if (mqttClient.connected()) {
      loopMQTT();
    } else {
      mqttConnect();
    }
  } else {
    // maybeConnectWifi();
  }

  loopButton();
  loopDisplay();
}
