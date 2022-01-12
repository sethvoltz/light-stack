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
(e.g. wifi, connections, errors, etc) and the actual user-driven state from the network. To ease
usage of the light stack, there is also a concept of presets. For the user-driven program, a second
set of parameters is able to be provided to declare which of the lights are enabled, whether they
are animating (e.g. blinking), if there is a count or timeout, and if they go to another preset
after a given time. The presets are just hard coded definitions of these actions, which are loaded
in at the time the user sets them, overriding any active set, and the user may also pass a full
definition, instead of calling on a preset.

=-----------------------------------------------------------------------------------------------= */

// =--------------------------------------------------------------------------------= Libraries =--=

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

// Buttons
#define MAIN_BUTTON_PIN               26

// Display
#define LED_GREEN_PIN                 18
#define LED_AMBER_PIN                 19
#define LED_RED_PIN                   23


// =----------------------------------------------------------------------------------= Globals =--=

// Program
uint8_t currentProgram = 0;

// Save data flag for setup config
bool shouldSaveConfig = false;

// HTTP
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
char mqtt_server[MQTT_SERVER_LENGTH] = DEFAULT_MQTT_SERVER;
char mqtt_port[MQTT_PORT_LENGTH] = DEFAULT_MQTT_PORT;
char mqtt_username[MQTT_USERNAME_LENGTH] = DEFAULT_MQTT_USERNAME;
char mqtt_password[MQTT_PASSWORD_LENGTH] = DEFAULT_MQTT_PASSWORD;
int connectionAttempts = 0;

// Wifi
WiFiClientSecure wifiClient;
WiFiManager wifiManager;
WiFiManagerParameter config_mqtt_server("server", "MQTT Server", mqtt_server, MQTT_SERVER_LENGTH);
WiFiManagerParameter config_mqtt_port("port", "MQTT Port", mqtt_port, MQTT_PORT_LENGTH);
WiFiManagerParameter config_mqtt_username("username", "MQTT Username", mqtt_username, MQTT_USERNAME_LENGTH);
WiFiManagerParameter config_mqtt_password("password", "MQTT Password", mqtt_password, MQTT_PASSWORD_LENGTH);
bool wifiFeaturesEnabled = false;
PubSubClient mqttClient(wifiClient);

// Buttons
OneButton mainButton(MAIN_BUTTON_PIN, true);


// =---------------------------------------------------------------------------------= Programs =--=

void setProgram(uint8_t program);
void programOff(bool first);
void programWifiConnecting(bool first);
void programWifiHotspot(bool first);
void programWifiError(bool first);
void programMqttConnecting(bool first);
void programMqttError(bool first);
void programUser(bool first);

void (*renderFunc[])(bool first) {
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

// TODO: Figure out data structure for the definitions - enabled LEDs, animation, timing, next definition or program, etc
#define PRESET_COUNT 8 // TODO: Replace this with auto-calc
const char *presetNames[] = {
  "off",
  "green",
  "green-blink",
  "amber",
  "amber-blink",
  "red",
  "red-blink",
  "all"
};


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

void programOff(bool first) {
  // TODO: Turn all LEDs off and hold
}

void programWifiConnecting(bool first) {
  // TODO: Sequence LEDs green, amber, red, loop
}

void programWifiHotspot(bool first) {
  // TODO: Sequence LEDs green, amber, loop
}

void programWifiError(bool first) {
  // TODO: Green and amber on, blink red
}

void programMqttConnecting(bool first) {
  // TODO: Amber on, blink green
}

void programMqttError(bool first) {
  // TODO: Amber on, blink red
}

void programUser(bool first) {
  // TODO: Run definition program
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
  // wifiClient.setCACert(rootCA);
  wifiClient.setInsecure();

  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }

  int port = atoi(mqtt_port);
  mqttClient.setServer(mqtt_server, port);
  mqttClient.setCallback(mqttCallback);
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
    Serial.printf("MQTT Server: %s@%s:%s\n", mqtt_username, mqtt_server, mqtt_port);
    Serial.print("Attempting MQTT connection... ");
    setProgram(PROGRAM_MQTT_CONNECTING);

    // Attempt to connect
    if (mqttClient.connect(client_id, mqtt_username, mqtt_password)) {
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

// Finishing steps for after wifi may be complete
void finalizeWifi() {
  if (WiFi.status() != WL_CONNECTED){
    wifiFeaturesEnabled = false;
    Serial.print("Failed to connect to wifi. ");
    setProgram(PROGRAM_WIFI_ERROR);
  } else {
    wifiFeaturesEnabled = true;
  }
}

// gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager) {
  setProgram(PROGRAM_WIFI_HOTSPOT);

  Serial.print("Entered config mode... ");
  Serial.println(WiFi.softAPIP());

  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

// Callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Saving config values");

  // Read updated parameters
  strcpy(mqtt_server, config_mqtt_server.getValue());
  strcpy(mqtt_port, config_mqtt_port.getValue());
  strcpy(mqtt_username, config_mqtt_username.getValue());
  strcpy(mqtt_password, config_mqtt_password.getValue());

  Serial.println("Saving config...");
  DynamicJsonDocument jsonDocument(1000);
  jsonDocument["mqtt_server"] = mqtt_server;
  jsonDocument["mqtt_port"] = mqtt_port;
  jsonDocument["mqtt_username"] = mqtt_username;
  jsonDocument["mqtt_password"] = mqtt_password;

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

          strcpy(mqtt_server, settingsDocument["mqtt_server"]);
          strcpy(mqtt_port, settingsDocument["mqtt_port"]);
          strcpy(mqtt_username, settingsDocument["mqtt_username"]);
          strcpy(mqtt_password, settingsDocument["mqtt_password"]);
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
  // TODO: Init pins for LEDs and set output
}

void loopDisplay(bool first = false) {
  (*renderFunc[currentProgram])(first);
}

void setProgram(uint8_t program) {
  if (program >= 0 && program < PROGRAM_COUNT && program != currentProgram) {
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

void setPreset(uint8_t preset) {
  if (preset >= 0 && preset < PRESET_COUNT) {
    Serial.printf("Setting preset to %s\n", presetNames[preset]);
    // TODO: Replace current definition with the preset's value
  }
}

void setPreset(String presetName) {
  for (uint8_t preset = 0; preset < PROGRAM_COUNT; preset++) {
    if (presetName.equals(presetNames[preset])) {
      setProgram(preset);
      break;
    }
  }
}

void setDefinition(String definition) {
  Serial.println("New Definition:");
  Serial.println(definition);
  // TODO: Parse definition as JSON, validate required format and set definition values
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
