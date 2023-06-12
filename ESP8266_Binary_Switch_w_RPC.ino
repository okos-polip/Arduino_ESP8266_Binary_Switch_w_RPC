/**
 * @file binarySwitch_w_RPC.ino
 * @author Curt Henrichs
 * @brief Binary Switch Example
 * @version 0.1
 * @date 2022-10-20
 * @copyright Copyright (c) 2022
 * 
 * Demonstrates core polip-lib behavior using a binary switch type.
 * This could directly control an AC power outlet for instance.
 * 
 * Dependency installation:
 * ---
 * - Arduino_JSON (via Arduino IDE)
 * - ESP8266 Board (via Arduino IDE)
 * - WiFiManager (via Arduino IDE)
 * - NTPClient (via Arduino IDE)
 * - Arduino Crypto (via Arduino IDE)
 *      - (I had to manually rename the library from Crypto.h to ArduinoCrypto.h)
 * 
 * Debug Serial:
 * ---
 * Several commands available for testing behavior
 * 
 *  [reset] -> Requests hardware state reset (will wipe EEPROM)
 *  [state?] -> Queries current state code
 *  [toggle] -> Flips current state boolean (power state)
 *  [error?] -> Queries error status of polip lib
 */

//==============================================================================
//  Libraries
//==============================================================================

#include <Arduino.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <polip-client.h>
#include <ESP8266HTTPClient.h>

//==============================================================================
//  Preprocessor Constants
//==============================================================================

#define SWITCH_PIN                      (LED_BUILTIN)
#define RESET_BTN_PIN                   (D0)

#define RESET_BTN_TIME_THRESHOLD        (200L)

#define NTP_URL                         "pool.ntp.org"

#define DEBUG_SERIAL_BAUD               (115200)

#define FALLBACK_AP_NAME                "AP-Polip-Device-Setup"

//==============================================================================
//  Preprocessor Macros
//==============================================================================

#define readResetBtnState() ((bool)digitalRead(RESET_BTN_PIN))
#define setSwitchState(state) (digitalWrite(SWITCH_PIN, !(bool)(state))) //Active LOW

//==============================================================================
//  Declared Constants
//==============================================================================

const char* SERIAL_STR = "binary-switch-0-0000";
const char* KEY_STR = "revocable-key-0";    //NOTE: Should be configurable
const char* HARDWARE_STR = POLIP_VERSION_STD_FORMAT(0,0,1);
const char* FIRMWARE_STR = POLIP_VERSION_STD_FORMAT(0,0,1);

//==============================================================================
//  Private Module Variables
//==============================================================================

static StaticJsonDocument<POLIP_MIN_RECOMMENDED_DOC_SIZE> _doc;
static WiFiUDP _ntpUDP;
static NTPClient _timeClient(_ntpUDP, NTP_URL, 0);
static WiFiManager _wifiManager;
static polip_device_t _polipDevice;
static polip_workflow_t _polipWorkflow;

static unsigned long _resetTime;
static bool _flag_reset = false;
static bool _prevBtnState = false;
static char _rxBuffer[50];
static int _rxBufferIdx = 0;
static bool _currentState = false;
static unsigned long _pollTime;

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc);
static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc);
static void _errorHandler(polip_device_t* dev, JsonDocument& doc, polip_workflow_source_t source);

//==============================================================================
//  MAIN
//==============================================================================

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.flush();

    pinMode(SWITCH_PIN, OUTPUT);
    pinMode(RESET_BTN_PIN, INPUT);
    setSwitchState(_currentState);

    _wifiManager.autoConnect(FALLBACK_AP_NAME);

    _timeClient.begin();

    POLIP_BLOCK_AWAIT_SERVER_OK();

    _polipDevice.serialStr = SERIAL_STR;
    _polipDevice.keyStr = (const uint8_t*)KEY_STR;
    _polipDevice.keyStrLen = strlen(KEY_STR);
    _polipDevice.hardwareStr = HARDWARE_STR;
    _polipDevice.firmwareStr = FIRMWARE_STR;
    _polipDevice.skipTagCheck = false;

    _polipWorkflow.device = &_polipDevice;
    _polipWorkflow.hooks.pushStateSetupCb = _pushStateSetup;
    _polipWorkflow.hooks.pollStateRespCb = _pollStateResponse;
    _polipWorkflow.hooks.workflowErrorCb = _errorHandler;

    unsigned long currentTime = millis();
    polip_workflow_initialize(&_polipWorkflow, currentTime);
    _pollTime = _resetTime = currentTime;
    _flag_reset = false;
    _prevBtnState = false;
}

void loop() {
    // We do most things against soft timers in the main loop
    unsigned long currentTime = millis();

    // Refresh time
    _timeClient.update();

    // Update Polip Server
    polip_workflow_periodic_update(&_polipWorkflow, _doc, _timeClient.getFormattedTime().c_str(), currentTime);

    // Serial debugging interface provides full state control
    while (Serial.available() > 0) {
        if (_rxBufferIdx >= (sizeof(_rxBuffer) - 1)) {
            Serial.println(F("Error - Buffer Overflow ~ Clearing"));
            _rxBufferIdx = 0;
        }

        char c = Serial.read();
        if (c != '\n') {
            _rxBuffer[_rxBufferIdx] = c;
            _rxBufferIdx++;
        } else {
            _rxBuffer[_rxBufferIdx] = '\0';
            _rxBufferIdx = 0;

            String str = String(_rxBuffer);

            if (str == "reset") {
                Serial.println(F("Debug Reset Requested"));
                _flag_reset = true;
            } else if (str == "state?") {
                Serial.print(F("STATE = "));
                Serial.println(_currentState); 
            } else if (str == "toggle") {
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
                _currentState = !_currentState;
            } else if (str == "error?") {
                if (POLIP_WORKFLOW_IN_ERROR(&_polipWorkflow)) {
                    Serial.println(F("Error in PolipLib: "));
                    Serial.println((int)_polipWorkflow.flags.error);
                } else {
                    Serial.println(F("No Error"));
                }
                POLIP_WORKFLOW_ACK_ERROR(&_polipWorkflow);
            } else {
                Serial.print(F("Error - Invalid Command ~ `"));
                Serial.print(str);
                Serial.println(F("`"));
            }
        }
    }

    // Handle Reset button
    bool btnState = readResetBtnState();
    if (!btnState && _prevBtnState) {
        _resetTime = currentTime;
    } else if (btnState && !_prevBtnState) {
        if ((currentTime - _resetTime) >= RESET_BTN_TIME_THRESHOLD) {
            Serial.println(F("Reset Button Held"));
            _flag_reset = true;
        }
        // Otherwise button press was debounced.
    }
    _prevBtnState = btnState;

     // Reset State Machine
    if (_flag_reset) {
        _flag_reset = false;
        Serial.println(F("Erasing Config, restarting"));
        _wifiManager.resetSettings();
        ESP.restart();
    }

    // Update physical state
    setSwitchState(_currentState);
    delay(1);
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc.createNestedObject("state");
    stateObj["power"] = _currentState;
}

static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc["state"];
    _currentState = stateObj["power"];
}

static void _errorHandler(polip_device_t* dev, JsonDocument& doc, polip_workflow_source_t source) {
    Serial.print(F("Error Handler ~ polip server error during OP="));
    Serial.print((int)source);
    Serial.print(F(" with CODE="));
    Serial.println((int)_polipWorkflow.flags.error);
} 