/** 
 * A sketch written for the ESP32 Dev Module that replicates the behaviors 
 * and proprietary UART output signals of an existing 7-button controller. 
 * The existing controller is connected to a black box that drives 
 * motors that move and position a car seat. 
 *
 * Version 1.2 adds preliminary bluetooth support. Note that builds that
 * include bluetooth consumes 86% of program storage space on an ESP32.
 * This compares to about 24% without bluetooth support. The included library
 * (BluetoothSerial.h) appears to be responsible for almost all of this 
 * additional proram space consumption.
 * 
 * Author: Philip Giacalone
 * Date: July 5, 2024
 * Version: 1.2
 * Description: Added bluetooth support (sufficient for initial testing)
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "LEDControl.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Uncomment the following line to run automated tests
//#define TESTING_PUBLIC_INTERFACE

// It is not known whether or not power-up signal pulses are required. 
// So this boolean flag (SEND_POWER_UP_PULSES) is provided to easily 
// turn them ON or OFF. 
// Note: Turning these pulses ON increases startup time by ~7 seconds.
bool SEND_POWER_UP_PULSES = false;

// Debug levels
constexpr int DEBUG_PRIORITY_HIGH = 3;
constexpr int DEBUG_PRIORITY_MEDIUM = 2;
constexpr int DEBUG_PRIORITY_LOW = 1;
constexpr int DEBUG_PRIORITY_NONE = 0;
// Debug messages are printed to the console when a debug's priority is >= debugSetting
int debugSetting = DEBUG_PRIORITY_HIGH;

// GPIO pins for all 7 buttons (Controller buttons #0 thru #6)
constexpr int buttonPins[] = {32, 33, 21, 26, 18, 27, 4};
const unsigned int NUMBER_OF_BUTTONS = sizeof(buttonPins) / sizeof(buttonPins[0]);

// The letter commands sent to the Adapt Systems black box by each of the buttons (for on-design operations)
const char commands[] = {'G', 'A', 'D', 'E', 'B', 'F', 'C'};

// Custom UART pins (avoiding the Serial Tx/Rx pins used for console output)
constexpr int uartTxPin = 17;
constexpr int uartRxPin = 16;   //NOTE: the Rx pin is NOT used, since we're assuming the legacy controller only sends signals.
constexpr int uartEnabledDelay = 150; // milliseconds - the time between enabling the uart and the first signal transmissions
bool isUartEnabled = false;

// Serial port parameters
constexpr long serialBaudRate = 19200; // baud rate for USB console output containing debug messages
constexpr long uartBaudRate = 19200; // baud rate for UART communication (19200 is required for Adapt Solutions)

HardwareSerial uartSerialPort(1); // uses UART1

enum ControllerStateEnum {
  INACTIVE,
  ARMED,
  DISARMED,
  TRANSMITTING, // transmitting commands to drive the seat motors
  BUTTON_0_STUCK_DOWN
};

enum ControllerLockOwner { 
  PHYSICAL, 
  VIRTUAL 
};

struct ControllerState {
  ControllerStateEnum controllerState;
  unsigned long stateTransitionTime;
  ControllerStateEnum priorControllerState;
  unsigned long priorStateTransitionTime;
  ControllerLockOwner lockOwner;
  unsigned long lockOwnerTimestamp;
};

ControllerState currentControllerState;

enum ButtonStateEnum { 
  BUTTON_UP, 
  BUTTON_DOWN 
};

struct ButtonState {
  int buttonId;
  ButtonStateEnum buttonState;
  unsigned long actionTime;
  ButtonStateEnum priorButtonState;
  unsigned long priorActionTime;
};

ButtonState currentButtonStates[NUMBER_OF_BUTTONS];

struct Debounce {
  unsigned long lastDebounceTime;
  int lastReading;
};

Debounce debouncers[NUMBER_OF_BUTTONS];

// Default values for ButtonState
constexpr ButtonStateEnum DEFAULT_BUTTON_STATE = BUTTON_UP;
constexpr ButtonStateEnum DEFAULT_PRIOR_BUTTON_STATE = BUTTON_DOWN;

// Default values for ControllerState
constexpr ControllerStateEnum DEFAULT_CONTROLLER_STATE = DISARMED;
constexpr ControllerStateEnum DEFAULT_PRIOR_CONTROLLER_STATE = DISARMED;
constexpr ControllerLockOwner DEFAULT_LOCK_OWNER = PHYSICAL;

unsigned int recentButtonChangeTime = 0; // the most recent time when any button was pressed or released
unsigned int recentCommandTime = 0; // the most recent time when a command was sent via UART
const unsigned int timeBetweenCommands = 337; // milliseconds, time between command resends if a button is held down (measured from lastCommandTime)

static unsigned int armedTimeout = 8500; // milliseconds
const unsigned int button0HoldThreshold = 8500; // 8 seconds threshold
const unsigned int debounceDelay = 50;      // 50 milliseconds debounce delay
const unsigned int ownerLockTimeout = 20000; // milliseconds


// ==============================================
// PUBLIC API - FUNCTIONS YOU CAN SAFELY CALL
// ==============================================
void armVirtual();
void setVirtualLockOwner();
void setPhysicalLockOwner();
void onButtonDown(int buttonId);
void onButtonUp(int buttonId);
void setArmedTimeout(unsigned int timeout);
bool isArmed();
unsigned int getArmedTimeout();
ControllerLockOwner getLockOwner();
String buttonStateToString(const ButtonState &button); // returns the state of the given button (the values from the ButtonState struct)
String controllerStateToString(); // returns the state of the controller (i.e., ARMED) and the state of each of the buttons (i.e., UP or DOWN)
// ==============================================
// END OF PUBLIC API
// ==============================================

// Private Function declarations
// DO _NOT_ CALL ANY OF THESE FUNCTIONS
void onVirtualButtonDown(int buttonId);
void onVirtualButtonUp(int buttonId);
void sendUARTMessage(char message);
// void sendPrefixMessage();
// void sendSuffixMessage();
bool updateButtonState(int buttonId, ButtonStateEnum action);
ButtonState scanButtonStates();
void initializeButtonPins();
void initializeButtonStates();
void initializeControllerState();
void initUART(HardwareSerial &serial, long baudRate, int txPin, int rxPin);
// void initUART();
void enableUART();
void disableUART();
bool updateControllerState(ControllerStateEnum newState);
void debug(const String &msg, int level);
void checkLockOwnerTimeout();
void handleControllerStateTransitions(ButtonState recentButton);
void transmitCommands(ButtonState recentButton);
void setLEDState();
bool buttonChanged(const ButtonState &button);
bool buttonHeldDown(const ButtonState &button);
bool buttonChangedTo(const ButtonState &button, ButtonStateEnum newState);
bool buttonHeldDownFor(const ButtonState &button, unsigned long timeoutInMillis);
// conditional compilation of testing function declarations
#ifdef TESTING_PUBLIC_INTERFACE
void testTask(void *pvParameters);
void testPublicInterface();
#endif

void setup() {
  Serial.begin(serialBaudRate); // Initialize console serial
  Serial.flush(); // Flush the serial buffer to clear bootloader messages

  debug("", DEBUG_PRIORITY_HIGH); // line break
  debug("setup() called", DEBUG_PRIORITY_HIGH);

  // Initialize the ESP32 LEDs
  initializeLEDs();

  // Create the LED control task
  ledBehavior = LED_BEHAVIOR_OFF;
  xTaskCreate(controlLEDs, "LED Control Task", 1024, NULL, 1, NULL);
  ledBehavior = LED_BEHAVIOR_CYCLE; // Cycle through all the LED colors

  // Initialize UART port (for communication with the Adapt Systems black box)
  initUART(uartSerialPort, uartBaudRate, uartTxPin, uartRxPin);
  debug("setup() UART initialized", DEBUG_PRIORITY_LOW);

  initializeTimes();

  // Initialize the ESP32 pins connected to the buttons
  initializeButtonPins();
  scanButtonStates();

  // Initialize button states
  debug(controllerStateToString(), DEBUG_PRIORITY_HIGH);
  debug("setup() calling initializeButtonStates()", DEBUG_PRIORITY_LOW);
  initializeButtonStates();
  debug(controllerStateToString(), DEBUG_PRIORITY_HIGH);

  debug("setup() calling initializeControllerState()", DEBUG_PRIORITY_LOW);
  initializeControllerState();

  //it's not clear whether or not these pulses are required at power-up
  if (SEND_POWER_UP_PULSES){
    sendPowerUpPulses();
  }

  debug("setup() complete", DEBUG_PRIORITY_HIGH);
  debug(controllerStateToString(), DEBUG_PRIORITY_HIGH);

  ledBehavior = LED_BEHAVIOR_OFF;

  setupBluetooth();

  #ifdef TESTING_PUBLIC_INTERFACE
    // Create the test task
    xTaskCreate(testTask, "Test Task", 2048, NULL, 1, NULL);
  #endif
}

void setupBluetooth() {
    // Initialize Bluetooth with the device name "ESP32_BT_Commander"
    String bluetoothName = "ESP32_BT_Commander";
    if (!SerialBT.begin(bluetoothName)) {
        Serial.println("Bluetooth init failed!");
        return;
    }

    Serial.println("Bluetooth initialized, device is discoverable as " + bluetoothName);

    // Register the callback to handle Bluetooth data reception
    SerialBT.register_callback([](esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
        if (event == ESP_SPP_DATA_IND_EVT) {
            String receivedData = SerialBT.readStringUntil('\n');
            receivedData.trim(); // Trim the data in-place
            handleBluetoothCommand(receivedData); // Now pass the trimmed string to handleCommand
        }
    });
}

void handleBluetoothCommand(String command) {
    // reset the lockOwnerTimestamp, since the remote user just sent a command
    currentControllerState.lockOwnerTimestamp = millis();

    if (command.startsWith("UP_")) {
        int buttonId = command.substring(3).toInt();
        if (buttonId >= 0 && buttonId <= 6) {
            onButtonUp(buttonId);
        }
    } else if (command.startsWith("DN_")) {
        int buttonId = command.substring(3).toInt();
        if (buttonId >= 0 && buttonId <= 6) {
            onButtonDown(buttonId);
        }
    } else if (command.equals("VIR")) {
        int buttonId = command.substring(3).toInt();
        if (buttonId >= 0 && buttonId <= 6) {
            setVirtualLockOwner();
        }
    } else if (command.equals("PHY")) {
        int buttonId = command.substring(3).toInt();
        if (buttonId >= 0 && buttonId <= 6) {
            setPhysicalLockOwner();
        }
    } else if (command.equals("ARM")) {
        int buttonId = command.substring(3).toInt();
        if (buttonId >= 0 && buttonId <= 6) {
            armVirtual();
        }
    } else {
      debug("Bluetooth command received but not recognized: " + command, DEBUG_PRIORITY_HIGH);
    }
}

void loop() {
  // The controller lock owner returns to the default owner, if there has been
  // no button activity for longer than the lock owner timeout. This handles the 
  // use-case in which a bluetooth device stops communicating. 
  checkLockOwnerTimeout();

  // Scans the state of all the buttons and returns the one with the most recent
  // status change. For physical buttons, this scan also updates the
  // currentButtonStates array, based on current button positions.
  ButtonState recentButtonEvent = scanButtonStates();

  // Sets the Controller State, based upon button states, etc.
  handleControllerStateTransitions(recentButtonEvent);

  // Transmits the appropriate command, based on the Controller State
  transmitCommands(recentButtonEvent);

  // debug(controllerStateToString(), DEBUG_PRIORITY_LOW);

  // vTaskDelay(30 / portTICK_PERIOD_MS); // Yields control to FreeRTOS (Delays loop execution for 30 ms)
  yield(); // gives other tasks a chance to run
}

/**
 * Arms the controller by pressing and releasing Button 0.
 * This function shall only be called by remote clients.
 */
void armVirtual(){
  onButtonDown(0);
  delay(100);
  onButtonUp(0);
}

void setPhysicalLockOwner() {
  debug("setPhysicalLockOwner() called)", DEBUG_PRIORITY_HIGH);
  if (currentControllerState.lockOwner == VIRTUAL){
    setLockOwner(PHYSICAL);
    //fast blink the GREEN LED as an indication that the physical controller is ready for use
    ledColor = LED_COLOR_GREEN;
    ledBehavior = LED_BEHAVIOR_SLOW_BLINK;
  }
}

void setVirtualLockOwner() {
  debug("setVirtualLockOwner() called)", DEBUG_PRIORITY_HIGH);
  if (currentControllerState.lockOwner == PHYSICAL){
    setLockOwner(VIRTUAL);
    //turn on the LED RED as an indication that the physical controller buttons are turned off
    ledColor = LED_COLOR_BLUE;
    ledBehavior = LED_BEHAVIOR_SLOW_BLINK;
  }
}

ControllerLockOwner getLockOwner() { 
  return currentControllerState.lockOwner; 
}

void setLockOwner(ControllerLockOwner newOwner) {
  if (currentControllerState.lockOwner != newOwner) {
    currentControllerState.lockOwner = newOwner;
    currentControllerState.lockOwnerTimestamp = millis();
    // Reinitialize button states when the owner changes
    initializeButtonStates();
    String ownerStr = (newOwner == PHYSICAL) ? "PHYSICAL" : "VIRTUAL";
    debug("Lock owner changed to " + ownerStr + " and button states initialized",
          DEBUG_PRIORITY_HIGH);
    debug("Current controller state: " + controllerStateToString(), DEBUG_PRIORITY_HIGH);

  } else {
    debug("Lock owner NOT changed.", DEBUG_PRIORITY_HIGH);
    // currentControllerState.lockOwnerTimestamp = millis(); //TODO is reseting the lockOwnerTimestamp a bug?
  }
}

// Setter for armedTimeout
void setArmedTimeout(unsigned int timeout) {
    armedTimeout = timeout;
}

// Getter for armedTimeout
unsigned int getArmedTimeout() {
    return armedTimeout;
}

// Checks if lockOwner has timed out. If timedout, set it to DEFAULT_LOCK_OWNER.
void checkLockOwnerTimeout() {
  if (millis() - currentControllerState.lockOwnerTimestamp > ownerLockTimeout && currentControllerState.lockOwner != DEFAULT_LOCK_OWNER) {    
    debug("Lock owner timed out", DEBUG_PRIORITY_HIGH);
    if (DEFAULT_LOCK_OWNER == PHYSICAL){
      setPhysicalLockOwner();
    } else if (DEFAULT_LOCK_OWNER == VIRTUAL){
      setVirtualLockOwner();
    }
  }
}

/**
 * Returns true if the controller state is Armed or Transmitting, else false
 */
bool isArmed() {
  if (currentControllerState.controllerState == ARMED || currentControllerState.controllerState == TRANSMITTING){
    return true;
  }
  return false;
}

/**
 * On power-up, the Adapt Solutions legacy controller sends out 4 voltage pulses.
 * It is not clear whether these pulses are actually required. But we're replicating 
 * them just in case they are required. 
 *
 * On power-up, the UART output voltage starts LOW
 * Pulse 1: The first pulse is 2.5 seconds HIGH
 *    This is followed 1.7 seconds LOW
 * Pulse 2: 1 pulse lasting 250 uSec (HIGH)
 *    This is followed by 1 second LOW 
 * Pulse 3: 1 pulse lasting 250 uSec (HIGH)
 *    This is followed by 1 second LOW 
 * Pulse 4: 1 pulse lasting 250 uSec (HIGH)
 *    The voltage then drops to LOW 
 */
void sendPowerUpPulses() {
    //delay for 1 second to wait for the ESP32 UART power-up voltage noise to end
    delay(1000);
    // Generate Pulse 1 for 2.5 seconds
    digitalWrite(uartTxPin, HIGH);
    delay(2500); // 2.5 seconds
    digitalWrite(uartTxPin, LOW);

    // Wait for 1.5 seconds
    delay(1700); // 1 second

    // Generate Pulse 2 for 250 uSecs
    digitalWrite(uartTxPin, HIGH);
    delayMicroseconds(250); // 250 microseconds
    digitalWrite(uartTxPin, LOW);

    // Wait for 2 seconds
    delay(2000); // 2 seconds

    // Generate Pulse 3 for 250 uSecs
    digitalWrite(uartTxPin, HIGH);
    delayMicroseconds(250); // 250 microseconds
    digitalWrite(uartTxPin, LOW);

    // Wait for 2 seconds
    delay(2000); // 2 seconds

    // Generate Pulse 4 for 250 uSecs
    digitalWrite(uartTxPin, HIGH);
    delayMicroseconds(250); // 250 microseconds
    digitalWrite(uartTxPin, LOW);
    //we end with the output LOW
}

/**
 * This function is designed to be called by the code that receives 
 * button DOWN commands from a remote wireless client.
 * It is specifically NOT designed to handle local button events.
 */ 
void onButtonDown(int buttonId) {
  if (currentControllerState.lockOwner == PHYSICAL) {
    debug("Error: onButtonDown called in PHYSICAL mode for virtual button " + String(buttonId) + ". This function should only be called in VIRTUAL mode.", DEBUG_PRIORITY_HIGH);
    return; // or handle the error appropriately
  }
  onVirtualButtonDown(buttonId);
}

/**
 * This function is designed to be called by the code that receives 
 * button UP commands from a remote wireless client.
 * It is specifically NOT designed to handle local button events.
 */ 
void onButtonUp(int buttonId) {
  if (currentControllerState.lockOwner == PHYSICAL) {
    debug("Error: onButtonUp called in PHYSICAL mode for virtual button " + String(buttonId) + ". This function should only be called in VIRTUAL mode.", DEBUG_PRIORITY_HIGH);
    return; // or handle the error appropriately
  }
  onVirtualButtonUp(buttonId);
}

void onVirtualButtonDown(int buttonId) {
  if (currentControllerState.lockOwner != VIRTUAL) {
    debug("Error: Attempt to press virtual button " + String(buttonId) + " when lock owner is not VIRTUAL.", DEBUG_PRIORITY_HIGH);
    // Optionally add additional error handling, like setting an error flag or state
    return; // Exit the function to avoid further processing
  }

  // Update the button state if the lock owner is virtual
  currentControllerState.lockOwnerTimestamp = millis(); // Reset the timeout to prevent accidental timeout
  updateButtonState(buttonId, BUTTON_DOWN);
}

void onVirtualButtonUp(int buttonId) {
  if (currentControllerState.lockOwner != VIRTUAL) {
    debug("Error: Attempt to release virtual button " + String(buttonId) + " when lock owner is not VIRTUAL.", DEBUG_PRIORITY_HIGH);
    // Optionally add additional error handling here as well
    return; // Exit the function to avoid further processing
  }

  // Update the button state if the lock owner is virtual
  currentControllerState.lockOwnerTimestamp = millis(); // Reset the timeout to keep the virtual control active
  updateButtonState(buttonId, BUTTON_UP);
}

/** Utility function */
String buttonStateToString(const ButtonState &button) {
  String result = "ButtonState { ";
  result += "Id: " + String(button.buttonId) + ", ";
  result +=
      "state: " + String(button.buttonState == BUTTON_DOWN ? "DOWN" : "UP") +
      ", ";
  result += "priorState: " +
            String(button.priorButtonState == BUTTON_DOWN ? "DOWN" : "UP") +
            ", ";
  result += "actionTime: " + String(button.actionTime) + ", ";
  result += "priorActionTime: " + String(button.priorActionTime);
  result += " }";
  return result;
}

/** Utility function 
 * Returns a one line string showing the current Controller State and the
 * current state of all of the buttons (i.e., UP or DOWN) 
 * For example: Lock Owner: PHYSICAL, Disarmed, 0:UP, 1:UP, 2:UP, 3:UP, 4:UP, 5:UP, 6:UP
 */
String controllerStateToString() {
  String result;

  // Add lock owner information
  switch (currentControllerState.lockOwner) {
  case PHYSICAL:
    result = "Lock Owner: PHYSICAL, ";
    break;
  case VIRTUAL:
    result = "Lock Owner: VIRTUAL, ";
    break;
  default:
    result = "Lock Owner: UNKNOWN, "; // Should not happen
    break;
  }

  // Add controller state information
  switch (currentControllerState.controllerState) {
  case INACTIVE:
    result += "INACTIVE";
    break;
  case ARMED:
    result += "ARMED";
    break;
  case TRANSMITTING:
    result += "TRANSMITTING";
    break;
  case DISARMED:
    result += "DISARMED";
    break;
  case BUTTON_0_STUCK_DOWN:
    result += "BUTTON_0_STUCK_DOWN";
    break;
  default:
    // This should never happen
    result += "---CODING ERROR---";
    break;
  }

  // Add button states information
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    result += ", " + String(i) + ":";
    if (currentButtonStates[i].buttonState == BUTTON_UP) {
      result += "UP";
    } else {
      result += "DOWN";
    }
  }

  return result;
}

// Initialize the UART port
void initUART(HardwareSerial &serial, long baudRate, int txPin, int rxPin) {
  serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
  // serial.begin(baudRate, SERIAL_8N1, rxPin, txPin, true);
  debug("UART initialized", DEBUG_PRIORITY_LOW);
  pinMode(uartTxPin, OUTPUT);
  digitalWrite(uartTxPin, LOW);
  isUartEnabled = false;
}

void enableUART() {
  if (!isUartEnabled) {
    debug("enableUART() UART PIN is LOW", DEBUG_PRIORITY_HIGH);
    // Set the UART TX pin to HIGH to enable UART communication
    digitalWrite(uartTxPin, HIGH);
    // sendPrefixMessage();

    // Initialize UART communication
    uartSerialPort.begin(uartBaudRate, SERIAL_8N1, uartRxPin, uartTxPin);
    isUartEnabled = true;
    // Delay so the first message signals are not sent immediately after the UART voltage is pulled HIGH.
    // This matches the Adapt Systems controller behavior.
    delay(uartEnabledDelay);
  }
}

void disableUART() {
  if (isUartEnabled) {
    // sendSuffixMessage();

    // Flush UART and disable it
    uartSerialPort.flush(); // Ensure all data is sent before disabling UART
    uartSerialPort.end();   // End UART communication

    // Set the UART TX pin to LOW
    pinMode(uartTxPin, OUTPUT); // Reinitialize as output
    digitalWrite(uartTxPin, LOW);
    isUartEnabled = false;
  }
}

// Send a character over UART (this sends the command to the Adapt Solutions
// black box)
void sendUARTMessage(char message) {
  // reset the lockOwnerTimestamp, since the user is sending a message
  currentControllerState.lockOwnerTimestamp = millis();

  if (uartSerialPort.availableForWrite()) {
    String formattedMessage = formatMessage(message);
    enableUART();
    uartSerialPort.print(formattedMessage); // Use print instead of println to
                                            // avoid adding a newline character

    String debugMessage = formattedMessage;
    debugMessage.replace("\r", "\\r");
    debug("Sent message: " + debugMessage, DEBUG_PRIORITY_HIGH);

  } else {
    debug("UART not available for write", DEBUG_PRIORITY_HIGH);
  }
}

// Function to format character messages per the required format of Adapt
// Solutions
String formatMessage(char inputChar) {
  String formattedMessage = "&";
  formattedMessage += inputChar;
  formattedMessage += '\r'; // Carriage return before hex value
  formattedMessage += String((int)inputChar, HEX);
  return formattedMessage;
}

// Initialize button pins
void initializeButtonPins() {
  unsigned long now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    int initialReading = digitalRead(buttonPins[i]);
    debouncers[i].lastDebounceTime = now;
    debouncers[i].lastReading = initialReading;

    ButtonStateEnum state;
    if (initialReading == LOW) {
      state = BUTTON_DOWN;
    } else {
      state = BUTTON_UP;
    }

    // Also initialize the button states to match the initial readings
    currentButtonStates[i] = {i, state, now, DEFAULT_PRIOR_BUTTON_STATE, now};

    // Debug: Initial reading of the button pin
    String readingStr = (initialReading == LOW) ? "LOW (DOWN)" : "HIGH (UP)";
    debug("Button " + String(i) + " initial reading: " + readingStr,
          DEBUG_PRIORITY_LOW);
  }
  debug("initializeButtonPins() configured button pins to INPUT_PULLUP",
        DEBUG_PRIORITY_LOW);
}

// Initialize button states
void initializeButtonStates() {
  switch (currentControllerState.lockOwner) {
  case PHYSICAL:
    initializePhysicalButtonStates();
    break;
  case VIRTUAL:
    initializeVirtualButtonStates();
    break;
  }
}

void initializePhysicalButtonStates() {
  unsigned long now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    int reading = digitalRead(buttonPins[i]);
    ButtonStateEnum state;
    if (reading == LOW) {
      state = BUTTON_DOWN;
    } else {
      state = BUTTON_UP;
    }
    currentButtonStates[i] = {i, state, now, state, now};
    debouncers[i].lastReading = reading;
  }
  debug("initializePhysicalButtonStates() initialized physical button states",
        DEBUG_PRIORITY_LOW);
}

void initializeVirtualButtonStates() {
  unsigned long now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    // Assuming virtual buttons might be initialized differently
    // Example: set all virtual buttons to BUTTON_UP by default
    currentButtonStates[i] = {i, BUTTON_UP, now, BUTTON_UP, now};
    debouncers[i].lastReading = HIGH; // Default to HIGH (not pressed)
  }
  debug("initializeVirtualButtonStates() initialized virtual button states",
        DEBUG_PRIORITY_LOW);
}

/**
 * Reads the state of each button and returns the ButtonState of the button that
 * changed state most recently
 */
ButtonState scanButtonStates() {
  ButtonState recentButton = {-1, BUTTON_UP, 0, BUTTON_UP, 0}; // Default to no recent button event

  switch (currentControllerState.lockOwner) {
  case PHYSICAL:
    recentButton = checkPhysicalButtons();
    break;
  case VIRTUAL:
    recentButton = checkVirtualButtons();
    break;
  }

  if (recentButton.buttonId == -1) {
    // debug("scanButtonStates() Returning placeholder ButtonState object", DEBUG_PRIORITY_LOW);
  } else {
    // Update the recentButtonChangeTime here if a real button state change occurred
    recentButtonChangeTime = recentButton.actionTime;
    // debug("scanButtonStates() Returning actual ButtonState object", DEBUG_PRIORITY_LOW);
  }

  return recentButton;
}

ButtonState checkPhysicalButtons() {
  ButtonState recentButton = {-1, BUTTON_UP, 0, BUTTON_UP, 0}; // Default to no recent button event

  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    int pin = buttonPins[i];
    int reading = digitalRead(pin);

    // Determine if the reading has changed from the last reading
    if (reading != debouncers[i].lastReading) {
      debouncers[i].lastDebounceTime = millis();
    }

    // Check if the debounce delay has passed
    if ((millis() - debouncers[i].lastDebounceTime) > debounceDelay) {
      ButtonStateEnum newState = (reading == LOW) ? BUTTON_DOWN : BUTTON_UP;
      if (updateButtonState(i, newState)) {
        // If the state changed, check if it's the most recent
        if (currentButtonStates[i].actionTime > recentButton.actionTime) {
          recentButton = currentButtonStates[i];
        }
      }
    }

    // Always check if the current button's actionTime is more recent
    if (currentButtonStates[i].actionTime > recentButton.actionTime) {
      recentButton = currentButtonStates[i];
    }

    debouncers[i].lastReading = reading;
  }

  return recentButton;
}

ButtonState checkVirtualButtons() {
  debug("checkVirtualButtons() called", DEBUG_PRIORITY_LOW);

  ButtonState recentButton = {-1, BUTTON_UP, 0, BUTTON_UP, 0}; // Default to no recent button event

  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    if (buttonChanged(currentButtonStates[i])) {
      if (currentButtonStates[i].actionTime > recentButton.actionTime) {
        recentButton = currentButtonStates[i];
      }
    }
  }
  return recentButton;
}

/**
 * Updates the state of the given buttonId and returns true if its button state
 * changed, else false
 */
bool updateButtonState(int buttonId, ButtonStateEnum action) {
  unsigned long actionTime = millis();
  ButtonState &button = currentButtonStates[buttonId];
  if (button.buttonState != action) {
    button.priorButtonState = button.buttonState;
    button.priorActionTime = button.actionTime;
    button.buttonState = action;
    button.actionTime = actionTime;
    // Update the global variable keeping track of the most recent button change
    recentButtonChangeTime = actionTime;
    debug("Button state changed: " + controllerStateToString(), DEBUG_PRIORITY_HIGH);
    return true;
  }
  return false;
}

void debug(const String &msg, int messagePriority) {
  if (messagePriority >= debugSetting) {
    Serial.println(msg);
  }
}

/**
 * Updates the currentControllerState and returns true if the state changed,
 * else false
 */
bool updateControllerState(ControllerStateEnum newState) {
  unsigned long now = millis();

  // Check if the state is actually changing
  if (currentControllerState.controllerState != newState) {
    debug("updateControllerState(): Attempting to update state from " +
              String(currentControllerState.controllerState) + " to " +
              String(newState),
          DEBUG_PRIORITY_LOW);

    // Update the prior state information
    currentControllerState.priorControllerState = currentControllerState.controllerState;
    currentControllerState.priorStateTransitionTime = currentControllerState.stateTransitionTime;

    // Update the current state and time
    currentControllerState.controllerState = newState;
    currentControllerState.stateTransitionTime = now;

    // Set the LED state based on the new controller state
    setLEDState();

    debug("updateControllerState(): Controller state updated to: " +
              controllerStateToString(),
          DEBUG_PRIORITY_LOW);
    return true;
  }
  return false;
}

// Puts the LEDs in the proper state (blinking, on, off, etc) associated with
// the current controller state LEDControl.ino scans for changes to these values
// and controls the LED behaviors based on these settings.
void setLEDState() {
  switch (currentControllerState.controllerState) {
  case INACTIVE:
    ledBehavior = LED_BEHAVIOR_OFF;
    break;
  case ARMED:
    ledColor = LED_COLOR_GREEN;
    ledBehavior = LED_BEHAVIOR_ON;
    break;
  case TRANSMITTING:
    ledColor = LED_COLOR_GREEN;
    ledBehavior = LED_BEHAVIOR_BLINK;
    break;
  case DISARMED:
    ledBehavior = LED_BEHAVIOR_OFF;
    break;
  case BUTTON_0_STUCK_DOWN:
    ledBehavior = LED_BEHAVIOR_CYCLE;
    break;
  default:
    // Handle unexpected states if necessary
    break;
  }
}

void initializeControllerState() {
  debug("initializeControllerState() called", DEBUG_PRIORITY_LOW);
  unsigned long now = millis();

  if (currentControllerState.lockOwner == PHYSICAL) {
    debug("Lock owner is PHYSICAL", DEBUG_PRIORITY_LOW);

    // If Button 0 is being held down at startup, all the other controller
    // buttons are inactivated
    if (currentButtonStates[0].buttonState == BUTTON_DOWN) {
      currentControllerState = {
          INACTIVE,           now, DEFAULT_PRIOR_CONTROLLER_STATE, now,
          DEFAULT_LOCK_OWNER, now};
      updateControllerState(INACTIVE);
      debug("Button 0 is DOWN at startup, setting state to INACTIVE",
            DEBUG_PRIORITY_HIGH);
    } else {
      currentControllerState = {
          DEFAULT_CONTROLLER_STATE, now, DEFAULT_PRIOR_CONTROLLER_STATE, now,
          DEFAULT_LOCK_OWNER,       now};
      updateControllerState(DEFAULT_CONTROLLER_STATE);
      debug("Button 0 is not DOWN, setting state to DEFAULT_CONTROLLER_STATE",
            DEBUG_PRIORITY_HIGH);
    }
  } else {
    debug("Lock owner is not PHYSICAL, setting default states",
          DEBUG_PRIORITY_HIGH);
    unsigned long now = millis();
    currentControllerState = {
        DEFAULT_CONTROLLER_STATE, now, DEFAULT_PRIOR_CONTROLLER_STATE, now,
        DEFAULT_LOCK_OWNER,       now};
  }

  debug("initializeControllerState() initialized state to: " + controllerStateToString(),
        DEBUG_PRIORITY_HIGH);
}

void handleControllerStateTransitions(ButtonState recentButton) {
    if (recentButton.buttonId == -1) {
        // Ignore this. ButtonId -1 indicates there were NO recent button events.
        return;
    }

    // Log the current state before any transitions
    debug("handleControllerStateTransitions(ENTERED): Current state: " + controllerStateToString(), DEBUG_PRIORITY_LOW);

    switch (currentControllerState.controllerState) {
        case INACTIVE:
            handleInactiveState(recentButton);
            break;
        case BUTTON_0_STUCK_DOWN:
            handleButton0StuckDownState(recentButton);
            break;
        case ARMED:
            handleArmedState(recentButton);
            break;
        case TRANSMITTING:
            handleTransmittingState(recentButton);
            break;
        case DISARMED:
            handleDisarmedState(recentButton);
            break;
        default:
            // Handle unexpected state if necessary
            break;
    }
}

void handleInactiveState(ButtonState recentButton) {
    if (recentButton.buttonId == 0 && buttonChangedTo(recentButton, BUTTON_UP)) {
        updateControllerState(DISARMED);
        disableUART();
        debug("handleInactiveState() Controller state updated from INACTIVE to DISARMED: " + controllerStateToString(), DEBUG_PRIORITY_LOW);
    } else {
        debug("handleInactiveState() Controller state is still INACTIVE: " + controllerStateToString(), DEBUG_PRIORITY_LOW);
    }
}

void handleButton0StuckDownState(ButtonState recentButton) {
    if (recentButton.buttonId == 0) {
        if (buttonChangedTo(recentButton, BUTTON_UP)) {
            updateControllerState(ARMED);
            enableUART();
            debug("handleButton0StuckDownState() Controller state updated from BUTTON_0_STUCK_DOWN to ARMED: " + controllerStateToString(), DEBUG_PRIORITY_HIGH);
        } else if (buttonHeldDownFor(recentButton, button0HoldThreshold)) {
            updateControllerState(BUTTON_0_STUCK_DOWN);
            disableUART();
            debug("handleButton0StuckDownState() Controller state updated to BUTTON_0_STUCK_DOWN: " + controllerStateToString(), DEBUG_PRIORITY_HIGH);
        } else {
            debug("handleButton0StuckDownState() LOGIC NEEDED HERE: " + controllerStateToString(), DEBUG_PRIORITY_HIGH);
            debug("handleButton0StuckDownState() ButtonState: " + buttonStateToString(recentButton), DEBUG_PRIORITY_HIGH);
        }
    }
}

void handleArmedState(ButtonState recentButton) {
    if (recentButton.buttonId == 0 && buttonHeldDownFor(recentButton, button0HoldThreshold)) {
        updateControllerState(INACTIVE);
        disableUART();
    } else if (recentButton.buttonId > 0 && recentButton.buttonId < 7 && recentButton.buttonState == BUTTON_DOWN) {
        updateControllerState(TRANSMITTING);
        enableUART();
    } else if ((millis() - recentButtonChangeTime) > armedTimeout) {
        updateControllerState(DISARMED);
        disableUART();
    }
}

void handleTransmittingState(ButtonState recentButton) {
    if (recentButton.buttonId > 0 && recentButton.buttonId < 7 && recentButton.buttonState == BUTTON_UP) {
        updateControllerState(ARMED);
        enableUART();
        debug("handleTransmittingState() Controller state updated from TRANSMITTING to ARMED: " + controllerStateToString(), DEBUG_PRIORITY_HIGH);
    }
}

void handleDisarmedState(ButtonState recentButton) {
    if (recentButton.buttonId == 0) {
        if (buttonChangedTo(recentButton, BUTTON_DOWN)) {
            updateControllerState(ARMED);
            enableUART();
        } else if (buttonHeldDownFor(recentButton, button0HoldThreshold)) {
            updateControllerState(INACTIVE);
            disableUART();
        }
    }
}

/**
 * Returns true if the buttonState has changed since its priorButtonState, else false
 */
bool buttonChanged(const ButtonState &button) {
  return (button.buttonState != button.priorButtonState &&
          button.actionTime > button.priorActionTime);
}

/**
 * Returns true if the buttonState has changed since its priorButtonState, else false
 */
bool buttonChangedTo(const ButtonState &button, ButtonStateEnum newState) {
  return (button.buttonState == newState &&
          button.priorButtonState != newState &&
          button.actionTime > button.priorActionTime);
}

/**
 * Returns true if the button is being held down, else false
 */
bool buttonHeldDown(const ButtonState &button) {
  return (button.buttonState == BUTTON_DOWN &&
          button.buttonState == button.priorButtonState &&
          button.actionTime > button.priorActionTime);
}

bool buttonHeldDownFor(const ButtonState &button, unsigned long timeoutInMillis) {
  return (button.buttonState == BUTTON_DOWN &&
          (millis() - button.actionTime) > timeoutInMillis);
}

void transmitCommands(ButtonState recentButton) {
  static unsigned int gCommandCounter = 0;   // Counter for 'G' commands
  static unsigned long lastGCommandTime = 0; // Last time a 'G' command was sent
  static unsigned long lastWarningTime = 0;  // Last time a ']' command was sent

  unsigned long currentTime = millis();

  if (currentControllerState.controllerState == BUTTON_0_STUCK_DOWN) {
    // Send 'G' command 10 times at intervals defined by timeBetweenCommands

    if (gCommandCounter < 10) {
      if (currentTime - lastGCommandTime >= timeBetweenCommands) {
        sendUARTMessage('G');
        gCommandCounter++;
        lastGCommandTime = currentTime;
      }
    } else if (currentTime - lastWarningTime >= 4000) {
      // After sending 'G' 10 times, send ']' command every 4 seconds
      sendUARTMessage(']');
      lastWarningTime = currentTime;
    }
  } else {
    // Reset the counters and timers when state is no longer BUTTON_0_STUCK_DOWN
    gCommandCounter = 0;
    lastGCommandTime = millis();
    lastWarningTime = millis();
  }

  // Continue to process commands when not in INACTIVE state
  if (millis() - recentCommandTime > timeBetweenCommands) {
    if (multipleButtonsDown()) {
      sendUARTMessage('H');
      recentCommandTime = millis();
      return;
    }

    if (currentControllerState.controllerState == ARMED &&
        recentButton.buttonState == BUTTON_DOWN) {
      char command = commands[recentButton.buttonId];
      sendUARTMessage(command);
      recentCommandTime = millis();
      return;
    }

    if (currentControllerState.controllerState == TRANSMITTING &&
        recentButton.buttonState == BUTTON_DOWN) {
      char command = commands[recentButton.buttonId];
      sendUARTMessage(command);
      recentCommandTime = millis();
      return;
    }

    if (recentButton.buttonId == 0 && recentButton.buttonState == BUTTON_DOWN) {
      char command = commands[0];
      sendUARTMessage(command);
      recentCommandTime = millis();
      return;
    }

    if (currentControllerState.controllerState == ARMED) {
      char command = commands[0];
      sendUARTMessage(command);
      recentCommandTime = millis();
      return;
    }
  }
}

/**
 * Returns true if more than one button is being held down, else false
 */
bool multipleButtonsDown() {
  int downCount = 0;
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    if (currentButtonStates[i].buttonState == BUTTON_DOWN) {
      downCount++;
      if (downCount > 1) {
        return true; // Early exit if more than one button is found to be down
      }
    }
  }
  return false; // No or only one button is down
}

void initializeTimes() {
  recentButtonChangeTime = millis();
  recentCommandTime = millis();
}

// /**-----------------------------------------------
//  * Keeping this code for now, although it is likely NOT needed. The prefix and suffix
//  * signals seen on the oscilloscope from the Adapt Solutions legacy controller appear 
//  * to be voltage noise, rather than meaningful UART signals. 
//  * -----------------------------------------------
//  * Function intended to output a short prefix message via UART before actual messages are sent.
//  * This does not seem to be needed, since what looked like prefixes are actually just noise.
//  */ 
// void sendPrefixMessage() {
//     char prefix[2];
//     prefix[0] = 0xF0; // Hex F0
//     prefix[1] = '~';  // ~ character

//     if (uartSerialPort.availableForWrite()) {
//         uartSerialPort.write(prefix, sizeof(prefix));
//         debug("Sent prefix message: 0xF0 '~'", DEBUG_PRIORITY_HIGH);
//     } else {
//         debug("sendPrefixMessage() UART not available for write", DEBUG_PRIORITY_HIGH);
//     }
// }

// /**
//  * Function intended to output a short suffix message via UART after actual messages are sent.
//  * This does not seem to be needed, since what looked like suffixes are actually just noise.
//  */ 
// void sendSuffixMessage() {
//     char suffix = 0xC0; // Hex C0

//     if (uartSerialPort.availableForWrite()) {
//         uartSerialPort.write(suffix);
//         debug("Sent suffix message: 0xC0", DEBUG_PRIORITY_HIGH);
//     } else {
//         debug("sendSuffixMessage() UART not available for write", DEBUG_PRIORITY_HIGH);
//     }
// }


// ---------------------------------------
// conditional compilation of testing code
// ---------------------------------------
#ifdef TESTING_PUBLIC_INTERFACE

// Custom assert macro for testing
#define custom_assert(cond, msg)                                               \
  do {                                                                         \
    if (!(cond)) {                                                             \
      Serial.print("Assertion failed: ");                                      \
      Serial.println(msg);                                                     \
      while (1)                                                                \
        ;                                                                      \
    }                                                                          \
  } while (0)

// Task to run the public interface tests
void testTask(void *pvParameters) {
  testPublicInterface();
  vTaskDelete(NULL); // Delete the task when done
}

// Function to run the public interface tests
void testPublicInterface() {
  Serial.println("");
  Serial.println("");
  Serial.println("==================================================");
  Serial.println("Compiled in Test Mode");
  Serial.println("Starting Public Interface Tests...");
  Serial.println("==================================================");
  Serial.println("");
  delay(5000); // 3-second pause before starting the tests

  // TEST 1: Setting the lock owner to VIRTUAL or PHYSICAL
  Serial.println("\n--------------------");
  Serial.println(
      "TEST 1: Setting the lock owner to VIRTUAL or PHYSICAL (lock=" +
      String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") + ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  Serial.println("Setting lock owner to VIRTUAL");
  Serial.println(controllerStateToString());
  setVirtualLockOwner();
  custom_assert(getLockOwner() == VIRTUAL,
                "Lock owner should be VIRTUAL after setVirtualLockOwner()");
  Serial.println(controllerStateToString());
  delay(4000); // 3-second pause before starting the tests
  Serial.println("----END STEP----");

  Serial.println("Setting lock owner to PHYSICAL (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println(controllerStateToString());
  setPhysicalLockOwner();
  custom_assert(getLockOwner() == PHYSICAL,
                "Lock owner should be PHYSICAL after setPhysicalLockOwner()");
  Serial.println(controllerStateToString());
  delay(4000); // 3-second pause before starting the tests
  Serial.println("----END STEP----");

  Serial.println("Setting lock owner to VIRTUAL (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println(controllerStateToString());
  setVirtualLockOwner();
  custom_assert(getLockOwner() == VIRTUAL,
                "Lock owner should be VIRTUAL after setVirtualLockOwner()");
  Serial.println(controllerStateToString());
  delay(4000); // 3-second pause before starting the tests
  Serial.println("----END----");

  // TEST 2: Getting the lock owner
  Serial.println("\n--------------------");
  Serial.println("TEST 2: Getting the lock owner (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  Serial.println(controllerStateToString());
  ControllerLockOwner lockOwner = getLockOwner();
  custom_assert(lockOwner == VIRTUAL, "Lock owner should be VIRTUAL in TEST 2");
  Serial.print("Lock Owner: ");
  Serial.println(lockOwner == VIRTUAL ? "VIRTUAL" : "PHYSICAL");
  Serial.println(controllerStateToString());
  Serial.println("----END----");

  // TEST 3: Setting to ARMED mode
  Serial.println("\n--------------------");
  Serial.println("TEST 3: Setting to ARMED mode (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  // Verify initial state is DISARMED
  Serial.println(controllerStateToString());
  custom_assert(currentControllerState.controllerState == DISARMED,
                "Initial state should be DISARMED in TEST 3");

  custom_assert(isArmed() == false,
                "isArmed() should return false in TEST 3");

  Serial.println("Pushing button 0 (DOWN)");
  onButtonDown(0);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());

  Serial.println("Releasing button 0 (UP)");
  onButtonUp(0);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(currentControllerState.controllerState == ARMED,
                "State should be ARMED after button 0 is released in TEST 3");

  custom_assert(isArmed() == true,
                "isArmed() should return true in TEST 3");

  delay(10000); // Wait for 10 seconds
  Serial.println(controllerStateToString());
  custom_assert(currentControllerState.controllerState == DISARMED,
                "State should be DISARMED after timeout in TEST 3");
  Serial.println("----END----");

  // TEST 4: Setting to ARMED mode and individually pushing each action button
  Serial.println("\n--------------------");
  Serial.println("TEST 4: Setting to ARMED mode and individually pushing each "
                 "action button (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  Serial.println("Arming by pushing button 0 (DOWN)");
  Serial.println(controllerStateToString());
  onButtonDown(0);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("0:DOWN") > -1,
                "Button 0 should be DOWN in TEST 4");

  Serial.println("Releasing button 0 (UP)");
  onButtonUp(0);
  delay(2000); // Wait for 2 seconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("0:UP") > -1,
                "Button 0 should be UP in TEST 4");

  for (int i = 1; i <= 6; i++) {
    delay(2000); // Wait for 2 seconds
    Serial.print("Pushing button ");
    Serial.print(i);
    Serial.println(" (DOWN)");
    onButtonDown(i);
    delay(60); // Wait for 60 milliseconds
    Serial.println(controllerStateToString());
    custom_assert(controllerStateToString().indexOf(String(i) + ":DOWN") > -1,
                  "Button " + String(i) + " should be DOWN in TEST 4");

    delay(2000); // Wait for 2 seconds

    Serial.print("Releasing button ");
    Serial.print(i);
    Serial.println(" (UP)");
    onButtonUp(i);
    delay(60); // Wait for 60 milliseconds
    Serial.println(controllerStateToString());
    custom_assert(controllerStateToString().indexOf(String(i) + ":UP") > -1,
                  "Button " + String(i) + " should be UP in TEST 4");

    delay(2000); // Wait for 2 seconds
  }

  delay(10000); // Wait for 10 seconds
  Serial.println(controllerStateToString());
  Serial.println("----END----");

  // TEST 5: Arming and then holding down 2 buttons simultaneously
  Serial.println("\n--------------------");
  Serial.println(
      "TEST 5: Arming and then holding down 2 buttons simultaneously (lock=" +
      String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") + ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  Serial.println("Pushing button 0 (DOWN)");
  Serial.println(controllerStateToString());
  onButtonDown(0);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("0:DOWN") > -1,
                "Button 0 should be DOWN in TEST 5");

  Serial.println("Releasing button 0 (UP)");
  onButtonUp(0);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("0:UP") > -1,
                "Button 0 should be UP in TEST 5");

  delay(1000); // Wait for 1 second

  Serial.println("Pushing button 1 (DOWN)");
  onButtonDown(1);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("1:DOWN") > -1,
                "Button 1 should be DOWN in TEST 5");

  Serial.println("Pushing button 2 (DOWN)");
  onButtonDown(2);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("2:DOWN") > -1,
                "Button 2 should be DOWN in TEST 5");

  delay(2000); // Wait for 2 seconds

  Serial.println("Releasing button 2 (UP)");
  onButtonUp(2);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("2:UP") > -1,
                "Button 2 should be UP in TEST 5");

  delay(2000); // Wait for 2 seconds

  Serial.println("Releasing button 1 (UP)");
  onButtonUp(1);
  delay(60); // Wait for 60 milliseconds
  Serial.println(controllerStateToString());
  custom_assert(controllerStateToString().indexOf("1:UP") > -1,
                "Button 1 should be UP in TEST 5");

  delay(10000); // Wait for 10 seconds
  Serial.println(controllerStateToString());
  Serial.println("----END----");

  // TEST 6: Setting the lock owner to PHYSICAL
  Serial.println("\n--------------------");
  Serial.println("TEST 6: Setting the lock owner to PHYSICAL (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  Serial.println(controllerStateToString());
  setPhysicalLockOwner();
  custom_assert(getLockOwner() == PHYSICAL,
                "Lock owner should be PHYSICAL in TEST 6");
  Serial.println(controllerStateToString());
  Serial.println("----END----");
  delay(1000); // Wait for 1 second

  // TEST 7: Getting the lock owner
  Serial.println("\n--------------------");
  Serial.println("TEST 7: Getting the lock owner (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  Serial.println(controllerStateToString());
  lockOwner = getLockOwner();
  custom_assert(lockOwner == PHYSICAL,
                "Lock owner should be PHYSICAL in TEST 7");
  Serial.print("Lock Owner: ");
  Serial.println(lockOwner == VIRTUAL ? "VIRTUAL" : "PHYSICAL");
  Serial.println(controllerStateToString());
  Serial.println("----END----");
  delay(1000); // Wait for 1 second

  // TEST 8: Test API button calls while in PHYSICAL mode
  Serial.println("\n--------------------");
  Serial.println("TEST 8: Test API onButton...() calls while in PHYSICAL mode "
                 "-- these should all FAIL (lock=" +
                 String(getLockOwner() == VIRTUAL ? "VIRTUAL" : "PHYSICAL") +
                 ")");
  Serial.println("--------------------");
  delay(3000); // 3-second pause before starting the test

  for (int i = 0; i <= 6; i++) {
    Serial.print("Pushing button ");
    Serial.print(i);
    Serial.println(" (DOWN)");
    onButtonDown(i);
    delay(60); // Wait for 60 milliseconds
    Serial.println(controllerStateToString());
    custom_assert(controllerStateToString().indexOf(String(i) + ":DOWN") == -1,
                  "Button " + String(i) +
                      " should NOT be DOWN in PHYSICAL mode");

    Serial.print("Releasing button ");
    Serial.print(i);
    Serial.println(" (UP)");
    onButtonUp(i);
    delay(60); // Wait for 60 milliseconds
    Serial.println(controllerStateToString());
    custom_assert(controllerStateToString().indexOf(String(i) + ":UP") > -1,
                  "Button " + String(i) + " should be UP in PHYSICAL mode");
  }

  delay(10000); // Wait for 10 seconds
  Serial.println(controllerStateToString());
  Serial.println("----END----");

  Serial.println("Public Interface Tests Completed.");

}

#endif
