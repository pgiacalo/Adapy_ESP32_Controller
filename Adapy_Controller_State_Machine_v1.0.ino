#include <Arduino.h>

// GPIO pins for all 7 buttons (#0 thru #6)
constexpr int buttonPins[] = {32, 33, 21, 26, 18, 27, 4}; 

// Custom UART pins (avoiding the Serial Tx/Rx pins used for console output)
constexpr int uartTxPin = 17; 
constexpr int uartRxPin = 16;

// GPIO pins for the RGB LED
constexpr int greenLEDPin = 12; 
constexpr int redLEDPin = 13; 
constexpr int blueLEDPin = 14; 

// Debug levels
constexpr int DEBUG_HIGH = 2;
constexpr int DEBUG_LOW = 1;
constexpr int DEBUG_NONE = 0;
int currentDebugLevel = DEBUG_LOW;

// Serial port parameters
constexpr long serialBaudRate = 19200;  // baud rate for USB console output containing debug messages
constexpr long uartBaudRate = 19200;  // baud rate for UART communication (19200 is the required rate for Adapt Solutions controller messages)

enum ControllerStateEnum {
    INACTIVE,
    ARMED,
    DISARMED,
    BUTTON_0_STUCK
};

enum ButtonStateEnum {
    BUTTON_UP,
    BUTTON_DOWN
};

enum ControllerLockOwner {
    PHYSICAL,
    VIRTUAL
};

struct ButtonState {
    int buttonId;
    ButtonStateEnum buttonState;
    unsigned long actionTime;
    ButtonStateEnum priorButtonState;
    unsigned long priorActionTime;
    unsigned long debounceTime;
};

struct ControllerState {
    ControllerStateEnum controllerState;
    unsigned long stateTransitionTime;
    ControllerStateEnum priorControllerState;
    unsigned long priorStateTransitionTime;
    ControllerLockOwner lockOwner;
    unsigned long lockOwnerTimestamp;
};

struct Debounce {
    unsigned long lastDebounceTime;
    int lastReading;
};

// Default values for ButtonState
constexpr ButtonStateEnum DEFAULT_BUTTON_STATE = BUTTON_UP;
constexpr ButtonStateEnum DEFAULT_PRIOR_BUTTON_STATE = BUTTON_DOWN;

// Default values for ControllerState
constexpr ControllerStateEnum DEFAULT_CONTROLLER_STATE = DISARMED;
constexpr ControllerStateEnum DEFAULT_PRIOR_CONTROLLER_STATE = DISARMED;
constexpr ControllerLockOwner DEFAULT_LOCK_OWNER = PHYSICAL;

const unsigned int NUMBER_OF_BUTTONS = 7; 

ControllerState currentControllerState;
ButtonState currentButtonStates[NUMBER_OF_BUTTONS];
unsigned int recentButtonChangeTime = 0;  // the most recent time when any button was pressed or released

Debounce debouncers[NUMBER_OF_BUTTONS];

constexpr unsigned int messageInterval = 340;  // milliseconds, the time between message resends if a button is held down (measured from lastMessageTime)
const unsigned int armedTimeout = 8000; // milliseconds
const unsigned int button0HoldThreshold = 8000; // 8 seconds threshold
const unsigned int debounceDelay = 50; // 50 milliseconds debounce delay
const unsigned int ownerLockTimeout = 8000; // milliseconds

HardwareSerial uartSerialPort(1);

// Function declarations
// PUBLIC FUNCTIONS - YOU CAN CALL THESE 3 FUNCTIONS
void onVirtualButtonDown(int buttonId); //public interface
void onVirtualButtonUp(int buttonId);   //public interface
void setVirtualLockOwner();             //public interface

// Private Function declarations 
// DO NOT CALL ANY OF THESE FUNCTION - THEY ARE ALL PRIVATE
void onPhysicalButtonDown(int buttonId);
void onPhysicalButtonUp(int buttonId);
void setPhysicalLockOwner();
void setVirtualLockOwner();
void handleButtonAction(int buttonId, ButtonStateEnum action);
void handleButton0(ButtonStateEnum action, bool buttonStateChanged);
void handleMotorButtons(int buttonId, ButtonStateEnum action, bool buttonStateChanged);
void moveMotor(int motorId, const char* direction);
void sendUARTMessage(char message);
String stateToString();
bool updateButtonState(int buttonId, ButtonStateEnum action);
ButtonState checkButtons();
void initializeButtonPins();
void initializeButtonStates();
void initializeLEDs();
void cycleLEDs();
void fadeLED(int pin, int duration);
void updateLEDState();
void resetLEDs();
void debug(const String& msg, int level);
bool updateControllerState(ControllerStateEnum newState);
void initializeControllerState();
void checkLockOwnerTimeout();
void handleControllerStateTransitions(ButtonState recentButton);
void handleSendCommands(ButtonState recentButton);
void handleLEDs();
bool buttonChanged(const ButtonState& button);
bool buttonHeldDown(const ButtonState& button);

void setup() {
    Serial.begin(serialBaudRate); // Initialize console serial

    debug("setup() called", DEBUG_LOW);

    // Initialize UART port (for communication with the Adapt Systems black box)
    initUART(uartSerialPort, uartBaudRate, uartTxPin, uartRxPin); 

    initializeControllerState();
    initializeButtonStates();

    //initialize the ESP32 pins connected to the buttons
    initializeButtonPins(); 
    //initialize the ESP32 LEDs
    initializeLEDs();
    //at startup, the ESP32 LEDs are cycled thru all colors (matches behavior of Adapt's existing controller)
    cycleLEDs();

    debug("setup() complete", DEBUG_LOW);
    debug(stateToString(), DEBUG_LOW);
}

void loop() {
    //TODO - do we want the controller to timeout (matches existing Adapt controller behavior)
    //checkLockOwnerTimeout();  

    ButtonState recentButton = checkButtons();

    handleControllerStateTransitions(recentButton);

    handleSendCommands(recentButton);

    handleLEDs();

    debug(stateToString(), DEBUG_LOW);

    delay(1000); // Adjust delay as necessary
    yield();
}

void setPhysicalLockOwner() {
  setLockOwner(PHYSICAL);
}

void setVirtualLockOwner() {
  setLockOwner(VIRTUAL);
}

void setLockOwner(ControllerLockOwner newOwner) {
    if (currentControllerState.lockOwner != newOwner) {
        currentControllerState.lockOwner = newOwner;
        currentControllerState.lockOwnerTimestamp = millis();
        initializeButtonStates(); // Reinitialize button states when the owner changes
        debug("Lock owner changed and button states initialized", DEBUG_LOW);
    } else {
        currentControllerState.lockOwnerTimestamp = millis();
    }
}

// Checks if lockOwner has timed out. If timedout, set it to DEFAULT_LOCK_OWNER. 
void checkLockOwnerTimeout() {
    if (millis() - currentControllerState.lockOwnerTimestamp > ownerLockTimeout) {
        setLockOwner(DEFAULT_LOCK_OWNER);
        debug("Lock owner timed out, reset to PHYSICAL", DEBUG_LOW);
        currentControllerState.lockOwnerTimestamp = millis(); // Reset the lockOwnerTimestamp
    }
}

/**
 * Public interface
 * Call this function whenever a button goes from UP to DOWN
 */
void onPhysicalButtonDown(int buttonId) {
    if (currentControllerState.lockOwner == PHYSICAL) {
        currentControllerState.lockOwnerTimestamp = millis();
        updateButtonState(buttonId, BUTTON_DOWN);
    }
}

/**
 * Public interface
 * Call this function whenever a button goes from DOWN to UP 
 */
void onPhysicalButtonUp(int buttonId) {
    if (currentControllerState.lockOwner == PHYSICAL) {
        currentControllerState.lockOwnerTimestamp = millis();
        updateButtonState(buttonId, BUTTON_UP);
    }
}

void onVirtualButtonDown(int buttonId) {
    if (currentControllerState.lockOwner == VIRTUAL) {
        currentControllerState.lockOwnerTimestamp = millis();
        updateButtonState(buttonId, BUTTON_DOWN);
    }
}

void onVirtualButtonUp(int buttonId) {
    if (currentControllerState.lockOwner == VIRTUAL) {
        currentControllerState.lockOwnerTimestamp = millis();
        updateButtonState(buttonId, BUTTON_UP);
    }
}

void handleButtonAction(int buttonId, ButtonStateEnum action) {
    bool buttonStateChanged = updateButtonState(buttonId, action);
    if (buttonId == 0) {
        handleButton0(action, buttonStateChanged);
    } else {
        handleMotorButtons(buttonId, action, buttonStateChanged);
    }
    updateLEDState();
}

void handleButton0(ButtonStateEnum action, bool buttonStateChanged) {
    if (action == BUTTON_DOWN) {
        // Ensure priorActionTime is set correctly for button 0
        if (currentButtonStates[0].priorButtonState == BUTTON_UP) {
            currentButtonStates[0].priorActionTime = millis();
        }
    } else {
        if (currentButtonStates[0].priorActionTime > 0 && millis() - currentButtonStates[0].priorActionTime <= button0HoldThreshold) {
            if (currentControllerState.controllerState == INACTIVE) {
                bool stateChanged = updateControllerState(ARMED);
                sendUARTMessage('A');
            } else if (currentControllerState.controllerState == ARMED) {
                updateControllerState(DISARMED);
                sendUARTMessage('D');
            } else if (currentControllerState.controllerState == DISARMED) {
                updateControllerState(INACTIVE);
                sendUARTMessage('I');
            }
        }
    }
    debug(stateToString(), DEBUG_LOW);
}

void handleMotorButtons(int buttonId, ButtonStateEnum action, bool buttonStateChanged) {
    if (currentControllerState.controllerState == ARMED && action == BUTTON_DOWN) {
        if (buttonId == 1) {
            moveMotor(1, "FORWARD");
        } else if (buttonId == 2) {
            moveMotor(1, "REVERSE");
        } else if (buttonId == 3) {
            moveMotor(2, "FORWARD");
        } else if (buttonId == 4) {
            moveMotor(2, "REVERSE");
        } else if (buttonId == 5) {
            moveMotor(3, "FORWARD");
        } else if (buttonId == 6) {
            moveMotor(3, "REVERSE");
        }
    }
}

void moveMotor(int motorId, const char* direction) {
    char command[20];
    sprintf(command, "MOTOR %d %s", motorId, direction);
    sendUARTMessage(command[0]);
}

// Returns a one line string showing the current Controller State and the current state of all of the buttons (i.e., UP or DOWN)
// For example: Disarmed, 0:UP, 1:UP, 2:UP, 3:UP, 4:UP, 5:UP, 6:UP
String stateToString() {
    String result;
    switch (currentControllerState.controllerState) {
        case INACTIVE: result = "Inactive"; break;
        case ARMED: result = "Armed"; break;
        case DISARMED: result = "Disarmed"; break;
        case BUTTON_0_STUCK: result = "Button_0 Stuck"; break;
        default: result = "Unknown"; break;
    }

    for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
        result += ", " + String(i) + ":" + (currentButtonStates[i].buttonState == BUTTON_UP ? "UP" : "DOWN");
    }

    return result;
}

/**
 * Updates the state of the given buttonId and returns true if its button state changed, else false 
 */
bool updateButtonState(int buttonId, ButtonStateEnum action) {
    unsigned long actionTime = millis();
    recentButtonChangeTime = actionTime;  //updates the global variable keeping track of the most recent button change
    ButtonState &button = currentButtonStates[buttonId];
    if (button.buttonState != action) {
        button.priorButtonState = button.buttonState;
        button.priorActionTime = button.actionTime;
        button.buttonState = action;
        button.actionTime = actionTime;
        debug("Button " + String(buttonId) + " state updated to " + (action == BUTTON_DOWN ? "DOWN" : "UP"), DEBUG_HIGH);
        return true;
    }
    return false;
}

// Initialize the UART port
void initUART(HardwareSerial &serial, long baudRate, int txPin, int rxPin) {
  serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
  if (currentDebugLevel >= DEBUG_LOW) debug("UART initialized", DEBUG_LOW);
}

// Send a character over UART (this sends the command to the Adapt Solutions black box)
void sendUARTMessage(char message) {
    if (uartSerialPort.availableForWrite()) {
        String formattedMessage = formatMessage(message);
        uartSerialPort.print(formattedMessage); // Use print instead of println to avoid adding a newline character
        if (currentDebugLevel >= DEBUG_HIGH) {
            String debugMessage = formattedMessage;
            debugMessage.replace("\r", "\\r");
            debug("Sent message: " + debugMessage, DEBUG_HIGH);
        }
    } else {
        debug("UART buffer full, message not sent", DEBUG_HIGH);
    }
}

// Function to format character messages per the required format of Adapt Solutions
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

    // Also initialize the button states to match the initial readings
    currentButtonStates[i] = {i, (initialReading == LOW) ? BUTTON_DOWN : BUTTON_UP, now, DEFAULT_PRIOR_BUTTON_STATE, now, debounceDelay};
  }
  debug("initializeButtonPins() configured button pins to INPUT_PULLUP", DEBUG_LOW);
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
        ButtonStateEnum state = (reading == LOW) ? BUTTON_DOWN : BUTTON_UP;
        currentButtonStates[i] = {i, state, now, state, now, debounceDelay};
        debouncers[i].lastReading = reading;
    }
    debug("initializePhysicalButtonStates() initialized physical button states", DEBUG_LOW);
}

void initializeVirtualButtonStates() {
    unsigned long now = millis();
    for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
        // Assuming virtual buttons might be initialized differently
        // Example: set all virtual buttons to BUTTON_UP by default
        currentButtonStates[i] = {i, BUTTON_UP, now, BUTTON_UP, now, debounceDelay};
        debouncers[i].lastReading = HIGH; // Default to HIGH (not pressed)
    }
    debug("initializeVirtualButtonStates() initialized virtual button states", DEBUG_LOW);
}

/**
 * Reads the state of each button and returns the ButtonState of the button that changed state most recently
 */
ButtonState checkButtons() {
    ButtonState recentButton = {-1, BUTTON_UP, 0, BUTTON_UP, 0, 0}; // Default to no recent button event

    switch (currentControllerState.lockOwner) {
        case PHYSICAL:
            recentButton = checkPhysicalButtons();
            break;
        case VIRTUAL:
            recentButton = checkVirtualButtons();
            break;
    }

    if (recentButton.buttonId == -1) {
        debug("Returning placeholder ButtonState object", DEBUG_LOW);
    }

    return recentButton;
}

ButtonState checkPhysicalButtons() {
    ButtonState recentButton = {-1, BUTTON_UP, 0, BUTTON_UP, 0, 0}; // Default to no recent button event

    for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
        int reading = digitalRead(buttonPins[i]);
        
        // Debug: Current reading of the button pin
        debug("Button " + String(i) + " reading: " + String(reading == LOW ? "DOWN" : "UP"), DEBUG_HIGH);
        
        if (reading != debouncers[i].lastReading) {
            debouncers[i].lastDebounceTime = millis();
        }
        
        if ((millis() - debouncers[i].lastDebounceTime) > debounceDelay) {
            ButtonStateEnum newState = (reading == LOW) ? BUTTON_DOWN : BUTTON_UP;
            if (updateButtonState(i, newState)) {
                if (currentButtonStates[i].actionTime > recentButton.actionTime) {
                    recentButton = currentButtonStates[i];
                }
            }
        }

        debouncers[i].lastReading = reading;
    }

    // Yield to prevent blocking
    yield();

    return recentButton;
}

ButtonState checkVirtualButtons() {
    ButtonState recentButton = {-1, BUTTON_UP, 0, BUTTON_UP, 0, 0}; // Default to no recent button event

    for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
        // Assuming the virtual button states are stored similarly to physical buttons
        if (buttonChanged(currentButtonStates[i])) {
            if (currentButtonStates[i].actionTime > recentButton.actionTime) {
                recentButton = currentButtonStates[i];
            }
        }
    }

    // Yield to prevent blocking
    yield();

    return recentButton;
}

void initializeLEDs() {
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  resetLEDs();
}

void resetLEDs() {
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(blueLEDPin, LOW);
}

void cycleLEDs() {
  // Fade red
  fadeLED(redLEDPin, 1000); // 1 second
  analogWrite(redLEDPin, 0);

  // Fade green
  fadeLED(greenLEDPin, 1000); // 1 second
  analogWrite(greenLEDPin, 0);

  // Fade blue
  fadeLED(blueLEDPin, 1000); // 1 second
  analogWrite(blueLEDPin, 0);
}

void fadeLED(int pin, int duration) {
  int stepDelay = 10; // milliseconds
  int steps = duration / stepDelay;
  for (int i = 0; i < steps; i++) {
    int duty = (255 * i) / steps;
    ledcWrite(pin, duty);
    delay(stepDelay);
  }
  for (int i = steps; i >= 0; i--) {
    int duty = (255 * i) / steps;
    ledcWrite(pin, duty);
    delay(stepDelay);
  }
}

void updateLEDState() {
    switch (currentControllerState.controllerState) {
        case INACTIVE:
            digitalWrite(greenLEDPin, LOW);
            digitalWrite(redLEDPin, LOW);
            digitalWrite(blueLEDPin, HIGH);
            break;
        case ARMED:
            digitalWrite(greenLEDPin, HIGH);
            digitalWrite(redLEDPin, LOW);
            digitalWrite(blueLEDPin, LOW);
            break;
        case DISARMED:
            digitalWrite(greenLEDPin, LOW);
            digitalWrite(redLEDPin, HIGH);
            digitalWrite(blueLEDPin, LOW);
            break;
        case BUTTON_0_STUCK:
            digitalWrite(greenLEDPin, LOW);
            digitalWrite(redLEDPin, HIGH);
            digitalWrite(blueLEDPin, HIGH);
            break;
    }
}

void debug(const String& msg, int level) {
    if (currentDebugLevel >= level) {
        Serial.println(msg);
    }
}

/**
 * Updates the currentControllerState and returns true if the state changed, else false 
 */
bool updateControllerState(ControllerStateEnum newState) {
    unsigned long now = millis();
    if (currentControllerState.controllerState != newState) {
        currentControllerState.priorControllerState = currentControllerState.controllerState;
        currentControllerState.priorStateTransitionTime = currentControllerState.stateTransitionTime;
        currentControllerState.controllerState = newState;
        currentControllerState.stateTransitionTime = now;
        debug("updateControllerState(): Controller state updated to: " + stateToString(), DEBUG_LOW);
        return true;
    }
    return false;
}

void initializeControllerState() {
    if (currentControllerState.lockOwner == PHYSICAL){
      // If Button 0 is being held down at startup, all the other controller buttons are inactivated
      if (currentButtonStates[0].buttonState == BUTTON_DOWN) {
          updateControllerState(INACTIVE);
      }
    } else {
      unsigned long now = millis();
      currentControllerState = {DEFAULT_CONTROLLER_STATE, now, DEFAULT_PRIOR_CONTROLLER_STATE, now, DEFAULT_LOCK_OWNER, now};
    }
    debug("initializeControllerState() initialized state to: " + stateToString(), DEBUG_LOW);
}

void handleControllerStateTransitions(ButtonState recentButton) {

    // The INACTIVE state only happens if Button 0 is DOWN when the controller is first turned on
    // Releasing Button 0 when in the INACTIVE state changes the state to ARMED
    if (currentControllerState.controllerState == INACTIVE){
      if (recentButton.buttonId == 0 && buttonChangedTo(recentButton, BUTTON_UP)){
          updateControllerState(ARMED);
          debug("1) Controller state updated from INACTIVE to ARMED: " + stateToString(), DEBUG_LOW);
          return;
      } else {
        // Only releasing Button 0 can get the controller working
        // All the other buttons are inactivated, so just return without changing the state
        debug("2) Controller state is still INACTIVE: " + stateToString(), DEBUG_LOW);
        return;
      }
    }

    // Check if Button 0 is stuck DOWN or if Button 0 was stuck and has just been released
    if (recentButton.buttonId == 0){
      if (buttonHeldDownFor(recentButton, button0HoldThreshold)) {
          updateControllerState(BUTTON_0_STUCK);
          debug("6) Controller state updated to BUTTON_0_STUCK: " + stateToString(), DEBUG_LOW);
          return;
      } else if (currentControllerState.controllerState == BUTTON_0_STUCK && buttonChangedTo(recentButton, BUTTON_UP)) {
          updateControllerState(ARMED);
          debug("7) Controller state updated from BUTTON_0_STUCK to ARMED: " + stateToString(), DEBUG_LOW);
          return;
      } else if(currentControllerState.controllerState == DISARMED && buttonChangedTo(recentButton, BUTTON_DOWN)){
          updateControllerState(ARMED);
          debug("8) Controller state updated from DISARMED to ARMED: " + stateToString(), DEBUG_LOW);
          return;
      }
    }

    if (currentControllerState.controllerState == ARMED){
      if (recentButton.buttonId == 0 && buttonHeldDownFor(recentButton, button0HoldThreshold)){
          updateControllerState(BUTTON_0_STUCK);
          debug("3) Controller state updated from ARMED to BUTTON_0_STUCK: " + stateToString(), DEBUG_LOW);
          return;
      }
    }

    if (currentControllerState.controllerState == DISARMED){
      if (recentButton.buttonId == 0 && buttonChanged(recentButton)){
          updateControllerState(ARMED);
          debug("4) Controller state updated from DISARMED to ARMED: " + stateToString(), DEBUG_LOW);
          return;
      }
    }

    // Check if we've been 
    if (currentControllerState.controllerState == ARMED){
      if (currentControllerState.stateTransitionTime > armedTimeout 
            && (millis() - recentButtonChangeTime) > armedTimeout){
          updateControllerState(DISARMED);
          debug("5) Controller state updated from ARMED to DISARMED (>timeout): " + stateToString(), DEBUG_LOW);
          return;
      }
    }

}

/** 
 * Returns true if the buttonState has changed since its priorButtonState, else false 
 */
bool buttonChanged(const ButtonState& button) {
    return (button.buttonState != button.priorButtonState
            && button.actionTime > button.priorActionTime);
}

/** 
 * Returns true if the buttonState has changed since its priorButtonState, else false 
 */
bool buttonChangedTo(const ButtonState& button, ButtonStateEnum newState) {
    return (button.buttonState == newState
            && button.priorButtonState != newState
            && button.actionTime > button.priorActionTime);
}

/** 
 * Returns true if the button is being held down, else false
 */
bool buttonHeldDown(const ButtonState& button) {
    return (button.buttonState == BUTTON_DOWN 
            && button.buttonState == button.priorButtonState 
            && button.actionTime > button.priorActionTime);
}

bool buttonHeldDownFor(const ButtonState& button, unsigned long timeInMillis) {
    return (button.buttonState == BUTTON_DOWN 
            && button.buttonState == button.priorButtonState 
            && (millis() - button.priorActionTime) > timeInMillis);
}

void handleSendCommands(ButtonState recentButton) {
    if (recentButton.buttonId != -1 && recentButton.buttonState == BUTTON_DOWN) {
        if (recentButton.buttonId == 0) {
            if (currentControllerState.controllerState == ARMED) {
                sendUARTMessage('A');
            } else if (currentControllerState.controllerState == DISARMED) {
                sendUARTMessage('D');
            } else if (currentControllerState.controllerState == INACTIVE) {
                sendUARTMessage('I');
            }
        } else {
            bool buttonStateChanged = updateButtonState(recentButton.buttonId, recentButton.buttonState);
            handleMotorButtons(recentButton.buttonId, recentButton.buttonState, buttonStateChanged);
        }
    }
}

void handleLEDs() {
    // Stub implementation: Add LED handling logic here
    switch (currentControllerState.controllerState) {
        case INACTIVE:
            // Example: Turn on the blue LED
            digitalWrite(greenLEDPin, LOW);
            digitalWrite(redLEDPin, LOW);
            digitalWrite(blueLEDPin, HIGH);
            break;
        case ARMED:
            // Example: Turn on the green LED
            digitalWrite(greenLEDPin, HIGH);
            digitalWrite(redLEDPin, LOW);
            digitalWrite(blueLEDPin, LOW);
            break;
        case DISARMED:
            // Example: Turn on the red LED
            digitalWrite(greenLEDPin, LOW);
            digitalWrite(redLEDPin, HIGH);
            digitalWrite(blueLEDPin, LOW);
            break;
        case BUTTON_0_STUCK:
            // Example: Turn on the red and blue LEDs
            digitalWrite(greenLEDPin, LOW);
            digitalWrite(redLEDPin, HIGH);
            digitalWrite(blueLEDPin, HIGH);
            break;
        default:
            // Example: Turn off all LEDs
            digitalWrite(greenLEDPin, LOW);
            digitalWrite(redLEDPin, LOW);
            digitalWrite(blueLEDPin, LOW);
            break;
    }
}

