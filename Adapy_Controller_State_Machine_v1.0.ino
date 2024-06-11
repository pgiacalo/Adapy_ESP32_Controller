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
constexpr ButtonStateEnum DEFAULT_PRIOR_BUTTON_STATE = BUTTON_UP;

// Default values for ControllerState
constexpr ControllerStateEnum DEFAULT_CONTROLLER_STATE = DISARMED;
constexpr ControllerStateEnum DEFAULT_PRIOR_CONTROLLER_STATE = DISARMED;
constexpr ControllerLockOwner DEFAULT_LOCK_OWNER = PHYSICAL;

ControllerState currentControllerState;

const unsigned int NUMBER_OF_BUTTONS = 7; 
ButtonState currentButtonStates[NUMBER_OF_BUTTONS];

constexpr unsigned int messageInterval = 340;  // milliseconds, the time between message resends if a button is held down (measured from lastMessageTime)
const unsigned int button0HoldThreshold = 8000; // 8 seconds threshold
const unsigned int debounceDelay = 50; // 50 milliseconds debounce delay
Debounce debouncers[NUMBER_OF_BUTTONS];

HardwareSerial uartSerialPort(1);

// Function declarations
void onPhysicalButtonDown(int buttonId);          //public interface
void onPhysicalButtonUp(int buttonId);            //public interface
void onVirtualButtonDown(int buttonId);           //public interface
void onVirtualButtonUp(int buttonId);             //public interface
void setLockOwner(ControllerLockOwner newOwner);  //public interface

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
void checkLockOwner();
void handleControllerStateTransitions(ButtonState recentButton);
void handleSendCommands(ButtonState recentButton);
void handleLEDs();

void setup() {
    Serial.begin(serialBaudRate); // Initialize console serial

    debug("setup() called", DEBUG_LOW);

    initUART(uartSerialPort, uartBaudRate, uartTxPin, uartRxPin); // Initialize UART communication with black box

    initializeControllerState(); // Initialize controller state
    initializeButtonPins();
    initializeButtonStates(); // Initialize button states

    // Check the state of button 0 after initializing button states
    bool stateChanged = false;
    if (currentButtonStates[0].buttonState == BUTTON_DOWN) {
        stateChanged = updateControllerState(INACTIVE);
        if (stateChanged){
          //TODO - what behavior 
        }
    }

    initializeLEDs();
    resetLEDs();

    cycleLEDs();

    debug(stateToString(), DEBUG_LOW);
}

void loop() {
    checkLockOwner();

    ButtonState recentButton = checkButtons();

    handleControllerStateTransitions(recentButton);

    handleSendCommands(recentButton);

    handleLEDs();

    debug(stateToString(), DEBUG_LOW);

    delay(1000); // Adjust delay as necessary
    yield();
}

void setLockOwner(ControllerLockOwner newOwner) {
    currentControllerState.lockOwner = newOwner;
    currentControllerState.lockOwnerTimestamp = millis();
}

void checkLockOwner() {
    // Check if lockOwner has timed out
    if (millis() - currentControllerState.lockOwnerTimestamp > 8000) {
        setLockOwner(PHYSICAL);
        debug("Lock owner timed out, reset to PHYSICAL", DEBUG_LOW);
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
  unsigned long now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    int reading = digitalRead(buttonPins[i]);
    ButtonStateEnum state = (reading == LOW) ? BUTTON_DOWN : BUTTON_UP;
    currentButtonStates[i] = {i, state, now, state, now, debounceDelay};
    debouncers[i].lastReading = reading;
  }
  debug("initializeButtonStates() initialized button states", DEBUG_LOW);
}

/**
 * Reads the state of each button and returns the ButtonState of the button that changed state most recently
 */
ButtonState checkButtons() {
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

void initializeLEDs() {
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
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
        debug("Controller state updated to: " + stateToString(), DEBUG_LOW);
        return true;
    }
    return false;
}

void initializeControllerState() {
    unsigned long now = millis();
    currentControllerState = {DEFAULT_CONTROLLER_STATE, now, DEFAULT_PRIOR_CONTROLLER_STATE, now, DEFAULT_LOCK_OWNER, now};
    debug("initializeControllerState() initialized controller state", DEBUG_LOW);
}

void handleControllerStateTransitions(ButtonState recentButton) {
    // Check if Button 0 is stuck
    if (currentButtonStates[0].buttonState == BUTTON_DOWN &&
        (millis() - currentButtonStates[0].priorActionTime > button0HoldThreshold)) {
        updateControllerState(BUTTON_0_STUCK);
    } else if (currentControllerState.controllerState == BUTTON_0_STUCK && currentButtonStates[0].buttonState == BUTTON_UP) {
        updateControllerState(ARMED);
    }

    // Handle transitions based on recent button action
    if (recentButton.buttonId == 0 && recentButton.buttonState == BUTTON_UP) {
        if (currentButtonStates[0].priorActionTime > 0 && millis() - currentButtonStates[0].priorActionTime <= button0HoldThreshold) {
            if (currentControllerState.controllerState == INACTIVE) {
                updateControllerState(ARMED);
            } else if (currentControllerState.controllerState == ARMED) {
                updateControllerState(DISARMED);
            } else if (currentControllerState.controllerState == DISARMED) {
                updateControllerState(INACTIVE);
            }
        }
    }
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


