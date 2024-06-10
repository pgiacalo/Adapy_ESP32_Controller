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

enum ControllerLockOwner {
    NONE,
    PHYSICAL,
    VIRTUAL
};

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

ControllerState currentControllerState = {DISARMED, 0, DISARMED, 0}; // Default to DISARMED
ButtonState currentButtonStates[7];

const unsigned long button0HoldThreshold = 8000; // 8 seconds threshold
const unsigned long debounceDelay = 50; // 50 milliseconds debounce delay
Debounce debouncers[7];

HardwareSerial uartSerialPort(1);

// Function declarations

void onPhysicalButtonDown(int buttonId);  //Interface
void onPhysicalButtonUp(int buttonId);    //Interface
void onVirtualButtonDown(int buttonId);   //Interface
void onVirtualButtonUp(int buttonId);     //Interface

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
    // Check if lockOwner has timed out
    if (currentControllerState.lockOwner != NONE && (millis() - currentControllerState.lockOwnerTimestamp > 8000)) {
        currentControllerState.lockOwner = NONE;
        debug("Lock owner timed out, reset to NONE", DEBUG_LOW);
    }

    ButtonState recentButton = checkButtons();

    // Check if Button 0 is stuck
    if (currentButtonStates[0].buttonState == BUTTON_DOWN &&
        (millis() - currentButtonStates[0].priorActionTime > button0HoldThreshold)) {
        updateControllerState(BUTTON_0_STUCK);
    } else if (currentControllerState.controllerState == BUTTON_0_STUCK && currentButtonStates[0].buttonState == BUTTON_UP) {
        updateControllerState(ARMED);
    } else if (currentControllerState.controllerState == INACTIVE) {
        cycleLEDs();
    }

    // Debug: Show most recent button event
    if (recentButton.buttonId != -1) {
        debug("Most recent button: " + String(recentButton.buttonId) + ", State: " + (recentButton.buttonState == BUTTON_DOWN ? "DOWN" : "UP"), DEBUG_HIGH);
    }

    if (recentButton.buttonId != -1 && recentButton.buttonState != recentButton.priorButtonState) {
        handleButtonAction(recentButton.buttonId, recentButton.buttonState);
    }

    updateLEDState();

    debug(stateToString(), DEBUG_LOW);
    delay(1000); // Button check delay
    yield();
}

/**
 * Interface
 */
void onPhysicalButtonDown(int buttonId) {
    if (currentControllerState.lockOwner == NONE || currentControllerState.lockOwner == PHYSICAL) {
        currentControllerState.lockOwner = PHYSICAL;
        currentControllerState.lockOwnerTimestamp = millis();
        handleButtonAction(buttonId, BUTTON_DOWN);
    }
}

/**
 * Interface
 */
void onPhysicalButtonUp(int buttonId) {
    if (currentControllerState.lockOwner == NONE || currentControllerState.lockOwner == PHYSICAL) {
        currentControllerState.lockOwner = PHYSICAL;
        currentControllerState.lockOwnerTimestamp = millis();
        handleButtonAction(buttonId, BUTTON_UP);
    }
}

/**
 * Interface
 */
void onVirtualButtonDown(int buttonId) {
    if (currentControllerState.lockOwner == NONE || currentControllerState.lockOwner == VIRTUAL) {
        currentControllerState.lockOwner = VIRTUAL;
        currentControllerState.lockOwnerTimestamp = millis();
        handleButtonAction(buttonId, BUTTON_DOWN);
    }
}

/**
 * Interface
 */
void onVirtualButtonUp(int buttonId) {
    if (currentControllerState.lockOwner == NONE || currentControllerState.lockOwner == VIRTUAL) {
        currentControllerState.lockOwner = VIRTUAL;
        currentControllerState.lockOwnerTimestamp = millis();
        handleButtonAction(buttonId, BUTTON_UP);
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

    for (int i = 0; i < 7; i++) {
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
  for (int i = 0; i < 7; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    int initialReading = digitalRead(buttonPins[i]);
    debouncers[i].lastDebounceTime = 0;
    debouncers[i].lastReading = initialReading;

    // Also initialize the button states to match the initial readings
    currentButtonStates[i] = {i, (initialReading == LOW) ? BUTTON_DOWN : BUTTON_UP, millis(), BUTTON_UP, 0, 0};
  }
  debug("initializeButtonPins() configured button pins to INPUT_PULLUP", DEBUG_LOW);
}

// Initialize button states
void initializeButtonStates() {
  for (int i = 0; i < 7; i++) {
    int reading = digitalRead(buttonPins[i]);
    ButtonStateEnum state = (reading == LOW) ? BUTTON_DOWN : BUTTON_UP;
    unsigned long currentTime = millis();
    currentButtonStates[i] = {i, state, currentTime, state, currentTime, debounceDelay};
    debouncers[i].lastReading = reading;
  }
  debug("initializeButtonStates() initialized button states", DEBUG_LOW);
}

/**
 * Reads the state of each button and returns the ButtonState of the button that changed state most recently
 */
ButtonState checkButtons() {
    ButtonState recentButton = {-1, BUTTON_UP, 0, BUTTON_UP, 0, 0}; // Default to no recent button event

    for (int i = 0; i < 7; i++) {
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
    if (currentControllerState.controllerState != newState) {
        currentControllerState.priorControllerState = currentControllerState.controllerState;
        currentControllerState.priorStateTransitionTime = currentControllerState.stateTransitionTime;
        currentControllerState.controllerState = newState;
        currentControllerState.stateTransitionTime = millis();
        debug("Controller state updated to: " + stateToString(), DEBUG_LOW);
        return true;
    }
    return false;
}

void initializeControllerState() {
    unsigned long currentTime = millis();
    currentControllerState = {DISARMED, currentTime, DISARMED, currentTime, NONE, 0};
    debug("initializeControllerState() initialized controller state", DEBUG_LOW);
}
