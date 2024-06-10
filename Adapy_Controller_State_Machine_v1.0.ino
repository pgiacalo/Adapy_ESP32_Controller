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

struct ButtonState {
    int buttonId;
    ButtonStateEnum buttonState;
    unsigned long actionTime;
    ButtonStateEnum priorButtonState;
    unsigned long priorActionTime;
    unsigned long debounceTime;
};

struct Debounce {
    unsigned long lastDebounceTime;
    int lastReading;
};

ControllerStateEnum currentState = INACTIVE;
unsigned long button0PressTime = 0;
const unsigned long button0HoldThreshold = 3000; // 3 seconds threshold
const unsigned long debounceDelay = 50; // 50 milliseconds debounce delay
ButtonState currentButtonStates[7];
Debounce debouncers[7];

HardwareSerial uartSerialPort(1);

// Function declarations
void onButtonDown(int buttonId);
void onButtonUp(int buttonId);
void handleButtonAction(int buttonId, ButtonStateEnum action);
void handleButton0(ButtonStateEnum action);
void handleMotorButtons(int buttonId, ButtonStateEnum action);
void moveMotor(int motorId, const char* direction);
void sendUARTMessage(char message);
String stateToString();
void updateButtonState(int buttonId, ButtonStateEnum action);
ButtonState checkButtons();
void initializeButtonPins();
void initializeLEDs();
void cycleLEDs();
void fadeLED(int pin, int duration);
void updateLEDState();
void resetLEDs();
void debug(const String& msg, int level);

void setup() {
    Serial.begin(serialBaudRate); // Initialize console serial

    initUART(uartSerialPort, uartBaudRate, uartTxPin, uartRxPin); // Initialize UART communication with black box

    initializeButtonPins();

    checkButtons();

    initializeLEDs();
    resetLEDs();

    debug(stateToString(), DEBUG_LOW);
}

void loop() {
    ButtonState recentButton = checkButtons();
    if (recentButton.buttonId != -1 && recentButton.buttonState != recentButton.priorButtonState) {
        handleButtonAction(recentButton.buttonId, recentButton.buttonState);
    }
    debug(stateToString(), DEBUG_LOW);
    delay(1000); // Button check delay
    yield();
}

void onButtonDown(int buttonId) {
    handleButtonAction(buttonId, BUTTON_DOWN);
}

void onButtonUp(int buttonId) {
    handleButtonAction(buttonId, BUTTON_UP);
}

void handleButtonAction(int buttonId, ButtonStateEnum action) {
    updateButtonState(buttonId, action);
    if (buttonId == 0) {
        handleButton0(action);
    } else {
        handleMotorButtons(buttonId, action);
    }
    updateLEDState();
}

void handleButton0(ButtonStateEnum action) {
    if (action == BUTTON_DOWN) {
        if (millis() - button0PressTime > button0HoldThreshold) {
            currentState = BUTTON_0_STUCK;
        } else if (button0PressTime == 0) {
            button0PressTime = millis();
        }
    } else {
        if (button0PressTime > 0 && millis() - button0PressTime <= button0HoldThreshold) {
            if (currentState == INACTIVE) {
                currentState = ARMED;
                sendUARTMessage('A');
            } else if (currentState == ARMED) {
                currentState = DISARMED;
                sendUARTMessage('D');
            } else if (currentState == DISARMED) {
                currentState = INACTIVE;
                sendUARTMessage('I');
            }
        }
        button0PressTime = 0;
    }
    debug(stateToString(), DEBUG_LOW);
}

void handleMotorButtons(int buttonId, ButtonStateEnum action) {
    if (currentState == ARMED && action == BUTTON_DOWN) {
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

String stateToString() {
    String result;
    switch (currentState) {
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

void updateButtonState(int buttonId, ButtonStateEnum action) {
    ButtonState &button = currentButtonStates[buttonId];
    button.priorButtonState = button.buttonState;
    button.priorActionTime = button.actionTime;
    button.buttonState = action;
    button.actionTime = millis();
    debug("Button " + String(buttonId) + " state updated to " + (action == BUTTON_DOWN ? "DOWN" : "UP"), DEBUG_HIGH);
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

// Read the states of each button and populate currentButtonStates with debounce logic
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
            if (newState != currentButtonStates[i].buttonState) {
                currentButtonStates[i].priorButtonState = currentButtonStates[i].buttonState;
                currentButtonStates[i].priorActionTime = currentButtonStates[i].actionTime;
                currentButtonStates[i].buttonState = newState;
                currentButtonStates[i].actionTime = millis();
                
                // Debug: Button state change
                debug("Button " + String(i) + " state changed to: " + (currentButtonStates[i].buttonState == BUTTON_DOWN ? "DOWN" : "UP"), DEBUG_LOW);
                
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
    switch (currentState) {
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
