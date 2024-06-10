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
constexpr int DEBUG_LOW = 1;
constexpr int DEBUG_HIGH = 2;
int debugLevel = DEBUG_LOW;

enum ControllerState {
    INACTIVE,
    ARMED,
    DISARMED,
    BUTTON_0_STUCK
};

enum ButtonStateEnum {
    UP,
    DOWN
};

struct ButtonState {
    int buttonId;
    ButtonStateEnum buttonState;
    unsigned long actionTime;
};

ControllerState currentState = INACTIVE;
unsigned long button0PressTime = 0;
const unsigned long button0HoldThreshold = 3000; // 3 seconds threshold
ButtonState currentButtonStates[7];

HardwareSerial mySerial(1);

// Function declarations
void onButtonDown(int buttonId);
void onButtonUp(int buttonId);
void handleButtonAction(int buttonId, ButtonStateEnum action);
void handleButton0(ButtonStateEnum action);
void handleMotorButtons(int buttonId, ButtonStateEnum action);
void moveMotor(int motorId, const char* direction);
void sendCommand(const char* command);
const char* stateToString(ControllerState state);
void updateButtonState(int buttonId, ButtonStateEnum action);
ButtonState getLastButtonAction();

void initUART(HardwareSerial &serial, long baudRate, int txPin, int rxPin);
void sendUARTMessage(char message);
String formatMessage(char inputChar);
void initializeButtons();
void initializeLEDs();
void cycleLEDs();
void fadeLED(int pin, int duration);

void setup() {
    Serial.begin(115200); // USB debugging
    initUART(mySerial, 9600, uartTxPin, uartRxPin); // Initialize UART communication with black box

    initializeButtons();
    initializeLEDs();

    // Initialize button states
    for (int i = 0; i < 7; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
        currentButtonStates[i] = (struct ButtonState){i, UP, millis()};
    }

    digitalWrite(greenLEDPin, LOW);
    digitalWrite(redLEDPin, LOW);
    digitalWrite(blueLEDPin, LOW);
}

void loop() {
    // Simulate button press events for testing
    for (int i = 0; i < 7; i++) {
        bool pressed = digitalRead(buttonPins[i]) == LOW; // Check button state
        if (pressed != (currentButtonStates[i].buttonState == DOWN)) {
            if (pressed) {
                onButtonDown(i);
            } else {
                onButtonUp(i);
            }
        }
    }
    delay(100); // Button check delay
}

void onButtonDown(int buttonId) {
    handleButtonAction(buttonId, DOWN);
}

void onButtonUp(int buttonId) {
    handleButtonAction(buttonId, UP);
}

void handleButtonAction(int buttonId, ButtonStateEnum action) {
    updateButtonState(buttonId, action);
    if (buttonId == 0) {
        handleButton0(action);
    } else {
        handleMotorButtons(buttonId, action);
    }
}

void handleButton0(ButtonStateEnum action) {
    if (action == DOWN) {
        if (millis() - button0PressTime > button0HoldThreshold) {
            currentState = BUTTON_0_STUCK;
        } else if (button0PressTime == 0) {
            button0PressTime = millis();
        }
    } else {
        if (button0PressTime > 0 && millis() - button0PressTime <= button0HoldThreshold) {
            if (currentState == INACTIVE) {
                currentState = ARMED;
                sendCommand("ARM");
            } else if (currentState == ARMED) {
                currentState = DISARMED;
                sendCommand("DISARM");
            } else if (currentState == DISARMED) {
                currentState = INACTIVE;
                sendCommand("INACTIVE");
            }
        }
        button0PressTime = 0;
    }
    Serial.println(stateToString(currentState));
}

void handleMotorButtons(int buttonId, ButtonStateEnum action) {
    if (currentState == ARMED && action == DOWN) {
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
    sendCommand(command);
}

void sendCommand(const char* command) {
    Serial.print("Sending command: ");
    Serial.println(command);
    mySerial.println(command); // Send command to black box via UART
}

const char* stateToString(ControllerState state) {
    switch (state) {
        case INACTIVE: return "Inactive";
        case ARMED: return "Armed";
        case DISARMED: return "Disarmed";
        case BUTTON_0_STUCK: return "Button_0 Stuck";
        default: return "Unknown";
    }
}

void updateButtonState(int buttonId, ButtonStateEnum action) {
    currentButtonStates[buttonId] = (struct ButtonState){buttonId, action, millis()};
}

ButtonState getLastButtonAction() {
    ButtonState latestAction = currentButtonStates[0];
    for (int i = 1; i < 7; i++) {
        if (currentButtonStates[i].actionTime > latestAction.actionTime) {
            latestAction = currentButtonStates[i];
        }
    }
    return latestAction;
}

// Initialize the UART port
void initUART(HardwareSerial &serial, long baudRate, int txPin, int rxPin) {
  serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
  if (debugLevel >= DEBUG_LOW) Serial.println("UART initialized");
}

// Send a character over UART (this sends the command to the Adapt Solutions black box)
void sendUARTMessage(char message) {
  String formattedMessage = formatMessage(message);
  mySerial.print(formattedMessage); // Use print instead of println to avoid adding a newline character
  if (debugLevel >= DEBUG_HIGH) {
    String debugMessage = formattedMessage;
    debugMessage.replace("\r", "\\r");
    Serial.printf("Sent message: %s\n", debugMessage.c_str());
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

// Initialize button pins and states
void initializeButtons() {
  for (int i = 0; i < 7; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    currentButtonStates[i] = (struct ButtonState){i, UP, millis()};
  }
  Serial.println("initializeButtons() configured button pins to INPUT_PULLUP");
}

void initializeLEDs() {
  ledcAttach(greenLEDPin, 5000, 8); // green LED, 5 kHz PWM, 8-bit resolution
  ledcAttach(redLEDPin, 5000, 8); // red LED, 5 kHz PWM, 8-bit resolution
  ledcAttach(blueLEDPin, 5000, 8); // blue LED, 5 kHz PWM, 8-bit resolution
}

void cycleLEDs() {
  // Fade red
  fadeLED(redLEDPin, 1000); // 1 second
  ledcWrite(1, 0);

  // Fade green
  fadeLED(greenLEDPin, 1000); // 1 second
  ledcWrite(0, 0);

  // Fade blue
  fadeLED(blueLEDPin, 1000); // 1 second
  ledcWrite(2, 0);
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
