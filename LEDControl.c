#include "LEDControl.h"

// Shared variables
volatile LedBehavior ledBehavior = LED_BEHAVIOR_OFF;
volatile LedColor ledColor = LED_COLOR_RED;

// Brightness variables (0-255)
static int redLEDBrightness = 255;
static int greenLEDBrightness = 255;
static int blueLEDBrightness = 255;

// Interval variables
static int blinkInterval = 250;  // Blinking interval in milliseconds
static int fastBlinkInterval = 100;  // Blinking interval in milliseconds
static int slowBlinkInterval = 1000;  // Blinking interval in milliseconds
static int cycleInterval = 1000; // Cycle through 3 colors interval in milliseconds

void initializeLEDs() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    resetLEDs();
}

void resetLEDs() {
    analogWrite(RED_LED_PIN, 0);
    analogWrite(GREEN_LED_PIN, 0);
    analogWrite(BLUE_LED_PIN, 0);
}

void setRedLEDBrightness(int brightness) {
    redLEDBrightness = brightness;
}

void setGreenLEDBrightness(int brightness) {
    greenLEDBrightness = brightness;
}

void setBlueLEDBrightness(int brightness) {
    blueLEDBrightness = brightness;
}

int getRedLEDBrightness() {
    return redLEDBrightness;
}

int getGreenLEDBrightness() {
    return greenLEDBrightness;
}

int getBlueLEDBrightness() {
    return blueLEDBrightness;
}

void setBlinkInterval(int interval) {
    blinkInterval = interval;
}

void setFastBlinkInterval(int interval) {
    fastBlinkInterval = interval;
}

void setSlowBlinkInterval(int interval) {
    slowBlinkInterval = interval;
}

void setCycleInterval(int interval) {
    cycleInterval = interval;
}

int getBlinkInterval() {
    return blinkInterval;
}

int getFastBlinkInterval() {
    return fastBlinkInterval;
}

int getSlowBlinkInterval() {
    return slowBlinkInterval;
}

int getCycleInterval() {
    return cycleInterval;
}

void controlLEDs(void *pvParameters) {
    while (true) {
        switch (ledBehavior) {
            case LED_BEHAVIOR_OFF:
                resetLEDs();
                break;
            case LED_BEHAVIOR_ON:
                resetLEDs();
                if (ledColor == LED_COLOR_RED) {
                    analogWrite(RED_LED_PIN, redLEDBrightness);
                } else if (ledColor == LED_COLOR_GREEN) {
                    analogWrite(GREEN_LED_PIN, greenLEDBrightness);
                } else if (ledColor == LED_COLOR_BLUE) {
                    analogWrite(BLUE_LED_PIN, blueLEDBrightness);
                }
                break;
            case LED_BEHAVIOR_BLINK:
                resetLEDs();
                if (ledColor == LED_COLOR_RED) {
                    analogWrite(RED_LED_PIN, redLEDBrightness);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                    analogWrite(RED_LED_PIN, 0);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_GREEN) {
                    analogWrite(GREEN_LED_PIN, greenLEDBrightness);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                    analogWrite(GREEN_LED_PIN, 0);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_BLUE) {
                    analogWrite(BLUE_LED_PIN, blueLEDBrightness);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                    analogWrite(BLUE_LED_PIN, 0);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                }
                break;
            case LED_BEHAVIOR_FAST_BLINK:
                resetLEDs();
                if (ledColor == LED_COLOR_RED) {
                    analogWrite(RED_LED_PIN, redLEDBrightness);
                    vTaskDelay(fastBlinkInterval / portTICK_PERIOD_MS);
                    analogWrite(RED_LED_PIN, 0);
                    vTaskDelay(fastBlinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_GREEN) {
                    analogWrite(GREEN_LED_PIN, greenLEDBrightness);
                    vTaskDelay(fastBlinkInterval / portTICK_PERIOD_MS);
                    analogWrite(GREEN_LED_PIN, 0);
                    vTaskDelay(fastBlinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_BLUE) {
                    analogWrite(BLUE_LED_PIN, blueLEDBrightness);
                    vTaskDelay(fastBlinkInterval / portTICK_PERIOD_MS);
                    analogWrite(BLUE_LED_PIN, 0);
                    vTaskDelay(fastBlinkInterval / portTICK_PERIOD_MS);
                }
                break;
            case LED_BEHAVIOR_SLOW_BLINK:
                resetLEDs();
                if (ledColor == LED_COLOR_RED) {
                    analogWrite(RED_LED_PIN, redLEDBrightness);
                    vTaskDelay(slowBlinkInterval / portTICK_PERIOD_MS);
                    analogWrite(RED_LED_PIN, 0);
                    vTaskDelay(slowBlinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_GREEN) {
                    analogWrite(GREEN_LED_PIN, greenLEDBrightness);
                    vTaskDelay(slowBlinkInterval / portTICK_PERIOD_MS);
                    analogWrite(GREEN_LED_PIN, 0);
                    vTaskDelay(slowBlinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_BLUE) {
                    analogWrite(BLUE_LED_PIN, blueLEDBrightness);
                    vTaskDelay(slowBlinkInterval / portTICK_PERIOD_MS);
                    analogWrite(BLUE_LED_PIN, 0);
                    vTaskDelay(slowBlinkInterval / portTICK_PERIOD_MS);
                }
                break;
            case LED_BEHAVIOR_CYCLE:
                resetLEDs();
                analogWrite(RED_LED_PIN, redLEDBrightness);
                vTaskDelay(cycleInterval / portTICK_PERIOD_MS);
                resetLEDs();
                analogWrite(GREEN_LED_PIN, greenLEDBrightness);
                vTaskDelay(cycleInterval / portTICK_PERIOD_MS);
                resetLEDs();
                analogWrite(BLUE_LED_PIN, blueLEDBrightness);
                vTaskDelay(cycleInterval / portTICK_PERIOD_MS);
                break;
        }
        
        // Add a short delay to ensure the task yields control and resets the watchdog
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
