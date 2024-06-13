#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Shared variables defined here
volatile LedBehavior ledBehavior = LED_BEHAVIOR_OFF;
volatile LedColor ledColor = LED_COLOR_RED;

// GPIO pins for the RGB LED
constexpr int redLEDPin = 13;
constexpr int greenLEDPin = 12;
constexpr int blueLEDPin = 14;

void controlLEDs(void *pvParameters) {
    int blinkInterval = 250;  // Blinking interval in milliseconds
    int flashDuration = 100;  // Flash duration in milliseconds
    int cycleInterval = 1000; // Cycle interval in milliseconds

    while (true) {
        switch (ledBehavior) {
            case LED_BEHAVIOR_OFF:
                digitalWrite(redLEDPin, LOW);
                digitalWrite(greenLEDPin, LOW);
                digitalWrite(blueLEDPin, LOW);
                break;
            case LED_BEHAVIOR_ON:
                if (ledColor == LED_COLOR_RED) {
                    digitalWrite(redLEDPin, HIGH);
                    digitalWrite(greenLEDPin, LOW);
                    digitalWrite(blueLEDPin, LOW);
                } else if (ledColor == LED_COLOR_GREEN) {
                    digitalWrite(redLEDPin, LOW);
                    digitalWrite(greenLEDPin, HIGH);
                    digitalWrite(blueLEDPin, LOW);
                } else if (ledColor == LED_COLOR_BLUE) {
                    digitalWrite(redLEDPin, LOW);
                    digitalWrite(greenLEDPin, LOW);
                    digitalWrite(blueLEDPin, HIGH);
                }
                break;
            case LED_BEHAVIOR_BLINK:
                if (ledColor == LED_COLOR_RED) {
                    digitalWrite(redLEDPin, HIGH);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                    digitalWrite(redLEDPin, LOW);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_GREEN) {
                    digitalWrite(greenLEDPin, HIGH);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                    digitalWrite(greenLEDPin, LOW);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                } else if (ledColor == LED_COLOR_BLUE) {
                    digitalWrite(blueLEDPin, HIGH);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                    digitalWrite(blueLEDPin, LOW);
                    vTaskDelay(blinkInterval / portTICK_PERIOD_MS);
                }
                break;
            case LED_BEHAVIOR_FLASH:
                if (ledColor == LED_COLOR_RED) {
                    digitalWrite(redLEDPin, HIGH);
                    vTaskDelay(flashDuration / portTICK_PERIOD_MS);
                    digitalWrite(redLEDPin, LOW);
                } else if (ledColor == LED_COLOR_GREEN) {
                    digitalWrite(greenLEDPin, HIGH);
                    vTaskDelay(flashDuration / portTICK_PERIOD_MS);
                    digitalWrite(greenLEDPin, LOW);
                } else if (ledColor == LED_COLOR_BLUE) {
                    digitalWrite(blueLEDPin, HIGH);
                    vTaskDelay(flashDuration / portTICK_PERIOD_MS);
                    digitalWrite(blueLEDPin, LOW);
                }
                vTaskDelay(blinkInterval / portTICK_PERIOD_MS); // Wait for the next flash
                break;
            case LED_BEHAVIOR_CYCLE:
                digitalWrite(redLEDPin, HIGH);
                digitalWrite(greenLEDPin, LOW);
                digitalWrite(blueLEDPin, LOW);
                vTaskDelay(cycleInterval / portTICK_PERIOD_MS);
                digitalWrite(redLEDPin, LOW);
                digitalWrite(greenLEDPin, HIGH);
                digitalWrite(blueLEDPin, LOW);
                vTaskDelay(cycleInterval / portTICK_PERIOD_MS);
                digitalWrite(redLEDPin, LOW);
                digitalWrite(greenLEDPin, LOW);
                digitalWrite(blueLEDPin, HIGH);
                vTaskDelay(cycleInterval / portTICK_PERIOD_MS);
                break;
        }
        
        // Add a short delay to ensure the task yields control and resets the watchdog
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
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
