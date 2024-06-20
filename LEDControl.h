#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define GPIO pins for the RGB LED using standard pin numbers
#define RED_LED_PIN 13
#define GREEN_LED_PIN 12
#define BLUE_LED_PIN 14

// Enumerations for LED behavior and color
typedef enum {
    LED_BEHAVIOR_OFF,
    LED_BEHAVIOR_ON,
    LED_BEHAVIOR_BLINK,
    LED_BEHAVIOR_FAST_BLINK,
    LED_BEHAVIOR_SLOW_BLINK,
    LED_BEHAVIOR_CYCLE
} LedBehavior;

typedef enum {
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE
} LedColor;

// Shared variables
extern volatile LedBehavior ledBehavior;
extern volatile LedColor ledColor;

// Getter and setter functions for the intervals and brightness
#ifdef __cplusplus
extern "C" {
#endif

void controlLEDs(void *pvParameters);
void initializeLEDs();
void resetLEDs();

// Getters for intervals
int getBlinkInterval();
int getFastBlinkInterval();
int getSlowBlinkInterval();
int getCycleInterval();

// Setters for intervals
void setBlinkInterval(int interval);
void setFastBlinkInterval(int interval);
void setSlowBlinkInterval(int interval);
void setCycleInterval(int interval);

// Getters for brightness
int getRedLEDBrightness();
int getGreenLEDBrightness();
int getBlueLEDBrightness();

// Setters for brightness
void setRedLEDBrightness(int brightness);
void setGreenLEDBrightness(int brightness);
void setBlueLEDBrightness(int brightness);

#ifdef __cplusplus
}
#endif

#endif // LED_CONTROL_H
