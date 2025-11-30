/*
 * active_mode.cpp
 *
 * Minimal firmware used to measure baseline power consumption of the ESP32
 * in active mode with radios disabled.  Intent and behavior:
 *  - Purpose: measure the MCU's active current draw while WiFi and Bluetooth
 *    are turned off (so radios don't affect the measurement).
 *  - This file intentionally contains minimal functionality so measurements
 *    represent the chip/activity itself (it toggles SIGNAL_PIN for scope).
 * 
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#define SIGNAL_PIN D4

void setup() {
    // Disable WiFi radio
    WiFi.mode(WIFI_OFF);

    // Stop WiFi and Bluetooth using Arduino-style helpers (safer and simpler)
    WiFi.disconnect(true);
    esp_wifi_stop();

    // Stop the controller + stack via the Arduino wrapper
    // (btStop() is the recommended convenience helper when using Arduino/PlatformIO)
    btStop();

    // Minimal I/O so we can observe status with a scope / LED
    pinMode(SIGNAL_PIN, OUTPUT);
}

void loop() {
    digitalWrite(SIGNAL_PIN, HIGH);

    delay(200);

    digitalWrite(SIGNAL_PIN, LOW);

    delay(200);
}
