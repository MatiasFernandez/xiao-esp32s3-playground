/*
 * light_sleep_mode.cpp
 *
 * Minimal firmware used to measure the ESP32 power while using light sleep
 * for 200 ms windows. This mirrors the behavior of active_mode.cpp but uses
 * light-sleep instead of busy waiting / delay() so the difference in current
 * draw can be measured.
 *
 * Behavior per loop:
 *  - Set SIGNAL_PIN HIGH, enter light-sleep for 200 ms, wake
 *  - Set SIGNAL_PIN LOW, enter light-sleep for 200 ms, wake
 *
 * Radios are disabled in setup to prevent their activity from affecting the
 * baseline measurement.
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_sleep.h" // esp_light_sleep_start + wakeup APIs

#define SIGNAL_PIN D4

void setup() {
    // Disable WiFi radio
    WiFi.mode(WIFI_OFF);

    // Stop WiFi
    WiFi.disconnect(true);
    esp_wifi_stop();

    // Stop Bluetooth via Arduino helper
    btStop();

    pinMode(SIGNAL_PIN, OUTPUT);
}

void loop() {
    // HIGH for one light-sleep period
    digitalWrite(SIGNAL_PIN, HIGH);
    esp_sleep_enable_timer_wakeup(200000); // 200 ms in microseconds
    esp_light_sleep_start();

    // LOW for one light-sleep period
    digitalWrite(SIGNAL_PIN, LOW);
    esp_sleep_enable_timer_wakeup(200000); // 200 ms in microseconds
    esp_light_sleep_start();
}
