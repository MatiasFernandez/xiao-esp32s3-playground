#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>

#define LED_ON LOW
#define LED_OFF HIGH

#define ONE_WIRE_BUS D0
#define ONE_WIRE_BUS_ENABLE D1

// Battery measurement pin and ADC settings
#define BAT_LEVEL D2

// ADC conversion constants
#define ADC_BITS 12
#define ADC_MAX ((1 << ADC_BITS) - 1) // 4095 for 12-bit
// Voltage reference used for ADC -> best-effort. For ESP32 this is approximate; for better accuracy
// use calibration APIs. We're using 3.3V as Vref for the calculation below.
#define ADC_VREF 3.30f

// Voltage divider: top resistor (between VBAT and ADC pin) = R_TOP
//                 bottom resistor (between ADC pin and GND) = R_BOTTOM
// According to the design: R_TOP = 220k, R_BOTTOM = 470k
// (i.e. a 220k top resistor and a 470k bottom resistor)
#define R_TOP 220000.0f
#define R_BOTTOM 470000.0f

// Oversampling: number of readings to average for each loop
#define OVERSAMPLE_COUNT 100

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long boot_time;

void enableOneWirePower() {
  Serial.println("Enabling power to 1-Wire bus...");

  digitalWrite(ONE_WIRE_BUS_ENABLE, HIGH);

  delay(10);
}

void disableOneWirePower() {
  Serial.println("Disabling power to 1-Wire bus...");

  digitalWrite(ONE_WIRE_BUS_ENABLE, LOW);
}

void logElapsedTime(String where, String step) {
  Serial.print("[");
  Serial.print(where);
  Serial.print("] - ");
  Serial.print(step);
  Serial.print(" - time: +");
  float durationMs = (micros() - boot_time) / 1000.0; // convert µs to ms with fraction
  Serial.print(durationMs, 3);
  Serial.println(" ms");
}

void setup() {
  boot_time = micros();

  WiFi.mode(WIFI_OFF);

  delay(500);

  Serial.begin(115200);

  logElapsedTime("setup", "Serial initialized");

  Serial.print("Boot time: ");
  Serial.println(boot_time);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ONE_WIRE_BUS_ENABLE, OUTPUT);
  pinMode(BAT_LEVEL, INPUT);

  // Configure ADC attenuation for ESP32: use 11 dB to extend measurable input range (~ up to Vref)
  // Use ESP32 Arduino core attenuation constant
  analogSetPinAttenuation(BAT_LEVEL, ADC_11db);

  // Set ADC resolution (ESP32 default is 12 bits -> 0..4095)
  analogReadResolution(ADC_BITS);

  logElapsedTime("setup", "ADC initialized");

  enableOneWirePower();

  sensors.begin();

  logElapsedTime("setup", "Sensors initialized");

  // Set DS18B20 resolution (9..12 bits) for the first sensor
  DeviceAddress address;

  if (sensors.getAddress(address, 0)) {
    sensors.setResolution(address, 9); // set to 9-bit (valid: 9,10,11,12)
    Serial.println("Sensor resolution set to 9 bits");
  }

  logElapsedTime("setup", "Sensor resolution set");

  disableOneWirePower();

  logElapsedTime("setup", "Setup completed");
}

void loop() {
  logElapsedTime("loop", "Starting loop");

  float adcSingle = analogRead(BAT_LEVEL);

  logElapsedTime("loop", "ADC read");

  // --- ADC: Measure battery level (oversampled) ---
  // Take OVERSAMPLE_COUNT analog readings and average them to reduce ADC variance
  unsigned long adcSum = 0;
  for (int i = 0; i < OVERSAMPLE_COUNT; ++i) {
    adcSum += (unsigned long)analogRead(BAT_LEVEL);
  }

  logElapsedTime("loop", "Oversampling complete");

  float adcAvg = (float)adcSum / (float)OVERSAMPLE_COUNT; // averaged raw ADC value
  float vAdc = (adcAvg / (float)ADC_MAX) * ADC_VREF; // voltage at ADC pin

  // Account for voltage divider (R_TOP between Vbat and ADC, R_BOTTOM between ADC and GND)
  // Vbat = Vadc * (R_TOP + R_BOTTOM) / R_BOTTOM
  float vBatt = vAdc * ((R_TOP + R_BOTTOM) / R_BOTTOM);

  Serial.print("BATT: ADC single raw: ");
  Serial.print(adcSingle, 2);
  Serial.print(" - ADC avg raw: ");
  Serial.print(adcAvg, 2);
  Serial.print(" - V_adc: ");
  Serial.print(vAdc, 3);
  Serial.print("V - V_bat: ");
  Serial.print(vBatt, 3);
  Serial.println("V");

  logElapsedTime("loop", "Enabling OneWire power");

  enableOneWirePower();

  logElapsedTime("loop", "Requesting temperature");
  unsigned long t0 = micros();               // start timer (use millis() if you prefer ms precision)
  sensors.requestTemperatures(); // send conversion command
  unsigned long t1 = micros();               // end timer
  
  logElapsedTime("loop", "Temperature Conversion complete");

  float durationMs = (t1 - t0) / 1000.0;     // convert µs to ms with fraction
  Serial.print("Conversion took ");
  Serial.print(durationMs, 3);
  Serial.println(" ms");

  logElapsedTime("loop", "Reading temperature");

  float tempC = sensors.getTempCByIndex(0); // read first sensor

  logElapsedTime("loop", "Temperature read");

  disableOneWirePower();

  logElapsedTime("loop", "OneWire power disabled");


  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Sensor DS18B20 desconectado");
  } else {
    Serial.print("Temperatura: ");
    Serial.print(tempC);
    Serial.println(" °C");
  }

  logElapsedTime("loop", "Turning ON LED");

  digitalWrite(LED_BUILTIN, LED_OFF);

  delay(200);

  logElapsedTime("loop", "Turning OFF LED");

  digitalWrite(LED_BUILTIN, LED_OFF);

  delay(200);

  logElapsedTime("loop", "Loop completed");
}

