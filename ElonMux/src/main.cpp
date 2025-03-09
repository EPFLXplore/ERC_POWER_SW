#include "BQ25756E.h"
#include <Arduino.h> // not needed if using BQ25756E.h
#include <Wire.h> // not needed if using BQ25756E.h
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// BQ25756E Configuration Parameters - Hardware dependent
#define BQ25756E_ADDRESS 0x6A
#define SWITCHING_FREQUENCY 600
#define MAX_CHARGE_CURRENT 10000
#define MAX_INPUT_CURRENT 20000
#define MIN_INPUT_VOLTAGE 4200
#define MAX_INPUT_VOLTAGE 36000

// ESP32-S3 GPIO pin definitions
#define IMON1_PIN 1
#define IMON2_PIN 2
#define SCL_PIN 4
#define SDA_PIN 5
#define SPI_NCS 6
#define SPI_MOSI 7
#define SPI_MISO 8
#define SPI_CLK 9
#define VALID_2 10
#define VALID_1 11
#define PRIORITY 12
#define PGOOD 13
#define CHARGER_INTERRUP 14
#define LED_DEBUG 21
#define FAULT2 33
#define FAULT1 34
#define GPIO35_SDA 35
#define GPIO36 36
#define GPIO37_SCL 37
#define PGn 38

// Display definitions for a 0.96" OLED (usually 128x64)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1  // Reset pin is not used
#define OLED_ADDRESS 0x3C // 7-bit I2C address for the OLED display (0x78 or 0x7A for 8-bit) 

// Function declarations
void setupGPIOs();
// void printByteAsBinary(uint8_t value);
// void print2BytesAsBinary(uint16_t value);
void printRegisters();
void updateDisplay();
void getCurrents();

// Creat a BQ25756E object
BQ25756E charger(BQ25756E_ADDRESS, SWITCHING_FREQUENCY, MAX_CHARGE_CURRENT, MAX_INPUT_CURRENT, MIN_INPUT_VOLTAGE, MAX_INPUT_VOLTAGE);

// Create a BQ25756E configuration structure
BQ25756E_Config chargerConfig = {
    .chargeVoltageLimit = 1524,
    .chargeCurrentLimit = 5000,
    .inputCurrentDPMLimit = 7000,
    .inputVoltageDPMLimit = 11000,
    .prechargeCurrentLimit = 500,
    .terminationCurrentLimit = 500,
    .terminationControlEnabled = true,
    .fastChargeThreshold = 0b11, // 71.4% x VFB_REG
    .prechargeControlEnabled = true,
    .enableMPPT = false,
    .verbose = true
};

// Create a second I2C instance; '1' selects the second hardware I2C controller (Used for the OLED display)
TwoWire myWire(1);

// Create an SSD1306 display object using myWire
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &myWire, OLED_RESET);

// Global variables for power path currents
float BAT_CURRENT = 0.0;
float SUPPLY_CURRENT = 0.0;
// ADC parameters for current sense
const float ADC_REF_VOLTAGE = 3.3;    // Reference voltage in volts
const int ADC_RESOLUTION = 4095;      // 12-bit ADC: 0-4095 counts
const float ADC_VOLTAGE_PER_COUNT = ADC_REF_VOLTAGE / ADC_RESOLUTION;
const float AMPLIFIER_GAIN = 20.0;      // The amplifier multiplies the sense voltage by 20

// Non-blocking LED blinking variables
unsigned long previousMillis = 0;
const unsigned long interval = 2000; // interval in milliseconds
bool ledState = LOW;

Stream* console = nullptr;

// Helper function to attempt USBSerial first, then fallback to hardware Serial:
void initializeConsole(unsigned long baudRate = 115200, unsigned long timeoutMs = 2000) {
    // First try initializing USBSerial
    USBSerial.begin(baudRate);

    unsigned long start = millis();
    while (!USBSerial && (millis() - start) < timeoutMs) {
        // waiting briefly to see if the USB CDC becomes ready
    }
    if (USBSerial) {
        // If the USB CDC is up, use USBSerial as the console
        console = &USBSerial;
        console->println("Using USB CDC for console");
    } else {
        // Otherwise, fall back to hardware Serial
        Serial.begin(baudRate);
        console = &Serial;
        console->println("Using hardware Serial for console");
    }
    delay(3000); // Allow time for the Serial port to initialize
}

void setup() {
  initializeConsole(); // Decide which Serial to use

  // Initialize the I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);
  myWire.begin(GPIO35_SDA, GPIO37_SCL);

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    console->println("SSD1306 allocation failed");
    while(1);
  } else {
    console->println("SSD1306 allocation successful");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Configure GPIO pins
  setupGPIOs();

  console->println("\nInitializing BQ25756E charger...\n");

  // Initialize the charger with the configuration structure
  charger.setDebugStream(console); // Let the charger's library know which stream to use for debug
  charger.init(chargerConfig);
  console->println("\nBQ25756E charger initialized successfully!\n");
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();

  // Non-blocking LED blinking
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_DEBUG, ledState);
    getCurrents();
    updateDisplay();
  }

  charger.printChargerConfig(false);
  // console->print("Charge voltage limit before: ");
  // console->println(charger.getChargeVoltageLimit());
  // charger.setChargeVoltageLimit(1536);
  // console->print("Charge voltage limit after: ");
  // console->println(charger.getChargeVoltageLimit());
  // charger.setChargeVoltageLimit(1512);
  // console->print("Pin configuration before: ");
  // charger.printByteAsBinary(charger.getPinControl());
  // charger.enablePins(1, 1, 1, 1);
  
  //printRegisters();
  delay(10000);

  // console->print("Pin configuration after: ");
  // charger.printByteAsBinary(charger.getPinControl());
  // charger.enablePins(0, 0, 0, 0);

  //delay(5000);
}

// Function to setup GPIO pins
void setupGPIOs() {
  // Debug LED
  pinMode(LED_DEBUG, OUTPUT);
  // LTC3126 GPIOs
  pinMode(VALID_1, INPUT);
  pinMode(VALID_2, INPUT);
  pinMode(PGOOD, INPUT);
  pinMode(PRIORITY, INPUT);
  // LTC7000 GPIOs
  pinMode(FAULT1, INPUT);
  pinMode(FAULT2, INPUT);
  pinMode(IMON1_PIN, INPUT);
  pinMode(IMON2_PIN, INPUT);
  // BQ25756E GPIOs
  pinMode(PGn, INPUT);
  pinMode(CHARGER_INTERRUP, INPUT);
}

// // Function to print a byte as binary (8 bits)
// void printByteAsBinary(uint8_t value) {
//   for (int i = 7; i >= 0; i--) {
//       Serial.print((value >> i) & 1);
//   }
//   Serial.println();
// }

// // Function to print two bytes as binary (16 bits)
// void print2BytesAsBinary(uint16_t value) {
//   for (int i = 15; i >= 0; i--) {
//       Serial.print((value >> i) & 1);
//   }
//   Serial.println();
// }

void printRegisters() {

  console->print("Charge voltage limit: ");
  console->print(charger.getChargeVoltageLimit());
  console->println(" mV");
  // console->print("Charge current limit: ");
  // console->print(charger.getChargeCurrentLimit());
  // console->println(" mA");
  // console->print("Input current DPM limit: ");
  // console->print(charger.getInputCurrentDPMLimit());
  // console->println(" mA");
  // console->print("Input voltage DPM limit: ");
  // console->print(charger.getInputVoltageDPMLimit());
  // console->println(" mV");
  // console->print("Reverse mode input current limit: ");
  // console->print(charger.getReverseModeInputCurrentLimit());
  // console->println(" mA");
  // console->print("Reverse mode input voltage limit: ");
  // console->print(charger.getReverseModeInputVoltageLimit());
  // console->println(" mV");
  console->println();
}

void updateDisplay() {
  // Update OLED display content
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ElonMux V1.0");
  display.println();
  // display.print("PGOOD: ");
  // display.println(digitalRead(PGOOD) ? "HIGH" : "LOW");
  display.println(digitalRead(PRIORITY) ? "MCU POWERED BY USB" : "MCU POWERED BY VOUT");
  // display.print("VALID1: ");
  // display.println(digitalRead(VALID_1) ? "HIGH" : "LOW");
  // display.print("VALID2: ");
  // display.println(digitalRead(VALID_2) ? "HIGH" : "LOW");
  if (!digitalRead(FAULT1) && digitalRead(FAULT2)) {
    display.println("SUPPLY OUT");
  } else if (digitalRead(FAULT1) && !digitalRead(FAULT2)) {
    display.println("BATTERY OUT");
  } else {
    display.println("NO OUTPUT");
  }
  display.print("SUPPLY curr: ");
  display.print(digitalRead(PRIORITY) ? 0.00 : SUPPLY_CURRENT);
  display.println("A");
  display.print("BATTER curr: ");
  display.print(digitalRead(PRIORITY) ? 0.00 : BAT_CURRENT );
  display.println("A");
  display.display();
}

void getCurrents() {
  // Read the raw ADC values from the two pins
  int rawSupply = analogRead(IMON1_PIN);
  int rawBattery = analogRead(IMON2_PIN);

  // Convert raw ADC counts to voltage at the ADC pin (in volts)
  float supplyVoltage = rawSupply * ADC_VOLTAGE_PER_COUNT;
  float batteryVoltage = rawBattery * ADC_VOLTAGE_PER_COUNT;

  // Convert the ADC voltage back to the original sense voltage
  float supplySenseVoltage = supplyVoltage / AMPLIFIER_GAIN;
  float batterySenseVoltage = batteryVoltage / AMPLIFIER_GAIN;

  // Convert the sense voltage from volts to millivolts
  supplySenseVoltage *= 1000;
  batterySenseVoltage *= 1000;

  // With 1 mV per 1A at the sense resistor, the sense voltage in mV equals the current in Amps
  SUPPLY_CURRENT = supplySenseVoltage;  // in Amps
  BAT_CURRENT = batterySenseVoltage;      // in Amps
}