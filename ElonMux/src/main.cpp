#include "BQ25756E.h"
#include "LedManager.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// BQ25756E Configuration Parameters - Hardware dependent
#define BQ25756E_ADDRESS 0x6A
#define SWITCHING_FREQUENCY 600
#define MAX_CHARGE_CURRENT 10000
#define MAX_INPUT_CURRENT 20000
#define MIN_INPUT_VOLTAGE 4200
#define MAX_INPUT_VOLTAGE 36000

/* ------------------------ BQ25756E Configuration Parameters - Xplore imposed specifications ------------------------ */
// charge current constraints for pre-charge mode between 0V and 20.48V
#define MIN_CHARGE_CURRENT_PRE 250
#define DEFAULT_CHARGE_CURRENT_PRE 500
#define MAX_CHARGE_CURRENT_PRE 1000
#define CURRENT_STEP_PRE 50
// charge current constraints for CC mode between 20.48V and 23V
#define MIN_CHARGE_CURRENT_CC_20 400
#define DEFAULT_CHARGE_CURRENT_CC_20 1000
#define MAX_CHARGE_CURRENT_CC_20 1500
#define CURRENT_STEP_CC_20 50
// charge current constraints for CC mode between 23V and 24V
#define MIN_CHARGE_CURRENT_CC_23 1000
#define DEFAULT_CHARGE_CURRENT_CC_23 1500
#define MAX_CHARGE_CURRENT_CC_23 2000
#define CURRENT_STEP_CC_23 100
// charge current constraints for CC mode between 24V and 28.3V
#define MIN_CHARGE_CURRENT_CC_24 1500
#define DEFAULT_CHARGE_CURRENT_CC_24 2300
#define MAX_CHARGE_CURRENT_CC_24 5500
#define CURRENT_STEP_CC_24 100
// charge current constraints for CV mode at 28.3V
#define MIN_CHARGE_CURRENT_CV 250
#define DEFAULT_CHARGE_CURRENT_CV 500
#define MAX_CHARGE_CURRENT_CV 1050
#define CURRENT_STEP_CV 50

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
#define PLUS_BUTTON 36
#define GPIO37_SCL 37
#define PGn 38
#define U0TXD_AS_MINUS_BUTTON 43
#define U0RXD_AS_GPIO 44

// ADC channels parameters
#define MOVING_AVG_SIZE 10

// Display definitions for a 0.96" OLED (usually 128x64)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1  // Reset pin is not used
#define OLED_ADDRESS 0x3C // 7-bit I2C address for the OLED display (0x78 or 0x7A for 8-bit) 

// Function declarations
void initializeConsole(unsigned long baudRate = 115200, unsigned long timeoutMs = 1500);
void setupGPIOs();
void updateDisplay();
void getCurrents();
void updateLED();
void displayChargerStage();
void updateChargeState();
void displayElapsedTime();
void handleCurrentButtons();
void checkChargeCurrentConstraints();
void get_connectors_status();

// Creat a BQ25756E object
BQ25756E charger(BQ25756E_ADDRESS, SWITCHING_FREQUENCY, MAX_CHARGE_CURRENT, MAX_INPUT_CURRENT, MIN_INPUT_VOLTAGE, MAX_INPUT_VOLTAGE);

// Create a BQ25756E configuration structure  
BQ25756E_Config chargerConfig = {
    .chargeVoltageLimit = 1506, // Range: 1504mV (28.31V) to 1566 mV(29.48V)
    .chargeCurrentLimit = DEFAULT_CHARGE_CURRENT_CC_24, // Displayed charge current will be lower by around 10%, but the real current will be close to the set value. Range: 0.4A to 10A
    .inputCurrentDPMLimit = 7000, // Range: 0.4A to 20A
    .inputVoltageDPMLimit = 15000, // voltage in mV under which the charger will reduce the input current
    .prechargeCurrentLimit = DEFAULT_CHARGE_CURRENT_PRE, // Range: 0.25A to 10A
    .terminationCurrentLimit = DEFAULT_CHARGE_CURRENT_CV, // Range: 0.25A to 10A
    .terminationControlEnabled = true, // Enable termination current control
    .fastChargeThreshold = 0b11, // 0b00 = 30% x VFB_REG, 0b01 = 55% x VFB_REG, 0b10 = 66.7% x VFB_REG, 0b11 = 71.4% x VFB_REG = 71.4% x 1524 = 1088mV -> fast charge above 20.48V
    .prechargeControlEnabled = true, // Enable pre-charge and trickle charge functions
    .topOffTimer = 0b00, // 0b00 = Disable, 0b01 = 15 minutes, 0b10 = 30 minutes, 0b11 = 45 minutes
    .watchdogTimer = 0b00, // 0b00 = Disable, 0b01 = 40s, 0b10 = 80s, 0b11 = 160s
    .safetyTimerEnabled = false, // disabled
    .safetyTimer = 0b00, // 0b00 = 5h, 0b01 = 8h, 0b10 = 12h, 0b11 = 24h
    .safetyTimerSpeed = false,
    .constantVoltageTimer = 0b0010, // 0b0000 = disable, 0b0001 = 1h, 0b0010 = 2h, ... 0b1111 = 15h
    .autoRechargeThreshold = 0b11, // 0b00 = 93%, 0b01 = 94.3%, 0b10 = 95.2%, 0b11 = 97.6%
    .watchdogTimerResetEnabled = false, 
    .CEPinEnabled = true, // Enable the control of the charger with the switch connected to the CE pin
    .ChargeBehaviorWatchdogExpired = true, // 0b = EN_CHG resets to 0, 1b = EN_CHG resets to 1 when watchdog expires
    .highZModeEnabled = false,
    .batteryLoadEnabled = false, // Disable battery load
    .chargeEnabled = true, // Enable charging
    .enableMPPT = false,
    .verbose = true
};

// Create a second I2C instance; '1' selects the second hardware I2C controller (Used for the OLED display)
TwoWire myWire(1);

// Create an SSD1306 display object using myWire
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &myWire, OLED_RESET);

// Create a LedManager object
LedManager led;

/* ---------------------------------- Global variables ---------------------------------- */
// Actual PCB connections
bool supply_connected = false;
bool battery_discharge_connected = false;
bool no_outputs = false;
// Global variables for power path currents
float BAT_CURRENT = 0.0;
float SUPPLY_CURRENT = 0.0;
// ADC parameters for current sense
const float ADC_REF_VOLTAGE = 3.3;    // Reference voltage in volts
const int ADC_RESOLUTION = 4095;      // 12-bit ADC: 0-4095 counts
const float ADC_VOLTAGE_PER_COUNT = ADC_REF_VOLTAGE / ADC_RESOLUTION;
const float AMPLIFIER_GAIN = 20.0;      // The amplifier multiplies the sense voltage by 20 + 4.2 because offset

// Non-blocking LED variables
uint8_t led_state = 0;
unsigned long previousMillis = 0;
unsigned long ledMillis = 0;
const unsigned long interval = 200; // interval in milliseconds
// Charger state
enum ChargeStates : uint8_t {
  NOT_CHA      = 0b000,
  TRICKLE      = 0b001,
  PRECHARGE    = 0b010,
  CC           = 0b011,
  CV           = 0b100,
  RSVD         = 0b101,
  TOPOFF       = 0b110,
  DONE         = 0b111
};

const char* const ChargeStateStrings[] = {
  "NOT_CHA",   // 0b000
  "TRICKLE",   // 0b001
  "PRECHARGE", // 0b010
  "CC",        // 0b011
  "CV",        // 0b100
  "RSVD",      // 0b101
  "TOPOFF",    // 0b110
  "DONE"       // 0b111
};
uint8_t chargeState = 0;
uint8_t previousChargeState = 0;
bool chargerisinFault = false;
unsigned long chargeStartTime = 0;  // Records when the charge cycle starts
bool charging = false;   

Stream* console = nullptr;

void setup() {
  initializeConsole(); // Decide which Serial to use

  // Initialize the two I2C bus
  Wire.begin(SDA_PIN, SCL_PIN); // Default I2C bus for the charger
  myWire.begin(GPIO35_SDA, GPIO37_SCL); // Second I2C bus for the OLED display

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) { console->println("Failed to initialize SSD1306"); } 
  else { console->println("SSD1306 initialized successfully!"); }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Configure GPIO pins
  setupGPIOs();
  // Initialize the RTOS LED
  led.begin();
  led.startBlinkForDuration(200, 200, 1400); // Blink for 1.4 seconds to indicate MCU boot

  console->println("\nInitializing BQ25756E charger...\n");
  charger.setDebugStream(console); // Let the charger's library know which stream to use for debug
  charger.init(chargerConfig); // Initialize the charger with the configuration structure
  previousChargeState = charger.getChargeCycleStatus(); // Get the initial charge state
  console->println("\nBQ25756E charger initialized successfully!\n");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { // Update every 'interval' milliseconds (400ms)
    previousMillis = currentMillis;
    chargeState = charger.getChargeCycleStatus();
    get_connectors_status();
    if ((charger.getVBATADC() > 20000 && charger.getVBATADC() < 29000) && (chargeState != NOT_CHA) && supply_connected) {checkChargeCurrentConstraints();}
    if (supply_connected) { 
      handleCurrentButtons();
      //getCurrents();
      updateChargeState();  
    }
    if ((previousChargeState != chargeState) && supply_connected) {
      updateLED();
      console->print("New Charge state: ");
      console->println(charger.getChargeCycleStatus());
      previousChargeState = chargeState;
    }
    if (led_state != chargeState) {updateLED();}
    //charger.printChargerConfig(false);
    updateDisplay();
  }
}

// Helper function to attempt USBSerial first, then fallback to hardware Serial:
void initializeConsole(unsigned long baudRate, unsigned long timeoutMs) {
  // First try initializing USBSerial
  //delay(1500); // Uncomment to allow time for the serial monitor of PlatformIO to connect and see the first messages
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
}

void setupGPIOs() {
  // ESP32-S3 GPIOs
  pinMode(LED_DEBUG, OUTPUT);
  pinMode(PLUS_BUTTON, INPUT_PULLUP);
  pinMode(U0TXD_AS_MINUS_BUTTON, INPUT_PULLUP);
  // LTC3126 GPIOs
  pinMode(VALID_1, INPUT);
  pinMode(VALID_2, INPUT);
  pinMode(PGOOD, INPUT);
  pinMode(PRIORITY, INPUT);
  // LTC7000 GPIOs
  pinMode(FAULT1, INPUT);
  pinMode(FAULT2, INPUT);
  analogReadResolution(12);  // Ensure 12-bit resolution
  // analogSetPinAttenuation(IMON1_PIN, ADC_0db);
  // analogSetPinAttenuation(IMON2_PIN, ADC_0db);
  // adcAttachPin(IMON1_PIN);
  // adcAttachPin(IMON2_PIN);
  pinMode(IMON1_PIN, INPUT);
  pinMode(IMON2_PIN, INPUT);
  // BQ25756E GPIOs
  pinMode(PGn, INPUT);
  pinMode(CHARGER_INTERRUP, INPUT);
  // Attach interrupt to the INT pin, trigger on FALLING edge
  // attachInterrupt(digitalPinToInterrupt(CHARGER_INTERRUP), handleFaultInterrupt, FALLING);
}

void updateDisplay() {
  // Update OLED display content
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ElonMux V1.0");
  display.println();
  if (supply_connected && !battery_discharge_connected) {
    display.println("SUPPLY OUT");
  } else if (!supply_connected && battery_discharge_connected) {
    display.println("BATTERY OUT");
  } else if (no_outputs) {
    display.println("NO OUTPUT - USB PWRD");
  }
  display.print("Charge state: ");
  displayChargerStage();
  if (charger.getVBATADC() > 20000 && !digitalRead(PGn)) {
    display.print("VBAT: ");
    float vbatValue = charger.getVBATADC() / 1000.0;
    display.print(vbatValue, 3);
    display.println("V");
  } else {
    display.println("NO BATTERY ON CHARGER");
  }
  if (chargeState != 0b000) {
    display.print("CHARGE curr: ");
    float chargeCurrent = charger.getIBATADC() / 1000.0;
    display.print(chargeCurrent, 3);
    display.println("A");
    display.print("INPUT  curr: ");
    float inputCurrent = charger.getIACADC() / 1000.0;
    display.print(inputCurrent, 3);
    display.println("A");
  } else {
    display.print("SET CHG. curr: ");
    if (supply_connected) {
      display.print(static_cast<float>(charger.getChargeCurrentLimit()) / 1000.0);
      display.println("A");
    } else {
      display.print(static_cast<float>(DEFAULT_CHARGE_CURRENT_CC_24) / 1000.0);
      display.println("A");
    }
    display.print("SET target V: ");
    display.print((((static_cast<float>(chargerConfig.chargeVoltageLimit) * 249000.0) / 13967.0) + chargerConfig.chargeVoltageLimit)/1000);
    display.println("V");
  }
  if (digitalRead(PGn)) {
    display.print("VSUPPLY TOO LOW");
    chargerisinFault = true;
  } else {
    chargerisinFault = false;
  }
  displayElapsedTime();
  display.display();
}

float compensateCurrent(float measured) {
  // Coefficients determined from calibration:
  const float a = 1.1417;  
  const float b = -6.3583; 
  const float c = 11.786;   
  return a * measured * measured + b * measured + c;
}

void getCurrents() {
  // Static buffers and variables to store readings and track the index and count
  static float supplyBuffer[MOVING_AVG_SIZE] = {0};
  static float batteryBuffer[MOVING_AVG_SIZE] = {0};
  static int bufferIndex = 0;
  static int sampleCount = 0;

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

  // Now, at this point your supplySenseVoltage (in mV) represents your uncorrected current reading
  // since you have 1 mV per 1 A. 
  // Instead of applying a fixed multiplier, you’ll use your calibration function.

  // Store the new readings into the buffers (moving average)
  supplyBuffer[bufferIndex] = supplySenseVoltage;
  batteryBuffer[bufferIndex] = batterySenseVoltage;

  // Update index and sample count
  bufferIndex = (bufferIndex + 1) % MOVING_AVG_SIZE;
  if (sampleCount < MOVING_AVG_SIZE) {
    sampleCount++;
  }

  // Calculate the moving average for each channel
  float supplySum = 0, batterySum = 0;
  for (int i = 0; i < sampleCount; i++) {
    supplySum += supplyBuffer[i];
    batterySum += batteryBuffer[i];
  }
  
  // With 1 mV per 1A at the sense resistor, the average sense voltage in mV equals the uncorrected current in Amps
  float supplyMeasured = supplySum / sampleCount;
  float batteryMeasured = batterySum / sampleCount;

  // Apply the non-linearity compensation via the calibration function
  if (supplyMeasured > 2.5) {
    SUPPLY_CURRENT = compensateCurrent(supplyMeasured);
  } else {
    SUPPLY_CURRENT = supplyMeasured;
  } 
  if (batteryMeasured > 2.5) {
    BAT_CURRENT = compensateCurrent(batteryMeasured);
  } else {
    BAT_CURRENT = batteryMeasured;
  }
}


void updateLED() {
  // if no supply or only battery is connected, turn off the LED
  if ((battery_discharge_connected || no_outputs) && (led_state != 0)) {
    led.LEDOFF();
    console->println("LED OFF");
    led_state = 0;
    return;
  }
  switch (previousChargeState) {
    case PRECHARGE: // Precharge
      led_state = 2;
      led.startFading(3000);
      break;
    case CC: // Constant Current
      led_state = 3;
      led.startFading(1000);
      break;
    case CV: // Constant Voltage
      led_state = 4;
      led.startFading(500);
      break;
    case DONE: // Done
      led_state = 7;
      led.LEDON();
      break;
    default: // Other states
      led_state = 0;
      led.LEDOFF();
      break;
  }
}

void displayChargerStage() {
  if (previousChargeState <= DONE) {
    display.println(ChargeStateStrings[previousChargeState]);
  } else {
    display.println("UNKNOWN");
  }
}

void updateChargeState() {
  // If the charger state is no longer "NOT_CHA" and we haven't started timing yet
  if (chargeState != NOT_CHA && !charging) {
    charging = true;
    chargeStartTime = millis();  // Start timer when the charge cycle begins
  }
  // Stop the timer when charging is complete (either not charging or done charging)
  if ((chargeState == NOT_CHA || chargeState == DONE) && charging) {
    charging = false;
  }
}

void displayElapsedTime() {
  if (charging) {
    unsigned long elapsedMillis = millis() - chargeStartTime;
    unsigned long totalSeconds = elapsedMillis / 1000;
    unsigned int seconds = totalSeconds % 60;
    unsigned int minutes = (totalSeconds / 60) % 60;
    unsigned int hours   = totalSeconds / 3600;
    display.print("Charge time: ");
    display.printf("%02u:%02u:%02u", hours, minutes, seconds);
  }
}

void handleCurrentButtons() {
  bool plusPressed  = !digitalRead(PLUS_BUTTON);
  bool minusPressed = !digitalRead(U0TXD_AS_MINUS_BUTTON);
  uint16_t actual_current_limit = charger.getChargeCurrentLimit();
  switch (chargeState) {
    // Not charging and Constant Current share the same behavior
    case NOT_CHA:
      if (plusPressed && actual_current_limit < MAX_CHARGE_CURRENT_CC_24) {
        console->println(actual_current_limit + CURRENT_STEP_CC_24);
        charger.setChargeCurrentLimit(actual_current_limit + CURRENT_STEP_CC_24);
      } else if (minusPressed && actual_current_limit > (MIN_CHARGE_CURRENT_CC_24)) {
        charger.setChargeCurrentLimit(actual_current_limit - CURRENT_STEP_CC_24);
      }
      break;
    case CC:
      if (plusPressed && (charger.getVBATADC() < 23000) && actual_current_limit < MAX_CHARGE_CURRENT_CC_20) {
        charger.setChargeCurrentLimit(actual_current_limit + CURRENT_STEP_CC_20);
      } else if (minusPressed && charger.getVBATADC() < 23000 && actual_current_limit > MIN_CHARGE_CURRENT_CC_20) {
        charger.setChargeCurrentLimit(charger.getChargeCurrentLimit() - CURRENT_STEP_CC_20);
      } else if (plusPressed && charger.getVBATADC() < 24000 && charger.getChargeCurrentLimit() < MAX_CHARGE_CURRENT_CC_23) {
        charger.setChargeCurrentLimit(charger.getChargeCurrentLimit() + CURRENT_STEP_CC_23);
      } else if (minusPressed && charger.getVBATADC() < 24000 && charger.getChargeCurrentLimit() > MIN_CHARGE_CURRENT_CC_23) {
        charger.setChargeCurrentLimit(charger.getChargeCurrentLimit() - CURRENT_STEP_CC_23);
      } else if (plusPressed && charger.getChargeCurrentLimit() < MAX_CHARGE_CURRENT_CC_24) {
        charger.setChargeCurrentLimit(charger.getChargeCurrentLimit() + CURRENT_STEP_CC_24);
      } else if (minusPressed && charger.getChargeCurrentLimit() > MIN_CHARGE_CURRENT_CC_24) {
        charger.setChargeCurrentLimit(charger.getChargeCurrentLimit() - CURRENT_STEP_CC_24);
      }
      break;
      
    case PRECHARGE: // Precharge
      if (plusPressed)
        charger.setPrechargeCurrentLimit(chargerConfig.prechargeCurrentLimit + CURRENT_STEP_PRE);
      else if (minusPressed)
        charger.setPrechargeCurrentLimit(chargerConfig.prechargeCurrentLimit - CURRENT_STEP_PRE);
      break;

    case CV: // Constant Voltage
      if (plusPressed)
        charger.setTerminationCurrentLimit(chargerConfig.terminationCurrentLimit + CURRENT_STEP_CV);
      else if (minusPressed)
        charger.setTerminationCurrentLimit(chargerConfig.terminationCurrentLimit - CURRENT_STEP_CV);
      break;

    default:
      break;
  }
}

void checkChargeCurrentConstraints() {
  uint16_t vbat = charger.getVBATADC();
  if (vbat < 23000 && (charger.getChargeCurrentLimit() >= (MAX_CHARGE_CURRENT_CC_20+CURRENT_STEP_CC_20/2) || charger.getChargeCurrentLimit() <= (MIN_CHARGE_CURRENT_CC_20-CURRENT_STEP_CC_20/2))) {
    charger.setChargeCurrentLimit(DEFAULT_CHARGE_CURRENT_CC_20);
  } else if (vbat < 24000 && (charger.getChargeCurrentLimit() >= (MAX_CHARGE_CURRENT_CC_23+CURRENT_STEP_CC_23/2) || charger.getChargeCurrentLimit() <= (MIN_CHARGE_CURRENT_CC_23-CURRENT_STEP_CC_23/2))) {
    charger.setChargeCurrentLimit(DEFAULT_CHARGE_CURRENT_CC_23);
  } else if ((24000 < vbat) && (vbat < 29000) && (charger.getChargeCurrentLimit() >= (MAX_CHARGE_CURRENT_CC_24+CURRENT_STEP_CC_24/2) || charger.getChargeCurrentLimit() <= (MIN_CHARGE_CURRENT_CC_24-CURRENT_STEP_CC_24/2))) {
    charger.setChargeCurrentLimit(DEFAULT_CHARGE_CURRENT_CC_24); 
  }
}

void get_connectors_status() {
  // Static variable to hold the previous state of supply_connected
  static bool prev_supply_connected = false;

  // Read current state from FAULT1
  bool current_supply_connected = !digitalRead(FAULT1);
  
  // Check for rising edge: previously false, now true
  if (current_supply_connected && !prev_supply_connected) {
    charger.init(chargerConfig);
  }
  // Update the supply_connected state and previous state
  supply_connected = current_supply_connected;
  prev_supply_connected = current_supply_connected;

  // Set battery_discharge_connected based on FAULT2 reading
  battery_discharge_connected = !digitalRead(FAULT2);
  
  // Determine no_outputs state based on both FAULT1 and FAULT2
  no_outputs = (digitalRead(FAULT1) && digitalRead(FAULT2));
}