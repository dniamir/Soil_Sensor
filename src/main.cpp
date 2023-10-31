#include <Wire.h>
#include <Arduino.h>

#define _TASK_SCHEDULING_OPTIONS
#include <TaskScheduler.h>

const int BME680_I2C_ADDRESS = 0b1110110;
const byte WHO_AM_I_REGISTER = 0xD0;

// Initialize I2C
int I2C_SDA = 43;
int I2C_SCL = 42;

// B6 is SCL
// B7 is SDA

// ADC Pin
const int ADC_PIN = PA0;  // Ensure that PB11 is defined as an ADC pin for your board's variant


// Declare and define the readRegister function before setup
byte readRegister(byte address, byte reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    byte error = Wire.endTransmission();

    // If there was an I2C error during transmission, reinitialize I2C and return 0xFF
    if (error) {
        Wire.flush(); // Clear the buffer
        Wire.end();   // End the I2C connection
        delay(10);    // Short delay (optional, for stabilization)
        Wire.begin(); // Reinitialize I2C
        return 0xFF;
    }

    Wire.requestFrom(address, (byte)1);
    if (Wire.available()) {
        return Wire.read();
    }

    return 0xFF; // Return 0xFF if there was an issue with the reading
}

// Define tasks for the TaskScheduler
void blinkLED();
void readSensor();
void readADC();

Scheduler runner;
// Task t1_blinkLED(1000, TASK_FOREVER, &blinkLED, &runner, true); // run every 1 second
// Task t2_readSensor(500, TASK_FOREVER, &readSensor, &runner, true); // run every 500ms

Task t1_blinkLED(1000, TASK_FOREVER, &blinkLED); // run every 1 second
Task t2_readSensor(500, TASK_FOREVER, &readSensor); // run every 500ms
Task t3_readADC(2000, TASK_FOREVER, &readADC); // run every 500ms

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();

  // ADC Initialization
  pinMode(ADC_PIN, INPUT_ANALOG);
  analogReadResolution(12);  // Set ADC resolution to 12 bits

  // Set the options for the task so that it "catches up" if there is a delay
  t1_blinkLED.setSchedulingOption(TASK_SCHEDULE);
  t2_readSensor.setSchedulingOption(TASK_SCHEDULE);
  t3_readADC.setSchedulingOption(TASK_SCHEDULE);

  // Start the scheduler
  runner.init();
  runner.addTask(t1_blinkLED);
  runner.addTask(t2_readSensor);
  runner.addTask(t3_readADC);
  t1_blinkLED.enable();
  t2_readSensor.enable();
  t3_readADC.enable();
}

void loop() {
  runner.execute(); // Run the scheduler to execute tasks
}

void blinkLED() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED state
  Serial.println("Toggling LED");
  Serial.println(" ");
}

void readSensor() {
  byte sensorID = readRegister(BME680_I2C_ADDRESS, WHO_AM_I_REGISTER);
  Serial.print("Sensor ID: 0x");
  Serial.println(sensorID, HEX);
  Serial.println(" ");
}

void readADC() {
  // Read and print ADC value
  int adcValue = analogRead(ADC_PIN);
  Serial.print("ADC Value: ");
  Serial.println(adcValue);
  Serial.println(" ");
}