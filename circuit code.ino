#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


// Define I2C pins for BME280 (SDA, SCL)
Adafruit_BME280 bme;  // I2C

// Define MQ-2 analog pin
const int mq2Pin = A3;
const float RL = 10.0;      // Load resistance in kilo-ohms (based on the datasheet)
const int cleanAirValue = 9.83; // Rs/R0 ratio in clean air for MQ2
const int TEMP_PIN = 9;
const int HUM_PIN = 8;
// Variables
float R0;                  // Initial calibration resistance value in clean air
float Rs;                  // Variable for sensor resistance
float ppmCH4;              // Calculated ppm of methane (example gas)
void calibrateMQ2() {
  long sum = 0;
  for (int i = 0; i < 50; i++) {  // Read 50 times to average and stabilize
    int sensorValue = analogRead(mq2Pin);
    float sensorVoltage = sensorValue * (5.0 / 1023.0); // Voltage reading
    Rs = (5.0 - sensorVoltage) / sensorVoltage * RL;   // Calculate Rs
    sum += Rs;
    delay(100);   // Short delay between readings
  }
  R0 = sum / 50.0 / cleanAirValue;  // Average Rs and calculate R0
}
float calculatePPM(float Rs) {
  float ratio = Rs / R0;  // Rs/R0 ratio
  // Using equation for Methane (CH4) from MQ2 datasheet approximation
  // ppm = 10 ^ ((log10(ratio) - b) / m), here using typical methane values
  float m = -0.168;
  float b = 1.3;
  return pow(10, ((log10(ratio) - b) / m));
}
// Set timing interval (30 seconds = 30000 milliseconds)
const unsigned long interval = 10000;
unsigned long previousMillis = 0;
unsigned long firstMillis = 0;
void setup() {
  Serial.begin(115200);

  //  Initialize BME280
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Initialize MQ-2
  pinMode(mq2Pin, INPUT);
  pinMode(TEMP_PIN, OUTPUT);
  pinMode(HUM_PIN, OUTPUT);
  calibrateMQ2();
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if 30 seconds have passed
  if (currentMillis - previousMillis >= interval) {
    if (!firstMillis) {
      firstMillis = currentMillis;
    }
    previousMillis = currentMillis;

    // Read temperature and humidity from BME280
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    int sensorValue = analogRead(mq2Pin);
    float sensorVoltage = sensorValue * (5.0 / 1023.0); // Voltage reading
    Rs = (5.0 - sensorVoltage) / sensorVoltage * RL;    // Calculate Rs
    ppmCH4 = calculatePPM(Rs);

    if (temperature > 27.0)
      digitalWrite(TEMP_PIN, HIGH);
    else
      digitalWrite(TEMP_PIN, LOW);
    if (humidity > 70.0)
      digitalWrite(HUM_PIN, HIGH);
    else
      digitalWrite(HUM_PIN, LOW);

    String DashboardString = "{\"Timestamp\":" + String(currentMillis - firstMillis) + ",\"Temperature\":" + String(temperature) + ",\"Humidity\":" + String(humidity) + ",\"Smoke\":" + String(ppmCH4) + "}";
    Serial.println(DashboardString);
  }
}
