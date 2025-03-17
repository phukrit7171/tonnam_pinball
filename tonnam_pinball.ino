#include <SoftwareSerial.h>
#include "RedMP3.h"

// Pin definitions
const int SCORE_SENSOR = A0;                    // Single analog pin for all scoring sensors (parallel circuit)
const int SCORE_LEDS[] = { 2, 3, 4, 5, 6 };     // 5 LEDs for score indication
const int SONG_SENSORS[] = { A1, A2, A3, A4 };  // 4 sensors for different songs
const int MOTOR_LIGHT_SENSOR = A5;              // Sensor for motor and lights
const int MARBLE_SENSOR = 12;                   // Changed to digital pin 12 to avoid conflict with A5

// Output pins
const int MOTOR_PIN = 7;           // Motor control pin
const int LIGHTS_PIN = 8;          // Lights control pin (parallel connected)
const int MARBLE_EJECTOR_PIN = 9;  // Marble ejector control pin

// MP3 Player setup
#define MP3_RX 10  // RX of Serial MP3 module connect to pin 10
#define MP3_TX 11  // TX to pin 11
MP3 mp3(MP3_RX, MP3_TX);

// Thresholds and variables
const int SENSOR_THRESHOLD = 500;     // Analog threshold for sensor detection
const int DETECTION_DEBOUNCE = 1000;  // Debounce time in milliseconds
int scoreCount = 0;                   // Counter for score detection
unsigned long lastScoreTime = 0;      // Time of last score detection

// Song file numbers - RedMP3 requires folder and file structure
// Assuming songs are in folder 01 with file names 001.wav, 002.wav, etc.
const int FOLDER_NUMBER = 0x01;                          // Folder "01"
const int SCORE_SONG[2] = { 0x01, 0x02 };                // score songs (001.wav, 002.wav)
const int UNIQUE_SONGS[4] = { 0x03, 0x04, 0x05, 0x06 };  // Songs 003.wav to 006.wav

// Volume level (0-30)
const int VOLUME_LEVEL = 0x17;  // Volume level 22

// Timing variables for non-blocking operation
unsigned long lastScoreSensorCheck = 0;
unsigned long lastSongSensorCheck = 0;
unsigned long lastMotorSensorCheck = 0;
unsigned long lastMarbleSensorCheck = 0;

// State variables
bool motorActive = false;
bool marbleEjectorActive = false;
unsigned long motorStartTime = 0;
unsigned long marbleStartTime = 0;
const unsigned long MOTOR_ON_DURATION = 3000;   // 3 seconds
const unsigned long MARBLE_ON_DURATION = 1000;  // 1 second

// Previous sensor states for edge detection
bool prevScoreSensorState = false;
bool prevMotorSensorState = false;
bool prevMarbleSensorState = false;
bool prevSongSensorStates[4] = { false, false, false, false };

void setup() {
  // Initialize serial communications
  Serial.begin(9600);

  // Initialize MP3 player volume
  delay(500);  // Required 500ms delay for MP3 module initialization
  mp3.setVolume(VOLUME_LEVEL);
  delay(50);  // Wait between commands
  mp3.stopPlay();
  delay(50);  // Wait between commands

  // Initialize LED pins as outputs
  for (int i = 0; i < 5; i++) {
    pinMode(SCORE_LEDS[i], OUTPUT);
    digitalWrite(SCORE_LEDS[i], LOW);
  }

  // Initialize output pins
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LIGHTS_PIN, OUTPUT);
  pinMode(MARBLE_EJECTOR_PIN, OUTPUT);

  // Initialize marble sensor pin as input
  pinMode(MARBLE_SENSOR, INPUT);

  // Turn off all outputs initially
  // For low-level trigger relays, HIGH means OFF
  digitalWrite(MOTOR_PIN, HIGH);
  digitalWrite(LIGHTS_PIN, LOW); // Lights are still using high-level trigger
  digitalWrite(MARBLE_EJECTOR_PIN, HIGH);

  Serial.println("System initialized for Arduino UNO with low-level trigger relays");
}

void loop() {
  unsigned long currentMillis = millis();

  // Check score sensor (every 50ms)
  if (currentMillis - lastScoreSensorCheck >= 50) {
    checkScoreSensor();
    lastScoreSensorCheck = currentMillis;
  }

  // Check song sensors (every 50ms)
  if (currentMillis - lastSongSensorCheck >= 50) {
    checkSongSensors();
    lastSongSensorCheck = currentMillis;
  }

  // Check motor and lights sensor (every 50ms)
  if (currentMillis - lastMotorSensorCheck >= 50) {
    checkMotorLightsSensor();
    lastMotorSensorCheck = currentMillis;
  }

  // Check marble ejector sensor (every 50ms)
  if (currentMillis - lastMarbleSensorCheck >= 50) {
    checkMarbleEjectorSensor();
    lastMarbleSensorCheck = currentMillis;
  }

  // Handle timed events (motor and marble ejector)
  handleTimedEvents(currentMillis);
}

void checkScoreSensor() {
  // Read the parallel circuit of score sensors
  bool currentState = analogRead(SCORE_SENSOR) > SENSOR_THRESHOLD;

  // Detect rising edge (sensor just activated)
  if (currentState && !prevScoreSensorState) {
    // Add debounce for multiple rapid detections
    if (millis() - lastScoreTime > DETECTION_DEBOUNCE) {
      // Increment score
      scoreCount++;
      lastScoreTime = millis();

      // Update LED indicators
      updateScoreLEDs();

      // Debug print
      Serial.print("Score increased: ");
      Serial.println(scoreCount);

      // Check if we've reached 5 or 10 detections
      if (scoreCount == 5) {
        // Play song 001.wav from folder 01 at the first 5 detections
        mp3.playWithFileName(FOLDER_NUMBER, SCORE_SONG[0]);  // Fixed: using array index
        Serial.println("Playing score song 1 (001.wav)");
        delay(50);  // Wait between commands
      } else if (scoreCount == 10) {
        // Play song 002.wav from folder 01 at the second 5 detections
        mp3.playWithFileName(FOLDER_NUMBER, SCORE_SONG[1]);  // Fixed: using array index
        Serial.println("Playing score song 2 (002.wav)");
        delay(50);  // Wait between commands

        // Reset score count
        scoreCount = 0;

        // Turn off all LEDs
        for (int j = 0; j < 5; j++) {
          digitalWrite(SCORE_LEDS[j], LOW);
        }
      }
    }
  }

  // Update previous state
  prevScoreSensorState = currentState;
}

void updateScoreLEDs() {
  // Calculate how many LEDs should be on based on score count
  int ledsOn = min(scoreCount, 5);

  // Update LED states
  for (int i = 0; i < 5; i++) {
    if (i < ledsOn) {
      digitalWrite(SCORE_LEDS[i], HIGH);
    } else {
      digitalWrite(SCORE_LEDS[i], LOW);
    }
  }
}

void checkSongSensors() {
  // Check all four song sensors
  for (int i = 0; i < 4; i++) {
    bool currentState = analogRead(SONG_SENSORS[i]) > SENSOR_THRESHOLD;

    // Detect rising edge (sensor just activated)
    if (currentState && !prevSongSensorStates[i]) {
      // Play the corresponding unique song (003.wav to 006.wav) from folder 01
      mp3.playWithFileName(FOLDER_NUMBER, UNIQUE_SONGS[i]);

      // Debug print
      Serial.print("Playing unique song ");
      Serial.print(UNIQUE_SONGS[i]);
      Serial.println(".wav");

      delay(50);  // Wait between commands
    }

    // Update previous state
    prevSongSensorStates[i] = currentState;
  }
}

void checkMotorLightsSensor() {
  bool currentState = analogRead(MOTOR_LIGHT_SENSOR) > SENSOR_THRESHOLD;

  // Detect rising edge (sensor just activated)
  if (currentState && !prevMotorSensorState && !motorActive) {
    // Activate motor and lights
    digitalWrite(MOTOR_PIN, LOW);   // LOW turns on the low-level trigger relay
    digitalWrite(LIGHTS_PIN, HIGH); // HIGH turns on the lights (assuming high-level trigger)
    motorActive = true;
    motorStartTime = millis();

    // Debug print
    Serial.println("Motor and lights activated");
  }

  // Update previous state
  prevMotorSensorState = currentState;
}

void checkMarbleEjectorSensor() {
  // Changed to digitalRead since we moved to a digital pin
  bool currentState = digitalRead(MARBLE_SENSOR) == HIGH;

  // Detect rising edge (sensor just activated)
  if (currentState && !prevMarbleSensorState && !marbleEjectorActive) {
    // Activate marble ejector
    digitalWrite(MARBLE_EJECTOR_PIN, LOW);  // LOW turns on the low-level trigger relay
    marbleEjectorActive = true;
    marbleStartTime = millis();

    // Debug print
    Serial.println("Marble ejector activated");
  }

  // Update previous state
  prevMarbleSensorState = currentState;
}

void handleTimedEvents(unsigned long currentMillis) {
  // Check if motor needs to be turned off
  if (motorActive && (currentMillis - motorStartTime >= MOTOR_ON_DURATION)) {
    digitalWrite(MOTOR_PIN, HIGH);   // HIGH turns off the low-level trigger relay
    digitalWrite(LIGHTS_PIN, LOW);   // LOW turns off the lights (assuming high-level trigger)
    motorActive = false;

    // Debug print
    Serial.println("Motor and lights deactivated");
  }

  // Check if marble ejector needs to be turned off
  if (marbleEjectorActive && (currentMillis - marbleStartTime >= MARBLE_ON_DURATION)) {
    digitalWrite(MARBLE_EJECTOR_PIN, HIGH);  // HIGH turns off the low-level trigger relay
    marbleEjectorActive = false;

    // Debug print
    Serial.println("Marble ejector deactivated");
  }
}