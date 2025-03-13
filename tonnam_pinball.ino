#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// Pin definitions
const int SCORE_SENSOR = A0;      // Single analog pin for all scoring sensors (parallel circuit)
const int SCORE_LEDS[] = {2, 3, 4, 5, 6}; // 5 LEDs for score indication
const int SONG_SENSORS[] = {A1, A2, A3, A4}; // 4 sensors for different songs
const int MOTOR_LIGHT_SENSOR = A5; // Sensor for motor and lights
const int MARBLE_SENSOR = A6;     // Sensor for marble ejection

// Output pins
const int MOTOR_PIN = 7;          // Motor control pin
const int LIGHTS_PIN = 8;         // Lights control pin (parallel connected)
const int MARBLE_EJECTOR_PIN = 9; // Marble ejector control pin

// MP3 Player setup
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

// Thresholds and variables
const int SENSOR_THRESHOLD = 500;     // Analog threshold for sensor detection
const int DETECTION_DEBOUNCE = 1000;  // Debounce time in milliseconds
int scoreCount = 0;                   // Counter for score detection
unsigned long lastScoreTime = 0;      // Time of last score detection

// Song file numbers (based on your 001.wav to 006.wav files)
const int SCORE_SONG_1 = 1;           // 001.wav - First score threshold song
const int SCORE_SONG_2 = 2;           // 002.wav - Second score threshold song
const int UNIQUE_SONGS[] = {3, 4, 5, 6}; // Songs 003.wav to 006.wav for the 4 unique sensors

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
const unsigned long MOTOR_ON_DURATION = 3000;    // 3 seconds
const unsigned long MARBLE_ON_DURATION = 1000;   // 1 second

// Previous sensor states for edge detection
bool prevScoreSensorState = false;
bool prevMotorSensorState = false;
bool prevMarbleSensorState = false;
bool prevSongSensorStates[4] = {false, false, false, false};

void setup() {
  // Initialize serial communications
  Serial.begin(9600);
  mySoftwareSerial.begin(9600);
  
  // Initialize DFPlayer
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println("Unable to begin DFPlayer Mini");
    while(true);
  }
  
  // Set volume (0-30)
  myDFPlayer.volume(20);
  
  // Initialize LED pins as outputs
  for (int i = 0; i < 5; i++) {
    pinMode(SCORE_LEDS[i], OUTPUT);
    digitalWrite(SCORE_LEDS[i], LOW);
  }
  
  // Initialize output pins
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LIGHTS_PIN, OUTPUT);
  pinMode(MARBLE_EJECTOR_PIN, OUTPUT);
  
  // Turn off all outputs initially
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(LIGHTS_PIN, LOW);
  digitalWrite(MARBLE_EJECTOR_PIN, LOW);
  
  Serial.println("System initialized for Arduino UNO");
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
        // Play song 001.wav at the first 5 detections
        myDFPlayer.play(SCORE_SONG_1);
        Serial.println("Playing score song 1 (001.wav)");
      } else if (scoreCount == 10) {
        // Play song 002.wav at the second 5 detections
        myDFPlayer.play(SCORE_SONG_2);
        Serial.println("Playing score song 2 (002.wav)");
        
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
      // Play the corresponding unique song (003.wav to 006.wav)
      int songNumber = UNIQUE_SONGS[i];
      myDFPlayer.play(songNumber);
      
      // Debug print
      Serial.print("Playing unique song ");
      Serial.print(songNumber);
      Serial.println(".wav");
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
    digitalWrite(MOTOR_PIN, HIGH);
    digitalWrite(LIGHTS_PIN, HIGH);
    motorActive = true;
    motorStartTime = millis();
    
    // Debug print
    Serial.println("Motor and lights activated");
  }
  
  // Update previous state
  prevMotorSensorState = currentState;
}

void checkMarbleEjectorSensor() {
  bool currentState = analogRead(MARBLE_SENSOR) > SENSOR_THRESHOLD;
  
  // Detect rising edge (sensor just activated)
  if (currentState && !prevMarbleSensorState && !marbleEjectorActive) {
    // Activate marble ejector
    digitalWrite(MARBLE_EJECTOR_PIN, HIGH);
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
    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(LIGHTS_PIN, LOW);
    motorActive = false;
    
    // Debug print
    Serial.println("Motor and lights deactivated");
  }
  
  // Check if marble ejector needs to be turned off
  if (marbleEjectorActive && (currentMillis - marbleStartTime >= MARBLE_ON_DURATION)) {
    digitalWrite(MARBLE_EJECTOR_PIN, LOW);
    marbleEjectorActive = false;
    
    // Debug print
    Serial.println("Marble ejector deactivated");
  }
}