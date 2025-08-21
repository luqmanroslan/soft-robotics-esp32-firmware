#include <Arduino.h>
#include <math.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <ArduinoJson.h>

// Pin definitions - using pid_tuner_fine_v3.ino pin assignments
const int inflateValvePin1 = 17;     // Inflate valve 1
const int deflateValvePin1 = 25;     // Deflate valve 1
const int pressureSensorPin1 = 12;   // Pressure sensor feedback 1

const int inflateValvePin2 = 14;     // Inflate valve 2
const int deflateValvePin2 = 16;     // Deflate valve 2
const int pressureSensorPin2 = 13;   // Pressure sensor feedback 2

const int inflateValvePin3 = 27;     // Inflate valve 3
const int deflateValvePin3 = 26;     // Deflate valve 3
const int pressureSensorPin3 = 4;    // Pressure sensor feedback 3

// Length sensor pins from pid_tuner
const int l1Pin = 36;
const int l2Pin = 39;
const int l3Pin = 34;
const int l4Pin = 35;
const int l5Pin = 32;
const int l6Pin = 33;

// Length constraints
const float min_l = 100.0;  // Minimum safe length in mm
const float max_l = 300.0;  // Maximum safe length in mm - increased from 250mm to 300mm

// Timings
int t0 = micros();
int lastUpdateTime = micros();
int HighLevelInterval= 200000; // microseconds

// Control mode flag
bool pressure_control_mode = false;  // Add this flag to track control mode
bool dance_mode = false;             // New: Dance mode flag
bool dance_started = false;          // New: Track if dance has been initiated

// Communication variables
String inputString = "";         // String for incoming data
bool stringComplete = false;     // Flag for complete commands
unsigned long lastSendTime = 0;  // Last sensor data transmission time
const int sendInterval = 10;     // Send interval (ms) - 100Hz

// Dance timing variables
int dance_start_time = 0;        // Dance start time in milliseconds
int dance_tm0 = 0;               // Dance timer reference

// Target lengths (will be set by PC commands)
float targetLength1 = 100.0;  // mm or normalized units
float targetLength2 = 100.0; 
float targetLength3 = 100.0;


// Current length values from PC
float currentLength1 = 200.0;  // Initial values
float currentLength2 = 200.0;
float currentLength3 = 200.0;

// Target pressures
float targetPressure1 = 0;
float targetPressure2 = 0;
float targetPressure3 = 0;
float max_p = 13.0;
float min_p = 0.0;

// Dance path from Dance.ino - normalized values (0.0 to 1.0)
double dance_path[][4] = {
{0, 0, 0, 0},
{3000, 0, 0, 0},
{4100, 0.8, 0.8, 0.8},

{6500, 0.8, 0.8, 0.8},
{8500, 0.8, 0.8, 0.3},
{9200, 0.6, 0.2, 0.2},
{9700, 0.6, 0.6, 0.6},
{10700, 0, 0, 0},

{13200, 0, 0, 0},
{15300, 0.2, 0.8, 0.2},
{19200, 0.1, 0.4, 0.8},

{20100, 0.1, 0.1, 0.1},
{22300, 0.4, 0.55, 0.8},
{23700, 0.7, 0.2, 0.2},
{24400, 0.8, 0, 0},
{26100, 0.0, 0.8, 0},
{27800, 0, 0, 0.8},
{29800, 0.8, 0.8, 0.8},

{32000, 0.8, 0.8, 0.8},
{32100, 1, 1, 1}, //ghost

{34100, 1, 1, 1},
{34200, 0, 0, 0},//ghost

{35100, 0.8, 0, 0},
{35500, 0, 0.65, 0.65},
{36100, 0.8, 0, 0},
{36700, 0, 0.65, 0.65},
{37200, 0.8, 0, 0},
{37900, 0, 0.8, 0.8},//chord
{39900, 0.2, 0, 0.2},//chord
{41100, 0.5, 0.5, 0.5},

{46300, 1, 1, 1},
{47100, 0, 0, 0},
{47700, 0, 0, 0},

{48700, 0, 0, 0.5},
{50900, 0, 0.5, 0.5},
{52900, 0.5, 0.5, 0.5},

{54600, 0.8, 0.3, 0.5},
{58300, 0, 0, 0.2},
{59100, 0, 0.4, 0},
{59700, 0.6, 0, 0},
{60900, 1.0, 1.0, 1.0},
{63100, 0.5, 0.5, 0.5},
{63200, 0, 0, 0},

{1000000000, 0, 0, 0}  // End marker
};

// Filtered pressures
float filteredPressure1 = -100;
float filteredPressure2 = -100;
float filteredPressure3 = -100;
float pressure_offset1 = 0;
float pressure_offset2 = 0;
float pressure_offset3 = 0;

// Low-Level Control Constants
float Low_Kp = 5600;
float Low_c = 2900;

// High-Level PID constants
float High_Kp = 0.02;
float High_Ki = 0.01;
float High_Kd = 0.001;

// High-Level PID state variables
float High_previousError1 = 0;
float High_previousError2 = 0;
float High_previousError3 = 0;
float High_integral1 = 0;
float High_integral2 = 0;
float High_integral3 = 0;

// IMU Definitions
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
int new_imu_flag = 0;

// IMU report type settings
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 10000; // 5000

// Control Functions -----------------------------------

float adc_to_voltage(float adc_count) { //Converts ADC Count to voltage (v)
  if (adc_count < 2690) {
    return 0.00084*adc_count + 0.1427;
  }
  return -0.0000001821*pow(adc_count, 2) + 0.001766*(adc_count) - 1.031;
}

float voltage_to_pressure(double voltage) { //Converts voltage (V) to absolute pressure (kPa)
  return (voltage - 0.00842*5)/(0.002421*5);
}

float readFilteredPressure(const int pressurePin, float filteredPressure, float pressure_offset) {
  int sensorValue = analogRead(pressurePin);
  float pressure = voltage_to_pressure(adc_to_voltage(sensorValue)) - pressure_offset;
  if (filteredPressure == -100) {
    filteredPressure = pressure; //overwrite filter if first value
  } else {
    filteredPressure = 0.15*pressure + 0.85*filteredPressure;
  }
  return filteredPressure;
}

int performLowLevelControl(float setpoint, float currentPressure, float prop_gain, float c_offset, const int inflateValvePin, const int deflateValvePin) {
  float error = setpoint - currentPressure;
  float output = constrain(error * prop_gain, -100000, 100000); //100ms

  if (output < 0) {
    digitalWrite(inflateValvePin, LOW);
    digitalWrite(deflateValvePin, HIGH);
  } else if (output > 0) { 
    digitalWrite(inflateValvePin, HIGH);
    digitalWrite(deflateValvePin, LOW);
  }
  return abs(output) + c_offset;
}

float performHighLevelControl(float targetLength, float currentLength, float& High_integral, float& High_previousError, float targetPressure, int interval) {
  // Use current length from PC instead of local sensor reading
  float error = targetLength - currentLength;

  // PID components
  High_integral += error * (interval / 1000.0);
  float derivative = (error - High_previousError) / (interval / 1000.0);
  float deltaPressure = High_Kp*error + High_Ki*High_integral + High_Kd*derivative;

  High_previousError = error;

  // Update pressure setpoint
  float currentSetpoint = targetPressure;  
  float newSetpoint = currentSetpoint + deltaPressure;

  // Clamp setpoint to allowed pressure range
  newSetpoint = constrain(newSetpoint, min_p, max_p);

  targetPressure = newSetpoint;

  return targetPressure;  // Return the new pressure setpoint
}

// IMU Functions
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// Communication Functions
void processCommands() {
  if (stringComplete) {
    // Create JSON document for parsing
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, inputString);
    
    if (!error) {
      String cmdType = doc["command"].as<String>();
      
      if (cmdType == "lengths_control") {
        // Disable pressure control mode
        pressure_control_mode = false;
        
        // Extract target lengths
        targetLength1 = doc["target_lengths"][0].as<float>();
        targetLength2 = doc["target_lengths"][1].as<float>();
        targetLength3 = doc["target_lengths"][2].as<float>();
        
        // Extract current lengths
        currentLength1 = doc["current_lengths"][0].as<float>();
        currentLength2 = doc["current_lengths"][1].as<float>();
        currentLength3 = doc["current_lengths"][2].as<float>();
        
        // Send detailed confirmation including received current lengths
        Serial.print("{\"confirm\":\"received_lengths_control\",\"received_targets\":[");
        Serial.print(targetLength1); Serial.print(",");
        Serial.print(targetLength2); Serial.print(",");
        Serial.print(targetLength3); Serial.print("],\"received_currents\":[");
        Serial.print(currentLength1); Serial.print(",");
        Serial.print(currentLength2); Serial.print(",");
        Serial.print(currentLength3); Serial.print("]}");
        Serial.println();
      }
      else if (cmdType == "length") {
        // Disable pressure control mode
        pressure_control_mode = false;
        
        // Simple length command handler added
        if (doc.containsKey("values") && doc["values"].size() >= 3) {
          // Extract target lengths from values array
          targetLength1 = doc["values"][0].as<float>();
          targetLength2 = doc["values"][1].as<float>();
          targetLength3 = doc["values"][2].as<float>();
          
          // For simple length command, we'll use the local sensor readings for current lengths
          // This is maintained for backward compatibility
          
          // Send confirmation
          Serial.print("{\"confirm\":\"received_simple_lengths\",\"targets\":[");
          Serial.print(targetLength1); Serial.print(",");
          Serial.print(targetLength2); Serial.print(",");
          Serial.print(targetLength3); Serial.print("]}");
          Serial.println();
        } else {
          Serial.println("{\"error\":\"Invalid length values\"}");
        }
      }
      else if (cmdType == "emergency_stop") {
        // Handle emergency stop - immediate action
        // Turn off all valves immediately
        digitalWrite(inflateValvePin1, LOW);
        digitalWrite(deflateValvePin1, LOW);
        digitalWrite(inflateValvePin2, LOW);
        digitalWrite(deflateValvePin2, LOW);
        digitalWrite(inflateValvePin3, LOW);
        digitalWrite(deflateValvePin3, LOW);
        
        // Stop dance mode if active
        dance_mode = false;
        dance_started = false;
        
        // Reset all control variables to safe values
        targetLength1 = min_l;
        targetLength2 = min_l;
        targetLength3 = min_l;
        targetPressure1 = min_p;
        targetPressure2 = min_p;
        targetPressure3 = min_p;
        
        // Reset PID state variables to prevent windup
        High_integral1 = 0;
        High_integral2 = 0;
        High_integral3 = 0;
        High_previousError1 = 0;
        High_previousError2 = 0;
        High_previousError3 = 0;
        
        // Force pressure control mode to prevent length commands from overriding
        pressure_control_mode = true;
        
        Serial.println("{\"status\":\"emergency_stop_activated\",\"dance_mode\":false}");
      }
      else if (cmdType == "init") {
        // Handle initialization
        Serial.println("{\"status\":\"initialized\"}");
      }
      else if (cmdType == "set_pressure") {
        // Enable pressure control mode
        pressure_control_mode = true;
        
        if (doc.containsKey("values") && doc["values"].size() >= 3) {
          // Extract target pressures from values array
          targetPressure1 = doc["values"][0].as<float>();
          targetPressure2 = doc["values"][1].as<float>();
          targetPressure3 = doc["values"][2].as<float>();
          
          // Clamp to allowed pressure range
          targetPressure1 = constrain(targetPressure1, min_p, max_p);
          targetPressure2 = constrain(targetPressure2, min_p, max_p);
          targetPressure3 = constrain(targetPressure3, min_p, max_p);
          
          // Send confirmation
          Serial.print("{\"confirm\":\"received_pressure_control\",\"targets\":[");
          Serial.print(targetPressure1); Serial.print(",");
          Serial.print(targetPressure2); Serial.print(",");
          Serial.print(targetPressure3); Serial.print("]}");
          Serial.println();
        } else {
          Serial.println("{\"error\":\"Invalid pressure values\"}");
        }
      }
      else if (cmdType == "start_dance") {
        // Start autonomous dance mode
        dance_mode = true;
        dance_started = false;
        pressure_control_mode = true;  // Use pressure control during dance
        
        // Reset dance timing
        dance_tm0 = millis();
        dance_start_time = dance_tm0;
        
        // Send PLAY SONG command for GUI to detect
        Serial.println("PLAY SONG");
        
        // Send confirmation
        Serial.println("{\"status\":\"dance_mode_started\",\"message\":\"PLAY SONG\"}");
      }
      else if (cmdType == "stop_dance") {
        // Stop dance mode
        dance_mode = false;
        dance_started = false;
        
        // Reset to safe pressures
        targetPressure1 = min_p;
        targetPressure2 = min_p;
        targetPressure3 = min_p;
        
        // Reset to normal pressure control mode
        pressure_control_mode = true;
        
        // Clear any PID state to prevent issues when switching modes
        High_integral1 = 0;
        High_integral2 = 0;
        High_integral3 = 0;
        High_previousError1 = 0;
        High_previousError2 = 0;
        High_previousError3 = 0;
        
        Serial.println("{\"status\":\"dance_mode_stopped\",\"mode\":\"pressure_control\",\"pressures\":[0.0,0.0,0.0]}");
      }
      else {
        // Unknown command
        Serial.print("{\"status\":\"error\",\"message\":\"Unknown command: ");
        Serial.print(cmdType);
        Serial.println("\"}");
      }
    } else {
      // JSON parsing error
      Serial.println("{\"error\":\"Invalid JSON command\"}");
    }
    
    // Clear input string
    inputString = "";
    stringComplete = false;
  }
}

// Add the potentiometer calibration function
float adc_to_voltage_pot(float adc_count) { 
  // Converts ADC count to voltage for potentiometers
  if (adc_count < 2690) {
    return 0.00084*adc_count + 0.1427;
  } else {
    return -0.0000001821*pow(adc_count, 2) + 0.001766*(adc_count) - 1.031;
  }
}

float voltage_to_length(float voltage) {
  // Converts voltage to potentiometer length
  return 46.4651*voltage + 165.1197;
}

float potentiometer_to_length(float adc_count) {
  // Combined function to convert ADC directly to length
  float voltage = adc_to_voltage_pot(adc_count);
  return voltage_to_length(voltage);
}

void sendSensorData() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    
    // Create sensor data JSON
    StaticJsonDocument<384> doc;
    
    // Add IMU data
    doc["imu_roll"] = ypr.roll;
    doc["imu_pitch"] = ypr.pitch;
    doc["imu_yaw"] = ypr.yaw;
    
    // Add potentiometer/length sensor data with calibration
    // Convert ADC values to calibrated length values
    doc["potentiometer_1"] = potentiometer_to_length(analogRead(l1Pin));
    doc["potentiometer_2"] = potentiometer_to_length(analogRead(l2Pin));
    doc["potentiometer_3"] = potentiometer_to_length(analogRead(l3Pin));
    doc["potentiometer_4"] = potentiometer_to_length(analogRead(l4Pin));
    doc["potentiometer_5"] = potentiometer_to_length(analogRead(l5Pin));
    doc["potentiometer_6"] = potentiometer_to_length(analogRead(l6Pin));
    
    // Add pressure data
    doc["pressure_1"] = filteredPressure1;
    doc["pressure_2"] = filteredPressure2;
    doc["pressure_3"] = filteredPressure3;
    
    // Add control data
    doc["target_pressure_1"] = targetPressure1;
    doc["target_pressure_2"] = targetPressure2;
    doc["target_pressure_3"] = targetPressure3;
    
    // Send JSON data
    serializeJson(doc, Serial);
    Serial.println();
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Reserve memory for input string
  inputString.reserve(256);
  
  // Set valve pins as outputs
  pinMode(inflateValvePin1, OUTPUT);
  pinMode(deflateValvePin1, OUTPUT);
  pinMode(inflateValvePin2, OUTPUT);
  pinMode(deflateValvePin2, OUTPUT);
  pinMode(inflateValvePin3, OUTPUT);
  pinMode(deflateValvePin3, OUTPUT);
  
  // Initialize IMU
  if (!bno08x.begin_I2C()) {
    Serial.println("{\"error\":\"Failed to find BNO08x chip\"}");
  } else {
    Serial.println("{\"status\":\"BNO08x Found\"}");
    setReports(reportType, reportIntervalUs);
  }
  
  // Initialize valves and calibrate pressure sensors
  digitalWrite(deflateValvePin1, HIGH);
  digitalWrite(deflateValvePin2, HIGH);
  digitalWrite(deflateValvePin3, HIGH);
  delay(1000);
  
  for (int i = 0; i < 100; i++) {
    pressure_offset1 += readFilteredPressure(pressureSensorPin1, -100, 0) / 100.0;
    pressure_offset2 += readFilteredPressure(pressureSensorPin2, -100, 0) / 100.0;
    pressure_offset3 += readFilteredPressure(pressureSensorPin3, -100, 0) / 100.0;
    delay(10);
  }
  
  digitalWrite(deflateValvePin1, LOW);
  digitalWrite(deflateValvePin2, LOW);
  digitalWrite(deflateValvePin3, LOW);
  
  // Send ready message
  Serial.println("{\"status\":\"ESP32 ready\"}");
  
  delay(1000);
  t0 = micros();
}

void loop() {
  int loop_start = micros();
  
  // Check IMU data
  if (bno08x.wasReset()) {
    Serial.println("{\"status\":\"IMU reset\"}");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    new_imu_flag = 1;
  }
  
  // Process incoming commands
  processCommands();
  
  // HIGH-LEVEL CONTROL LOOP
  // Update pressure setpoints based on length control
  int currentinterval = micros()-lastUpdateTime;
  if (currentinterval > HighLevelInterval){
    lastUpdateTime = micros();
  }
  
  // Update based on current control mode
  if (pressure_control_mode) {
    // Direct pressure control mode - already set targetPressure values
    // No need to update pressure setpoints, they're already set via command
  } else {
    // Length-based control (default)
    // Update pressure setpoints based on length control
    targetPressure1 = performHighLevelControl(targetLength1, currentLength1, High_integral1, High_previousError1, targetPressure1, currentinterval);
    targetPressure2 = performHighLevelControl(targetLength2, currentLength2, High_integral2, High_previousError2, targetPressure2, currentinterval);
    targetPressure3 = performHighLevelControl(targetLength3, currentLength3, High_integral3, High_previousError3, targetPressure3, currentinterval);
  }
  
  // DANCE MODE EXECUTION
  if (dance_mode) {
    int dance_tm = millis() - dance_tm0;  // Time since dance started
    
    // Find current position in dance sequence
    int i_after = 0;
    while (dance_path[i_after][0] < dance_tm && dance_path[i_after][0] < 1000000000) {
      i_after++;
    }
    
    if (i_after > 0 && dance_path[i_after][0] < 1000000000) {
      // Interpolate between dance points
      float t_before = dance_path[i_after-1][0];
      float t_after = dance_path[i_after][0];
      float x = (dance_tm - t_before) / (t_after - t_before);
      
      // Interpolated normalized values (0.0 to 1.0)
      float norm_pressure1 = dance_path[i_after-1][1] + (dance_path[i_after][1] - dance_path[i_after-1][1]) * x;
      float norm_pressure2 = dance_path[i_after-1][2] + (dance_path[i_after][2] - dance_path[i_after-1][2]) * x;
      float norm_pressure3 = dance_path[i_after-1][3] + (dance_path[i_after][3] - dance_path[i_after-1][3]) * x;
      
      // Convert normalized values to actual pressures
      targetPressure1 = min_p + norm_pressure1 * (max_p - min_p);
      targetPressure2 = min_p + norm_pressure2 * (max_p - min_p);
      targetPressure3 = min_p + norm_pressure3 * (max_p - min_p);
      
      // Ensure pressures are within bounds
      targetPressure1 = constrain(targetPressure1, min_p, max_p);
      targetPressure2 = constrain(targetPressure2, min_p, max_p);
      targetPressure3 = constrain(targetPressure3, min_p, max_p);
    } else if (dance_path[i_after][0] >= 1000000000) {
      // Dance sequence finished
      dance_mode = false;
      dance_started = false;
      
      // Reset to safe pressures
      targetPressure1 = min_p;
      targetPressure2 = min_p;
      targetPressure3 = min_p;
      
      // Reset to normal pressure control mode (not length control)
      pressure_control_mode = true;
      
      // Clear any PID state to prevent issues when switching modes
      High_integral1 = 0;
      High_integral2 = 0;
      High_integral3 = 0;
      High_previousError1 = 0;
      High_previousError2 = 0;
      High_previousError3 = 0;
      
      Serial.println("{\"status\":\"dance_finished\",\"mode\":\"pressure_control\",\"pressures\":[0.0,0.0,0.0]}");
    }
  }
  
  // LOW-LEVEL CONTROL LOOP
  // Read and filter pressure sensor data
  filteredPressure1 = readFilteredPressure(pressureSensorPin1, filteredPressure1, pressure_offset1);
  filteredPressure2 = readFilteredPressure(pressureSensorPin2, filteredPressure2, pressure_offset2);
  filteredPressure3 = readFilteredPressure(pressureSensorPin3, filteredPressure3, pressure_offset3);
  
  // Perform low-level pressure control
  int error_1 = performLowLevelControl(targetPressure1, filteredPressure1, Low_Kp, Low_c, inflateValvePin1, deflateValvePin1);
  int error_2 = performLowLevelControl(targetPressure2, filteredPressure2, Low_Kp, Low_c, inflateValvePin2, deflateValvePin2);
  int error_3 = performLowLevelControl(targetPressure3, filteredPressure3, Low_Kp, Low_c, inflateValvePin3, deflateValvePin3);
  
  int valve_on_time = micros();
  
  // Valve timing control loop
  while (micros() - valve_on_time < 20 * 1000){ // 50Hz control loop
    int v_time = micros() - valve_on_time;
    if (v_time > error_1) {
      digitalWrite(inflateValvePin1, LOW);
      digitalWrite(deflateValvePin1, LOW);
    }
    if (v_time > error_2) {
      digitalWrite(inflateValvePin2, LOW);
      digitalWrite(deflateValvePin2, LOW);      
    }
    if (v_time > error_3) {
      digitalWrite(inflateValvePin3, LOW);
      digitalWrite(deflateValvePin3, LOW);
    }
  }
  
  // Send sensor data to PC at regular intervals
  sendSensorData();
}
