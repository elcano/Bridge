/*
 * Test Executor - Reads and executes test scripts
 * Runs on Router Arduino Due
 * 
 * Reads CSV test scripts from SD card
 * Sends commands to simulator
 * Verifies results from DBW log
 * Reports PASS/FAIL
 */

#include <SD.h>

#define SD_CS_PIN 37

File scriptFile;
File logFile;
bool sdAvailable = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize SD card
  pinMode(SD_CS_PIN, OUTPUT);
  if (SD.begin(SD_CS_PIN)) {
    sdAvailable = true;
    Serial.println("SD card initialized");
  } else {
    Serial.println("ERROR: SD card not found");
    while(1);
  }
  
  delay(1000);
  Serial.println("=== Test Executor Started ===");
  Serial.println("");
}

void loop() {
  // Execute test scripts
  executeTestScript("test_steering_comprehensive.csv", "Steering Comprehensive");
  delay(2000);
  
  executeTestScript("test_speed_and_steering.csv", "Speed and Steering");
  delay(2000);
  
  executeTestScript("test_brake.csv", "Brake Test");
  delay(2000);
  
  Serial.println("=== All Tests Completed ===");
  while(1);
}

void executeTestScript(char* filename, char* testName) {
  Serial.print("Running Test: ");
  Serial.println(testName);
  Serial.print("Script: ");
  Serial.println(filename);
  
  // Check if file exists
  if (!SD.exists(filename)) {
    Serial.println("ERROR: Script file not found");
    return;
  }
  
  // Open script file
  scriptFile = SD.open(filename);
  if (!scriptFile) {
    Serial.println("ERROR: Could not open script file");
    return;
  }
  
  // Read and execute each command
  char line[256];
  int lineIdx = 0;
  unsigned long scriptStartTime = millis();
  
  while (scriptFile.available()) {
    char c = scriptFile.read();
    
    if (c == '\n' || c == '\r') {
      line[lineIdx] = '\0';
      
      // Skip empty lines and comments
      if (line[0] != '\0' && line[0] != '#') {
        // Parse and execute command
        executeCommand(line, scriptStartTime);
      }
      lineIdx = 0;
    } else {
      if (lineIdx < 255) {
        line[lineIdx++] = c;
      }
    }
  }
  
  scriptFile.close();
  
  // Verify results
  verifyResults(testName);
  
  Serial.println("");
}

void executeCommand(char* line, unsigned long scriptStartTime) {
  // Parse: time_ms, CANID, nbytes, speed, brake, mode, angle
  int fields[7];
  int nFields = 0;
  char *ptr = line;
  
  while (*ptr && nFields < 7) {
    fields[nFields++] = (int)strtol(ptr, &ptr, 0);
    if (*ptr == ',') ptr++;
  }
  
  if (nFields < 7) return;
  
  unsigned long executeTime = fields[0];
  int canId = fields[1];
  int nbytes = fields[2];
  int speed = fields[3];
  int brake = fields[4];
  int mode = fields[5];
  int angle = fields[6];
  
  // Wait until it's time to execute this command
  while (millis() - scriptStartTime < executeTime) {
    delay(10);
  }
  
  // Send command to simulator
  Serial.print("CMD: ");
  Serial.print(canId);
  Serial.print(",");
  Serial.print(nbytes);
  Serial.print(",");
  Serial.print(speed);
  Serial.print(",");
  Serial.print(brake);
  Serial.print(",");
  Serial.print(mode);
  Serial.print(",");
  Serial.println(angle);
}

void verifyResults(char* testName) {
  Serial.print("Verifying: ");
  Serial.println(testName);
  
  // Find most recent LOG file
  char logFilename[20] = "";
  int logNum = -1;
  
  // Check for LOG files
  for (int i = 0; i < 100; i++) {
    sprintf(logFilename, "LOG%02d.CSV", i);
    if (SD.exists(logFilename)) {
      logNum = i;  // Get the latest one
    }
  }
  
  if (logNum < 0) {
    Serial.println("WARNING: No DBW log file found");
    return;
  }
  
  sprintf(logFilename, "LOG%02d.CSV", logNum);
  Serial.print("Reading log: ");
  Serial.println(logFilename);
  
  // Open and read log file
  logFile = SD.open(logFilename);
  if (!logFile) {
    Serial.println("ERROR: Could not open log file");
    return;
  }
  
  // Read log and check steering angles
  int passCount = 0;
  int totalCount = 0;
  char line[512];
  int lineIdx = 0;
  
  while (logFile.available()) {
    char c = logFile.read();
    
    if (c == '\n' || c == '\r') {
      line[lineIdx] = '\0';
      
      if (line[0] != '\0' && line[0] != 't') {  // Skip header
        totalCount++;
        // Parse steering angle from log
        int angle = parseAngleFromLog(line);
        
        if (angle != 0) {  // Non-zero means steering happened
          passCount++;
        }
      }
      lineIdx = 0;
    } else {
      if (lineIdx < 511) {
        line[lineIdx++] = c;
      }
    }
  }
  
  logFile.close();
  
  // Report results
  Serial.print("Results: ");
  Serial.print(passCount);
  Serial.print("/");
  Serial.print(totalCount);
  Serial.println(" commands executed");
  
  if (passCount > 0) {
    Serial.println("PASS: Steering commands were executed");
  } else {
    Serial.println("FAIL: No steering response detected");
  }
}

int parseAngleFromLog(char* line) {
  // CSV format: time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths,...
  // angle_tenths is column 5 (0-indexed)
  
  int col = 0;
  char *ptr = line;
  char *token;
  int angle = 0;
  
  while ((token = strtok(ptr, ",")) != NULL && col < 6) {
    if (col == 5) {
      angle = atoi(token);
    }
    col++;
    ptr = NULL;
  }
  
  return angle;
}