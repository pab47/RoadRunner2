/* Roadrunner PID Controller
    ------------------------
   Eric Sanchez 9/26/2018
   -------------------------
   Communicates with RPi and gets sensor information. Locally
   runs PID loop using PID library from Brett Beaugard
*/

#include <PID_v1_micros.h>
#include <Wire.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

// Variables passed from RPi
double Setpoint, Input, Output;
bool setptOn = false;

// Loop Speed
static int pidFreq = 1000; // us 1000 Hz
unsigned long ioFreq = 10; // ms 100 Hz

// PID Gains
// double Kp = .4, Ki = 2.75, Kd = .04;
//double Kp = 0.025, Ki = 0.0015, Kd = 0.001; //MEH
//double Kp = 0.45, Ki = 0.0, Kd = 1.5;
//double Kp = 0.4, Ki = 0.0, Kd = 37.0; // Tuned Kd
//double Kp = 0.45, Ki = 0.0, Kd = 37.0; // Tuned Kp
//double Kp = 0.45, Ki = 0.0005, Kd = 37; // Tuned Ki P_ON_E
//double Kp = 0.3, Ki = 0.0, Kd = 30.0;

//double Kp = 0.125, Ki = 0.0, Kd = 0.0; // Tuned Kp
//double Kp = 0.13, Ki = 0.0004, Kd = 11.0; // Tuned

//double Kp = 0.15, Ki = 0.0005, Kd = 16.0; // Tuned Kp

//double Kp = 0.01, Ki = 0.0, Kd = 16.0; // Tuned Kd
double Kp = 0.17, Ki = 0.0005, Kd = 16.0; // Tuned Kp


// Initialize vars
unsigned long prevTime = 0, currTime = 0;

double prevSetpoint = 0.0;
bool prevSetptOn = false;

bool write_;
float mapped_out = 0.0;
//double esc_min = 0.0, esc_max = 15.0;
double esc_min = -1.0, esc_max = 15.0;

float turn_diff = 0.0;

// Vars for reading Serial data
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

bool newData = false;

// Vars for Sending Data
static char sendbuf[5];
String out = String(8);

// Initialize PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Initialize ODrive object
// ODriveArduino odrive(Serial1);

void setup() {
  // Initialize vars
  prevTime = millis();

  // Set PID sample time
  myPID.SetSampleTime(pidFreq);
  myPID.SetOutputLimits(esc_min, esc_max);

  // Initialize serial ports: USB (Serial) and HardwareSerial (Serial1)
  Serial.begin(115200);
  Serial1.begin(115200);

  // Start the out string
  out = '<';

  // Turn on LED to show that it is ready/working
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  // Check to see whether to enable or disable PID
  checkSetptOn();

  // Calculate PID
  write_ = myPID.Compute();

  if (write_) {
    // Write the PID output to ODrive
    // writeAmpsODrive(Serial1, (float) Output);
    writeAmpsODriveDiff(Serial1, Output, turn_diff);
  }

  // Grab/send data loop
  currTime = millis();

  if (((currTime - prevTime) >= ioFreq) && setptOn) {
    // Send Data
    dtostrf(Output, 3, 2, sendbuf);
    Serial.println(out + sendbuf + '>');
    // Serial.print('<'); Serial.print(Output); Serial.print('>');

    prevTime = currTime;
  }

  // Grab Data
  recvWithStartEndMarkers();

  if (newData) {
    // This temporary copy is necessary to protect the original data
    // because strtok() used in parseData() replaces the commas with \0
    strcpy(tempChars, receivedChars);
    // Parse incoming data into Input Setpoint and setptOn
    parseData();
    newData = false;
  }
}

// Functions ---------------------------------------------------
// -------------------------------------------------------------

// Map function, but float instead of int or long
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Use Serial to write a desired current to the ODrive (ASCII Interface)
void writeAmpsODrive(HardwareSerial _serial_, float current) {
  _serial_ << "c " << 0 << " " << -1 * current << "\n";
  _serial_ << "c " << 1 << " " << current << "\n";
}

// Use Serial to write a differential current (one side more than other) to turn
void writeAmpsODriveDiff(HardwareSerial _serial_, float current, float diff) {
  float absdiff = fabs(diff);
  // Turn Left
  if (diff < 0.0) {
    _serial_ << "c " << 0 << " " << -1.0 * (current - absdiff) << "\n";
    _serial_ << "c " << 1 << " " << current + absdiff << "\n";
  }
  // Turn Right
  else if (diff > 0.0) {
    _serial_ << "c " << 0 << " " << -1.0 * (current + absdiff) << "\n";
    _serial_ << "c " << 1 << " " << current - absdiff << "\n";
  }
  // Straight
  else {
    _serial_ << "c " << 0 << " " << -1.0 * current << "\n";
    _serial_ << "c " << 1 << " " << current << "\n";
  }
}

void writeAmpsODrive(float current) {
  Serial1.print("c 0 -"); Serial1.print(current); Serial1.print('\n');
  Serial1.print("c 0 ");  Serial1.print(current); Serial1.print('\n');
}

// Recieve Serial data with a desired start and end marker. Allows
// the Arduino to continue doing stuff in the loop, not a blocking
// function (change Serial_ to HardwareSerial if needed)
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// Takes a char array and converts its contents into numbers
// Message from the Pi will look like this:
// "<feedback,setpt,setptOn>"
// "<50.0000,50.0,1>": float, float, bool
void parseData() {

  char * strtokIndx;    // this is used by strtok() as an index
  int setptOn_ = 0;

  strtokIndx = strtok(tempChars, ",");      // Get feedback - double
  Input = atof(strtokIndx);                 // Convert to double

  strtokIndx = strtok(NULL, ",");           // Get setpoint - double
  Setpoint = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");           // Get setptOn - bool
  setptOn_ = atoi(strtokIndx);
  setptOn = (bool) setptOn_;

  strtokIndx = strtok(NULL, ",");
  turn_diff = atof(strtokIndx);
}

// Check for change in setptOn to turn off PID
void checkSetptOn() {
  if (setptOn != prevSetptOn) {
    if (setptOn) {
      myPID.SetMode(AUTOMATIC);
    }
    else {
      myPID.SetMode(MANUAL);
    }
  }
  prevSetptOn = setptOn;
}
