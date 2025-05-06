/*
 * E6C2-CWZ3E Encoder Reading Program
 * 
 * This program reads an E6C2-CWZ3E 1024P/R encoder with 3 phase outputs (A, B, Z)
 * and displays position, direction, and speed on the serial monitor.
 * 
 * Connection details (based on the color coding):
 * - Brown wire (Vcc) -> 5V on Arduino
 * - Black wire (Phase A) -> Pin 2 (interrupt capable)
 * - White wire (Phase B) -> Pin 3 (interrupt capable) 
 * - Orange wire (Phase Z) -> Pin 4
 * - Blue wire (0V/COMMON) -> GND on Arduino
 * - Shield -> GND on Arduino
 * 
 * Note: The encoder is scaled to match the 1024 pulses per revolution specification.
 */

// Encoder pins
const int PIN_ENCODER_A = 2;  // Connect to Phase A (Black)
const int PIN_ENCODER_B = 3;  // Connect to Phase B (White)
const int PIN_ENCODER_Z = 4;  // Connect to Phase Z (Orange)

// Variables for encoder position tracking
volatile int rawEncoderPos = 0;   // Raw position counter (0-4095)
volatile long scaledEncoderPos = 0; // Scaled position to match 1024 P/R
volatile byte lastEncoded = 0;    // Store the previous encoded value
volatile long lastZPulse = 0;     // Position at last Z-pulse (for full rotations)
volatile bool zPulseDetected = false;  // Flag for Z pulse detection
float angleDegrees = 0.0;         // Current angle in degrees

// Variables for speed calculation
unsigned long lastTime = 0;
long lastPosition = 0;
float rpm = 0;
const int PPR = 1024;  // Pulses Per Revolution specified for this encoder
const int CPR = 4096;  // Counts Per Revolution in quadrature mode (internal tracking)
const float SCALE_FACTOR = 0.25; // Scale factor to convert 4096 counts to 1024 pulses

// Variables for direction detection
boolean clockwise = true;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("E6C2-CWZ3E Encoder Test");
  Serial.println("1024 Pulses Per Revolution");
  
  // Set pin modes
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_Z, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), updateEncoder, CHANGE);
  
  // Z-pulse interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_Z), zPulse, FALLING);
  
  lastTime = millis();
}

void loop() {
  // Calculate speed and display information every 200ms
  if (millis() - lastTime >= 200) {
    // Calculate time elapsed
    unsigned long timeElapsed = millis() - lastTime;
    
    // Calculate change in position
    long positionChange = scaledEncoderPos - lastPosition;
    
    // Calculate speed in RPM
    // (positionChange / PPR) = rotations
    // (rotations / timeElapsed) * 60000 = RPM
    rpm = (float)abs(positionChange) / PPR * (60000.0 / timeElapsed);
    
    // Determine direction
    clockwise = (positionChange >= 0);
    
    // Calculate angle in degrees (0-360)
    // First, get the absolute position modulo PPR to get position within one rotation
    long positionInRotation = scaledEncoderPos % PPR;
    if (positionInRotation < 0) positionInRotation += PPR; // Ensure positive value
    
    // Convert to degrees (position / PPR * 360)
    angleDegrees = (float)positionInRotation / PPR * 360.0;
    
    // Print information
    Serial.print("Position: ");
    Serial.print(scaledEncoderPos);
    Serial.print(" | Angle: ");
    Serial.print(angleDegrees, 2);  // 2 decimal places
    Serial.print("° | Direction: ");
    Serial.print(clockwise ? "CW" : "CCW");
    Serial.print(" | Speed: ");
    Serial.print(rpm, 2);  // 2 decimal places
    Serial.print(" RPM");
    
    if (zPulseDetected) {
      Serial.print(" | Z Pulse Detected! Rotation count: ");
      Serial.print(scaledEncoderPos / PPR);
      zPulseDetected = false;
    }
    
    Serial.println();
    
    // Update last values for next iteration
    lastTime = millis();
    lastPosition = scaledEncoderPos;
  }
}

void updateEncoder() {
  // Read the current state of encoder pins
  byte MSB = digitalRead(PIN_ENCODER_A);
  byte LSB = digitalRead(PIN_ENCODER_B);
  
  // Combine the values to get the encoded value
  byte encoded = (MSB << 1) | LSB;
  
  // Use Gray code pattern for more robust direction determination
  if((encoded == 0b00 && lastEncoded == 0b01) || 
     (encoded == 0b01 && lastEncoded == 0b11) ||
     (encoded == 0b11 && lastEncoded == 0b10) ||
     (encoded == 0b10 && lastEncoded == 0b00)) {
    // Clockwise rotation
    rawEncoderPos++;
  } else if((encoded == 0b00 && lastEncoded == 0b10) ||
            (encoded == 0b10 && lastEncoded == 0b11) ||
            (encoded == 0b11 && lastEncoded == 0b01) ||
            (encoded == 0b01 && lastEncoded == 0b00)) {
    // Counter-clockwise rotation
    rawEncoderPos--;
  }
  
  // Scale the raw position to match the 1024 P/R specification
  scaledEncoderPos = (long)(rawEncoderPos * SCALE_FACTOR);
  
  // Store the encoded value for next time
  lastEncoded = encoded;
}

void zPulse() {
  // Z pulse is detected (falling edge)
  lastZPulse = scaledEncoderPos;
  zPulseDetected = true;
}