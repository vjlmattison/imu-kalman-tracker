#include <Wire.h>
#include <Servo.h>

const int MPU_ADDR = 0x68;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

unsigned long lastSampleTime = 0;
const long sampleInterval = 10; // 10ms = ~100Hz sampling rate

Servo myServo;
String incomingCommand = ""; // builds up incoming text one character at a time

void setup() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  Serial.begin(115200);

  myServo.attach(9); // D9, matches the wiring
  myServo.write(90);  // start centered

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // calibrate gyro
  delay(1000);
  const int numSamples = 200;
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < numSamples; i++) {
    readRaw();
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    delay(5);
  }
  gyroX_offset = sumX / (float)numSamples;
  gyroY_offset = sumY / (float)numSamples;
  gyroZ_offset = sumZ / (float)numSamples;

  lastSampleTime = millis();
}

void loop() {
  // --- OUTGOING: stream sensor data, same as before ---
  unsigned long currentTime = millis();

  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    readRaw();

    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
    } else {
      float accX_g = accX / 16384.0;
      float accY_g = accY / 16384.0;
      float accZ_g = accZ / 16384.0;
      float gyroX_dps = (gyroX - gyroX_offset) / 131.0;
      float gyroY_dps = (gyroY - gyroY_offset) / 131.0;
      float gyroZ_dps = (gyroZ - gyroZ_offset) / 131.0;

      Serial.print(currentTime); Serial.print(",");
      Serial.print(accX_g, 4); Serial.print(",");
      Serial.print(accY_g, 4); Serial.print(",");
      Serial.print(accZ_g, 4); Serial.print(",");
      Serial.print(gyroX_dps, 4); Serial.print(",");
      Serial.print(gyroY_dps, 4); Serial.print(",");
      Serial.println(gyroZ_dps, 4);
    }
  }

  // --- INCOMING: listen for angle commands from Python, servo control ---
  // Serial.available() checks if any bytes have arrived that we haven't
  // read yet -- this is non-blocking, same "check, don't wait" philosophy
  // as millis() timing. We read one character at a time and build up a
  // full line; when we see the newline, the command is complete.
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      float targetAngle = incomingCommand.toFloat();
      // Servo library expects roughly 0-180. Our Kalman angle can be
      // negative or beyond that range, so map/constrain it into a safe
      // servo range centered at 90.
      float servoAngle = targetAngle + 90.0;
      servoAngle = constrain(servoAngle, 0, 180);
      myServo.write(servoAngle);
      incomingCommand = ""; // reset for the next command
    } else {
      incomingCommand += c;
    }
  }
}

void readRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}
