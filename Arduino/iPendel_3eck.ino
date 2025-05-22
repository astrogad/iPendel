#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050.h>

// **MPU6050 IMU-Sensor**
MPU6050 mpu;


// **AS5047P Magnetic Encoder**
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);

// **IMU-Values**
float RateRoll, AccX, AccY, AccZ;
float AngleRoll, AngularSpeedRoll;
float dt = 0.01;
unsigned long last_time;
int pwmValue;
float supplyVoltage = 24.0; // Nanotec DC supply voltage
float motRe = 1.47; // Motor resistance [Ohm]
float motKe = 0.0355; // Motor back EMF Constant [Vs/rad] = Motor Torque Constant [Nm/A]
float angleFixRate = 0.0174; // Correct target angle in direction of the fall to keep fall smooth

const int pwmPin = 5;  // PWM pin for speed control
const int dirPin = 20;  // Digital pin for direction control

unsigned long previousMillis = 0;
const long interval = 5000; // 5 seconds

#define NUM_SAMPLES 20  // Increase samples for averaging
float alpha = 0.05;  // EMA smoothing factor
float filteredAccX = 0.0, filteredAccY = 0.0, filteredAngleRoll = 0.0, filteredAngularSpeedRoll = 0.0;
float motorVelocity = 0.0, filteredMotorVelocity = 0.0;
float controllInput = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C speed to 400kHz

  // Set PWM Frequency

  analogWriteFrequency(pwmPin, 10000); // 10 kHz

  // **MPU6050 Initialization**
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 Connection failed!");
    while (1);
  }

  // **Configure MPU6050**
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x07);  // DLPF at 3Hz cutoff
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);  // Accelerometer to ±2g
  Wire.endTransmission();

  delay(100);

  // **Initialize AS5047P Magnetic Encoder**
  sensor.init();

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  analogWrite(pwmPin, 0);  // Motor initially stopped
  digitalWrite(dirPin, LOW);  // Initial direction  

  last_time = micros();
}

void loop() {
  unsigned long current_time = micros();
  if (current_time - last_time < 5000) return;
  dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  sensor.update();
  gyro_signals();

  motorVelocity = sensor.getVelocity();
  
  if (abs(filteredAngleRoll) < 0.5) {
    controllInput = controllerLQR(filteredAngleRoll, filteredAngularSpeedRoll, motorVelocity);
    float current = controllInput;
    controllInput = motRe * current + motKe * motorVelocity;
    pwmValue = constrain(abs(controllInput), 3, 255);

    // Set motor speed
    analogWrite(pwmPin, pwmValue);    
    
    if (controllInput > 0) {
      digitalWrite(dirPin, LOW);
    } else {
        digitalWrite(dirPin, HIGH);
    }   
  } 
  else {
    controllInput = 0.0;
    pwmValue = 0;
    analogWrite(pwmPin, 0);
  }

  // Debugging
  Serial.print("Pendulum Angle : "); Serial.println(filteredAngleRoll); 
  Serial.print("Pendulum Velocity : "); Serial.println(filteredAngularSpeedRoll); 
  Serial.print("Motor Velocity : "); Serial.println(sensor.getVelocity()); 
  Serial.print("PWM Duty cycle : "); Serial.println(pwmValue); 
  Serial.print("Control Input : "); Serial.println(controllInput); 
  Serial.print("Loop Time : "); Serial.println(dt);
}

void gyro_signals() {
  long sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  long sumGyroX = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);

    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature
    int16_t GyroXLSB = Wire.read() << 8 | Wire.read();

    sumAccX += AccXLSB;
    sumAccY += AccYLSB;
    sumAccZ += AccZLSB;
    sumGyroX += GyroXLSB;
  }

  AccX = (float)(sumAccX / NUM_SAMPLES) / 4096.0;
  AccY = (float)(sumAccY / NUM_SAMPLES) / 4096.0;
  AccZ = (float)(sumAccZ / NUM_SAMPLES) / 4096.0;
  RateRoll = (float)(sumGyroX / NUM_SAMPLES) / 65.536;
  AngleRoll = atan2(AccY, -AccX);

  float gyro_offset = 0.0;
  AngularSpeedRoll = RateRoll * (PI / 180.0) - gyro_offset;

  // Exponential smoothing
  filteredAccX = alpha * AccX + (1 - alpha) * filteredAccX;
  filteredAccY = alpha * AccY + (1 - alpha) * filteredAccY;
  filteredAngleRoll = alpha * AngleRoll + (1 - alpha) * filteredAngleRoll;
  filteredAngularSpeedRoll = alpha * AngularSpeedRoll + (1 - alpha) * filteredAngularSpeedRoll;
}

float controllerLQR(float p_angle, float p_vel, float m_vel) {
  float u = 63.70 * p_angle + 3.07 * p_vel + 0.03 * m_vel;
  return u;
}