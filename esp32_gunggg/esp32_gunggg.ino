#include <Wire.h>
#include <BleMouse.h>

#define MPU_ADDR 0x68
#define PIN_SDA 8
#define PIN_SCL 9   

#define PIN_TRIGGER 4
#define PIN_RECAL   5
#define PIN_EXTRA   6
#define PIN_CLUTCH  3   // TTP223 touch module signal pin - active HIGH when touched

BleMouse bleMouse("ESP32 Gun", "DIY", 100);

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
const float GYRO_SCALE = 1.0f / 131.0f; // for FS_SEL=0 (+/-250 deg/s)

float SENS_X = 4.0f;   // tune these for feel
float SENS_Y = 4.0f;

float smoothRateX = 0, smoothRateY = 0;
bool trigDown = false, extraDown = false;
unsigned long lastMicros = 0;
unsigned long lastStatusPrint = 0;

// Auto-recenter (ZUPT) state
int stillCount = 0;
const int STILL_THRESHOLD = 8;      // raw gyro counts considered "basically still"
const int STILL_SAMPLES_NEEDED = 40; // ~40 loops (~0.3-0.5s) of stillness before nudging bias
const float BIAS_NUDGE_RATE = 0.02f; // how aggressively to pull bias toward current reading (0-1)

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int i2cErrorCount = 0;

bool mpuReadGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    i2cErrorCount++;
    Serial.print(">>> I2C ERROR code="); Serial.println(err);
    if (i2cErrorCount > 10) {
      Serial.println(">>> Resetting I2C bus...");
      Wire.end();
      delay(50);
      Wire.begin(PIN_SDA, PIN_SCL);
      Wire.setClock(100000);
      mpuWrite(0x6B, 0x00);
      mpuWrite(0x1B, 0x00);
      i2cErrorCount = 0;
    }
    return false;
  }
  uint8_t n = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (bool)true);
  if (n < 6) {
    i2cErrorCount++;
    Serial.print(">>> I2C SHORT READ n="); Serial.println(n);
    return false;
  }
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  i2cErrorCount = 0;
  return true;
}

uint8_t readWhoAmI() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (bool)true);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void calibrateGyro() {
  long sx = 0, sy = 0, sz = 0;
  const int N = 300;
  int valid = 0;
  for (int i = 0; i < N; i++) {
    int16_t gx = 0, gy = 0, gz = 0;
    if (mpuReadGyro(gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
      valid++;
    }
    delay(2);
  }
  if (valid > 0) {
    gyroBiasX = sx / (float)valid;
    gyroBiasY = sy / (float)valid;
    gyroBiasZ = sz / (float)valid;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_RECAL, INPUT_PULLUP);
  pinMode(PIN_EXTRA, INPUT_PULLUP);
  pinMode(PIN_CLUTCH, INPUT);   // touch module has its own output driver, no internal pullup needed

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  mpuWrite(0x6B, 0x00); // wake up MPU6050
  delay(100);
  mpuWrite(0x1B, 0x00); // gyro config: FS_SEL=0, +/-250 deg/s

  Serial.println("Calibrating gyro, keep still...");
  calibrateGyro();
  Serial.println("Done.");

  bleMouse.begin();
  lastMicros = micros();
}

void loop() {
  if (!bleMouse.isConnected()) {
    if (millis() - lastStatusPrint > 1000) {
      Serial.println("Not connected to PC...");
      lastStatusPrint = millis();
    }
    delay(50);
    return;
  }

  // Recalibrate button
  bool recal = digitalRead(PIN_RECAL) == LOW;
  if (recal) {
    delay(20); // debounce
    if (digitalRead(PIN_RECAL) == LOW) {
      Serial.println(">>> RECAL BUTTON PRESSED - recalibrating, keep still...");
      calibrateGyro();
      Serial.println(">>> RECAL DONE");
      delay(300); // avoid repeat trigger
    }
  }

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  int16_t gx = 0, gy = 0, gz = 0;
  bool ok = mpuReadGyro(gx, gy, gz);

  // Auto-recenter: detect stillness and slowly nudge bias, no button needed
  bool autoRecentered = false;
  if (ok) {
    float devX = gx - gyroBiasX;
    float devY = gy - gyroBiasY;
    if (fabs(devX) < STILL_THRESHOLD && fabs(devY) < STILL_THRESHOLD) {
      stillCount++;
      if (stillCount > STILL_SAMPLES_NEEDED) {
        gyroBiasX += devX * BIAS_NUDGE_RATE;
        gyroBiasY += devY * BIAS_NUDGE_RATE;
        autoRecentered = true;
      }
    } else {
      stillCount = 0;
    }
  }

  float rateX = ok ? (gx - gyroBiasX) * GYRO_SCALE : 0; // deg/s, yaw axis (tune which axis maps to X)
  float rateY = ok ? (gy - gyroBiasY) * GYRO_SCALE : 0; // pitch axis

  // Light smoothing to cut jitter (EMA filter)
  const float SMOOTH_ALPHA = 0.6f; // higher = less smoothing, more responsive
  smoothRateX = SMOOTH_ALPHA * rateX + (1 - SMOOTH_ALPHA) * smoothRateX;
  smoothRateY = SMOOTH_ALPHA * rateY + (1 - SMOOTH_ALPHA) * smoothRateY;
  rateX = smoothRateX;
  rateY = smoothRateY;

  int mouseY = (int)(rateX * dt * SENS_X * 10);
  int mouseX = (int)(rateY * dt * SENS_Y * 10); // invert if aim is upside down

  // Clutch: hold touch pad to freely reposition the gun without moving the cursor
  bool clutchHeld = digitalRead(PIN_CLUTCH) == HIGH;

  if (ok && !clutchHeld && (mouseX != 0 || mouseY != 0)) {
    bleMouse.move(mouseX, mouseY, 0);
  }

  // Trigger -> left click
  bool trig = digitalRead(PIN_TRIGGER) == LOW;
  if (trig && !trigDown) {
    Serial.println(">>> TRIGGER PRESSED");
    bleMouse.press(MOUSE_LEFT);
    trigDown = true;
  } else if (!trig && trigDown) {
    Serial.println(">>> TRIGGER RELEASED");
    bleMouse.release(MOUSE_LEFT);
    trigDown = false;
  }

  // Extra button -> right click (ADS)
  bool extra = digitalRead(PIN_EXTRA) == LOW;
  if (extra && !extraDown) {
    Serial.println(">>> EXTRA BUTTON PRESSED");
    bleMouse.press(MOUSE_RIGHT);
    extraDown = true;
  } else if (!extra && extraDown) {
    Serial.println(">>> EXTRA BUTTON RELEASED");
    bleMouse.release(MOUSE_RIGHT);
    extraDown = false;
  }

  // Continuous gyro/mouse debug line, ~5 times/sec
  if (millis() - lastStatusPrint > 200) {
    uint8_t who = readWhoAmI();
    Serial.print("WHO_AM_I=0x"); Serial.print(who, HEX);
    Serial.print(" gx="); Serial.print(gx);
    Serial.print(" gy="); Serial.print(gy);
    Serial.print(" gz="); Serial.print(gz);
    Serial.print(" | rateX="); Serial.print(rateX, 2);
    Serial.print(" rateY="); Serial.print(rateY, 2);
    Serial.print(" | mouseX="); Serial.print(mouseX);
    Serial.print(" mouseY="); Serial.print(mouseY);
    Serial.print(" | TRIG="); Serial.print(trig);
    Serial.print(" RECAL="); Serial.print(recal);
    Serial.print(" EXTRA="); Serial.print(extra);
    Serial.print(" CLUTCH="); Serial.print(clutchHeld);
    Serial.print(" AUTORECAL="); Serial.println(autoRecentered);
    lastStatusPrint = millis();
  }

  delay(5); // ~150-200Hz loop
}
