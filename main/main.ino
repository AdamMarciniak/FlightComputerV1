#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pitches.h"
#include <Metro.h>  // Timing
#include <SD.h>
#include <SPI.h>
#include <Chrono.h>
#include <Servo.h>

File dataFile;
File configFile;
#define TVC_X_PIN 2
#define TVC_Y_PIN 3
#define IMU_INTERRUPT_PIN 14
#define BUZZER_PIN 8
#define PYRO_1_DETECT_PIN 23
#define PYRO_2_DETECT_PIN 22
#define PYRO_1_FIRE_PIN 5
#define PYRO_2_FIRE_PIN 6
#define SD_CHIP_SELECT_PIN 10
#define X_SERVO_PIN 2
#define Y_SERVO_PIN 3

#define X_SERVO_CENTER 80
#define Y_SERVO_CENTER 96
#define X_SERVO_MIN 53
#define X_SERVO_MAX 107
#define Y_SERVO_MIN 69
#define Y_SERVO_MAX 123

#define IMU_UPDATE_RATE_MS 10
#define ALTIMETER_UPDATE_RATE_MS 50
#define LOG_RATE_MS 10

#define LOG_TIME_MS 10000

#define PYRO_1_THRESHOLD 200
#define PYRO_2_THRESHOLD 200

#define SEALEVELPRESSURE_HPA 1013.25

Metro IMUMetro = Metro(10);
Metro idleBuzzerMetro = Metro(1000);
Metro altimeterMetro = Metro(50);
Metro printMetro = Metro(10);

Servo x_servo;
Servo y_servo;

Chrono IMUChrono;
Chrono altimeterChrono;
Chrono printChrono;

int servoRotations[108][2] = {
  {  80  , 69  },
  { 81  , 69  },
  { 82  , 69  },
  { 83  , 69  },
  { 84  , 69  },
  { 85  , 69  },
  { 86  , 70  },
  { 87  , 70  },
  { 88  , 70  },
  { 89  , 71  },
  { 90  , 71  },
  { 91  , 71  },
  { 92  , 72  },
  { 93  , 72  },
  { 94  , 73  },
  { 95  , 74  },
  { 96  , 74  },
  { 97  , 75  },
  { 98  , 76  },
  { 99  , 77  },
  { 100 , 78  },
  { 101 , 79  },
  { 102 , 80  },
  { 103 , 82  },
  { 104 , 84  },
  { 105 , 86  },
  { 106 , 89  },
  { 107 , 96  },
  { 106 , 103 },
  { 105 , 106 },
  { 104 , 108 },
  { 103 , 110 },
  { 102 , 112 },
  { 101 , 113 },
  { 100 , 114 },
  { 99  , 115 },
  { 98  , 116 },
  { 97  , 117 },
  { 96  , 118 },
  { 95  , 118 },
  { 94  , 119 },
  { 93  , 120 },
  { 92  , 120 },
  { 91  , 121 },
  { 90  , 121 },
  { 89  , 121 },
  { 88  , 122 },
  { 87  , 122 },
  { 86  , 122 },
  { 85  , 123 },
  { 84  , 123 },
  { 83  , 123 },
  { 82  , 123 },
  { 81  , 123 },
  { 80  , 123 },
  { 79  , 123 },
  { 78  , 123 },
  { 77  , 123 },
  { 76  , 123 },
  { 75  , 123 },
  { 74  , 122 },
  { 73  , 122 },
  { 72  , 122 },
  { 71  , 121 },
  { 70  , 121 },
  { 69  , 121 },
  { 68  , 120 },
  { 67  , 120 },
  { 66  , 119 },
  { 65  , 118 },
  { 64  , 118 },
  { 63  , 117 },
  { 62  , 116 },
  { 61  , 115 },
  { 60  , 114 },
  { 59  , 113 },
  { 58  , 112 },
  { 57  , 110 },
  { 56  , 108 },
  { 55  , 106 },
  { 54  , 103 },
  { 53  , 96  },
  { 54  , 89  },
  { 55  , 86  },
  { 56  , 84  },
  { 57  , 82  },
  { 58  , 80  },
  { 59  , 79  },
  { 60  , 78  },
  { 61  , 77  },
  { 62  , 76  },
  { 63  , 75  },
  { 64  , 74  },
  { 65  , 74  },
  { 66  , 73  },
  { 67  , 72  },
  { 68  , 72  },
  { 69  , 71  },
  { 70  , 71  },
  { 71  , 71  },
  { 72  , 70  },
  { 73  , 70  },
  { 74  , 70  },
  { 75  , 69  },
  { 76  , 69  },
  { 77  , 69  },
  { 78  , 69  },
  { 79  , 69  }
};


void rotateServos() {
  for (int i = 0; i < 108; i += 1) {

    x_servo.write(servoRotations[i][0] );
    y_servo.write(servoRotations[i][1] );
    delay(20);
  }

  for (int i = 0; i < 108; i += 1) {
    x_servo.write(servoRotations[i][0] );
    y_servo.write(servoRotations[i][1] );
    delay(20);
  }

}

void initServos() {
  x_servo.attach(X_SERVO_PIN);
  x_servo.write(X_SERVO_CENTER);

  y_servo.attach(Y_SERVO_PIN);
  y_servo.write(Y_SERVO_CENTER);

  delay(500);
  x_servo.write(X_SERVO_MIN);
  delay(500);
  x_servo.write(X_SERVO_CENTER);
  delay(500);
  x_servo.write(X_SERVO_MAX);
  delay(500);
  x_servo.write(X_SERVO_CENTER);

  delay(750);
  y_servo.write(Y_SERVO_MIN);
  delay(500);
  y_servo.write(Y_SERVO_CENTER);
  delay(500);
  y_servo.write(Y_SERVO_MAX);
  delay(500);
  y_servo.write(Y_SERVO_CENTER);

  rotateServos();
  x_servo.write(X_SERVO_CENTER);
  y_servo.write(Y_SERVO_CENTER);
}



Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;

enum state {
  PAD_IDLE,
  IGNITION_FIRING,
  POWERED_ASCENT,
  UNPOWERED_ASCENT,
  DESCENT,
  PARACHUTE_FIRING,
  PARACHUTE_DESCENT,
  LANDED,
  FAILURE
};

String stateStrings[] {
  "PAD_IDLE",
  "IGNITION_FIRING",
  "POWERED_ASCENT",
  "UNPOWERED_ASCENT",
  "DESCENT",
  "PARACHUTE_FIRING",
  "PARACHUTE_DESCENT",
  "LANDED",
  "FAILURE"
};

state currentState = PAD_IDLE;

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long deltaT;

double yaw, pitch, roll;
double x_accel, y_accel, z_accel;
double x_gyro, y_gyro, z_gyro;

int8_t IMU_temp;

const int maxSteps = LOG_TIME_MS / LOG_RATE_MS;
const int numProperties = 10;
double dataArray[numProperties][maxSteps];
state stateArray[maxSteps];
unsigned long timeArray[maxSteps];
int dataStep = 0;

double altimeterTemperature = 0;
double altimeterPressure = 0;
float altimeter_alt_abs = 0;
float altimeter_alt_ground = 0;
float altimeter_alt_rel = 0;

bool pyro_1_state = false;
bool pyro_2_state = false;
bool pyro_1_detect = false;
bool pyro_2_detect = false;
bool pyro_1_fire = false;
bool pyro_2_fire = false;
bool pyro_check = false;

bool printed = false;

bool idleBuzzerState = false;

void buzz(int melody[], int durations[], int melodySize) {
  for (int thisNote = 0; thisNote < melodySize; thisNote++) {
    int noteDuration = 1000 / durations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
}

void buzzerSuccess() {
  int melody[] = {
    NOTE_G4, NOTE_C5, NOTE_E5, NOTE_G5, NOTE_E5, NOTE_G5
  };
  int noteDurations[] = {
    12, 12, 12, 6, 12, 3
  };
  int melodySize = 6;
  buzz(melody, noteDurations, melodySize);
}

void downloadConfig() {

}

void initSD() {
  Serial.print("Initializing SD card...  ");
  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    Serial.println("SD begin failed!");
    while (1);
  }
  Serial.println("  SD begin success.  ");


  //  configFile = SD.open("config.txt");
  //
  //  if(!configFile) {
  //    Serial.println("Config File open failed! ");
  //    while(1);
  //  } else {
  //    Serial.println("Config File open Success. Downloading Data");
  //    downloadConfig();
  //  }
  SD.remove("data1.txt");
  dataFile = SD.open("data1.txt", FILE_WRITE);

  if (!dataFile) {
    Serial.print(" SD dataFile open failed!");
    while (1);
  } else {
    Serial.print(" SD dataFile open success");
  }
  Serial.println();

}

void buzzerFlightFinished() {
  // Old spice jingle
  int melody[] = {
    NOTE_G5, NOTE_G5, NOTE_A5, NOTE_C6, NOTE_B5, NOTE_D6, NOTE_E6, NOTE_C6
  };
  int noteDurations[] = {
    12, 12, 6, 6, 6, 12, 6, 6
  };
  int melodySize = 8;
  buzz(melody, noteDurations, melodySize);
}

void buzzerIdle() {

  if (idleBuzzerMetro.check() == 1) {
    if (idleBuzzerState == false) {
      tone(BUZZER_PIN, NOTE_G5, 100);
      idleBuzzerState == true;
    } else {
      noTone(BUZZER_PIN);
      idleBuzzerState == false;
    }
  }
}

void initIMU() {
  Serial.print("Initializing IMU...   ");
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("IMU Initialize Success");


  delay(1000);
}

void printData() {
  Serial.print(deltaT);
  Serial.print(" ");
  Serial.print(stateStrings[currentState]);
  Serial.print(" ");
  Serial.print(altimeter_alt_rel);
  Serial.print(" ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(x_accel);
  Serial.print(" ");
  Serial.print(y_accel);
  Serial.print(" ");
  Serial.print(z_accel);
  Serial.print(" ");
  Serial.print(x_gyro);
  Serial.print(" ");
  Serial.print(y_gyro);
  Serial.print(" ");
  Serial.print(z_gyro);
  Serial.print(" ");
  Serial.println();
}


void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  while (1) {
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
      Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.println(mag, DEC);
  }


}


void updateIMU() {
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  IMU_temp = bno.getTemp();

  yaw = orientationData.orientation.x;
  pitch = orientationData.orientation.y;
  roll = orientationData.orientation.z;

  x_accel = linearAccelData.acceleration.x;
  y_accel = linearAccelData.acceleration.y;
  z_accel = linearAccelData.acceleration.z;

  x_gyro = angVelocityData.gyro.x;
  y_gyro = angVelocityData.gyro.y;
  z_gyro = angVelocityData.gyro.z;
}

void initAltimeter() {
  Serial.print("Initializing Altimeter..");
  if (!bmp.begin()) {
    Serial.println("Altimeter failed!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.print("Altimeter Success. Reading Ground Altitude");
  Serial.println();

  int i = 0;
  while (i < 20) {
    if (altimeterChrono.hasPassed(50)) {
      altimeterChrono.restart();
      if (!bmp.performReading()) {
        Serial.println("Failed to perform Ground Altimeter reading");
        while (1);
      }
      bmp.readAltitude(SEALEVELPRESSURE_HPA);
      i += 1;
    }
  }

  if (!bmp.performReading()) {
    Serial.println("Failed to perform Ground Altimeter reading");
    while (1);
  }

  altimeter_alt_ground = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.print("Ground Altitude (m)");
  Serial.print(altimeter_alt_ground);
  Serial.println();
}

void updateAltimeter() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform Altimeter reading");
    return;
  }
  bmp.temperature;
  bmp.pressure;
  altimeterTemperature = bmp.temperature;
  altimeterPressure = bmp.pressure / 100.0;
  altimeter_alt_abs = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  altimeter_alt_rel = altimeter_alt_abs - altimeter_alt_ground;

}

void upload(String value) {
  Serial.print(value);
  Serial.print(" ");
  dataFile.print(value);
  dataFile.print(" ");
}

void printDataArray() {

  for (int i = 0; i < maxSteps; i += 1) {
    upload(String(timeArray[i]));
    upload(String(stateStrings[stateArray[i]]));
    for (int j = 0; j < numProperties; j += 1) {
      upload(String(dataArray[j][i]));
    }
    Serial.println();
    dataFile.println();
  }
  dataFile.close();
}

bool checkPyros() {
  if (analogRead(PYRO_1_DETECT_PIN) > PYRO_1_THRESHOLD) {
    pyro_1_state = true;
    Serial.print("PYRO 1 SUCCESS");
  } else {
    pyro_1_state = false;
    Serial.print("PYRO 1 FAIL");
    return false;
  }
  Serial.println();
  if (analogRead(PYRO_2_DETECT_PIN) > PYRO_2_THRESHOLD) {
    pyro_2_state = true;
    Serial.print("PYRO 2 SUCCESS");
  } else {
    pyro_2_state = false;
    Serial.print("PYRO 2 FAIL");
    return false;
  }
  Serial.println();
  if (pyro_1_state != true && pyro_2_state != true) {
    Serial.print("PYRO CHECK FAILED");
    Serial.println();
    return false;
  } else {
    Serial.print("PYRO CHECK PASSED");
    Serial.println();
    return true;
  }
}

void saveDataToArray() {
  if (dataStep < maxSteps) {
    timeArray[dataStep] = millis() - previousMillis;
    stateArray[dataStep] = currentState;
    dataArray[0][dataStep] = altimeter_alt_rel;
    dataArray[1][dataStep] = pitch;
    dataArray[2][dataStep] = roll;
    dataArray[3][dataStep] = x_accel;
    dataArray[4][dataStep] = y_accel;
    dataArray[5][dataStep] = z_accel;
    dataArray[6][dataStep] = x_gyro;
    dataArray[7][dataStep] = y_gyro;
    dataArray[8][dataStep] = z_gyro;
    dataArray[9][dataStep] = yaw;
    dataStep += 1;
  }
}

unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr, lastInput;
double kp, ki, kd;
double ITerm;
double outputSum

void

#define PID_RATE_MS 10.0
#define DIRECT 0
#define REVERSE 1
int pidDirection = DIRECT;
#define P_ON_M 0
#define P_ON_E 1
bool pOnE = true;
double PTerm = 0;

void setTunings(double Kp, double Ki, double Kd, int pOn) {
  kp = Kp;
  ki = Ki * (double)PID_RATE_MS / 1000.0;
  kd = Kd / (double)PID_RATE_MS / 1000.0;

  pOnE = pOn == P_ON_E;

  if (pidDirection == REVERSE) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

void computePID(double outMax, double outMin) {

  double error = Setpoint - Input;
  double dInput = (Input - lastInput);
  outputSum += (ki * error);

  if (!pOnE) outputSum -= kp * dInput;

  if (pOnE) Output = kp * error;
  else Output = 0;

  Output += outputSum - kd * dInput;
  if (Output > outMax) Output = outMax;
  else if (Output < outMin) Output = outMin;
  lastInput = Input;
  lastErr = error;
}

void setup() {
  Serial.begin(115200);
  initSD();
  initIMU();
  //displayCalStatus();
  initAltimeter();
  initServos();
  buzzerSuccess();
  previousMillis = millis();
  Serial.println("SETUP COMPLETE. STARTING LOOP");
}

void loop() {

  switch (currentState) {
    case PAD_IDLE: {
        buzzerIdle();

        if (IMUChrono.hasPassed(IMU_UPDATE_RATE_MS)) {
          IMUChrono.restart();
          updateIMU();
        }

        if (altimeterChrono.hasPassed(ALTIMETER_UPDATE_RATE_MS)) {
          altimeterChrono.restart();
          updateAltimeter();
        }

        if (printChrono.hasPassed(LOG_RATE_MS)) {
          printChrono.restart();
          printData();
          saveDataToArray();
        }

        if (dataStep == maxSteps - 1) {
          currentState = IGNITION_FIRING;
          Serial.println("PRINTING DATA...");
          delay(1000);
        }
        break;
      }

    case IGNITION_FIRING: {
        if (printed == false) {
          printDataArray();
          printed = true;
          buzzerFlightFinished();
        }

        break;
      }

    case POWERED_ASCENT: {
        break;
      }

    case UNPOWERED_ASCENT: {
        break;
      }

    case DESCENT: {
        break;
      }

    case PARACHUTE_FIRING: {
        break;
      }

    case PARACHUTE_DESCENT: {
        break;
      }

    case LANDED: {
        break;
      }

    case FAILURE: {
        break;
      }
  }

}
