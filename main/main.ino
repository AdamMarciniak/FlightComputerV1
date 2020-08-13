#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pitches.h"
#include <Metro.h>  // Timing

#define TVC_X_PIN 2
#define TVC_Y_PIN 3
#define IMU_INTERRUPT_PIN 14
#define BUZZER_PIN 8
#define PYRO_1_DETECT_PIN 23
#define PYRO_2_DETECT_PIN 22
#define PYRO_1_FIRE_PIN 5
#define PYRO_2_FIRE_PIN 6

#define IMU_RATE_MS 10
#define LOG_TIME_MS 10000

void buzz(int melody[], int durations[], int melodySize){
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


const int maxSteps = LOG_TIME_MS / IMU_RATE_MS;
const int numProperties = 9;
double dataArray[numProperties][maxSteps];
int dataStep = 0;


Metro IMUMetro = Metro(10);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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

void saveDataToArray() {
  if(dataStep < maxSteps){
    dataArray[0][dataStep] = yaw;
    dataArray[1][dataStep] = pitch;
    dataArray[2][dataStep] = roll;
    dataArray[3][dataStep] = x_accel;
    dataArray[4][dataStep] = y_accel;
    dataArray[5][dataStep] = z_accel;
    dataArray[6][dataStep] = x_gyro;
    dataArray[7][dataStep] = y_gyro;
    dataArray[8][dataStep] = z_gyro;
    dataStep += 1;
  }
}

void printData(){
  Serial.print(deltaT);
  Serial.print(" ");
  Serial.print(stateStrings[currentState]);
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



void setup() {
  Serial.begin(115200);
  initIMU();
  buzzerSuccess();
}

bool printed = false;

void printDataArray() {
 for(int i = 0; i < maxSteps; i += 1){
  for(int j = 0; j < numProperties; j += 1){
    Serial.print(dataArray[j][i]);
    Serial.print(" ");
  }
  Serial.println();
 }
}


void loop() {
  

  switch(currentState) {
    case PAD_IDLE: {
      if(IMUMetro.check() == 1){
        deltaT = micros() - previousMillis;
        previousMillis = micros();
        updateIMU();
        printData();
        saveDataToArray();
       }
      if(dataStep == maxSteps - 1){
        currentState = IGNITION_FIRING;
        Serial.println("PRINTING DATA...");
        delay(1000);
      }
      break;
    }

    case IGNITION_FIRING: {
      if(printed == false){
        printDataArray();
        printed = true;
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
