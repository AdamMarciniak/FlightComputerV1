#define TVC_X_PIN 2
#define TVC_Y_PIN 3
#define IMU_INTERRUPT_PIN 14
#define BUZZER_PIN 8
#define PYRO_1_DETECT_PIN 23
#define PYRO_2_DETECT_PIN 22
#define PYRO_1_FIRE_PIN 5
#define PYRO_2_FIRE_PIN 6


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

void setup() {

}



void loop() {

  switch(currentState) {
    case PAD_IDLE: {
      break;
    }

    case IGNITION_FIRING: {
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
