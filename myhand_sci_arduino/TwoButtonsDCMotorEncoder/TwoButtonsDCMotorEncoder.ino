#include <Adafruit_MotorShield.h>
#include <Encoder.h>

// Pins
#define forwardButtonPin 4
#define backwardButtonPin 5

// Encoder Pins
#define ENCODER1 2
#define ENCODER2 3

// Variables
int forwardButtonState = 1;
int backwardButtonState = 1;

const int speed_dc = 150;
volatile long int encoder_pos = 0;

// Safety Variables:
long int enc_lb = 0;
long int enc_ub = 20000;

// Motor Shield Initiation:
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// DC Motor:
Adafruit_DCMotor * myDCMotor = AFMS.getMotor(1);

// Encoder Initiation:
Encoder myEnc(ENCODER1, ENCODER2);

void setup() {
  Serial.begin(115200);

  // Initialize Button Pins
  pinMode(forwardButtonPin, INPUT_PULLUP);
  pinMode(backwardButtonPin, INPUT_PULLUP);

  //Check Motor Shield Status:
  if (!AFMS.begin()) { 
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myDCMotor->setSpeed(0);

  // Enable DC Motor
  myDCMotor->run(RELEASE);
}

void loop() {
  forwardButtonState = digitalRead(forwardButtonPin);
  backwardButtonState = digitalRead(backwardButtonPin);
  
  if (forwardButtonState == LOW && myEnc.read() < enc_ub) { 
    
    // Run DC Motor Forwards
    myDCMotor->run(FORWARD);
    Serial.println("Forward");
    while (forwardButtonState == LOW && myEnc.read() < enc_ub) {
      myDCMotor->setSpeed(speed_dc);
      forwardButtonState = digitalRead(forwardButtonPin);

      // delay(100);
      Serial.println("Going Forward");
    }
  } else if (backwardButtonState == LOW && myEnc.read() > enc_lb) { 
    
    // Run DC Motor Backwards
    myDCMotor->run(BACKWARD);
    Serial.println("Backward");
    while (backwardButtonState == LOW && myEnc.read() > enc_lb) {
      myDCMotor->setSpeed(speed_dc);
      backwardButtonState = digitalRead(backwardButtonPin);

      // delay(100);
      Serial.println("Going Backward");
    }
  } else { 
   myDCMotor->setSpeed(0);
  }
  Serial.println(myEnc.read());
}
