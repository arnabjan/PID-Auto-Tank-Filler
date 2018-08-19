#include <NewPing.h>

#define TRIGGER_PIN 8
#define ECHO_PIN 7
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#include <PID_v1.h>
//Define Variables we'll be connecting to
//Specify the links and initial tuning parameters
double Setpoint, Input, Output;

// Pid Stuff here the inputs all the outputs
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

int analogInPin = A0;
int sensorValue = 0;
int outputValue = 0;
int transistorPin = 9;

void setup()
{
  Serial.begin(115200);
  pinMode(8, OUTPUT); // tank Full indicator
  pinMode(9, OUTPUT); // tank empty indicator
  pinMode(transistorPin, OUTPUT);

  Setpoint = 80;
  Input = digitalRead(7);   // Read the echo form the Ultrasnoic sensor considering this as input to the PID controller
  myPID.SetMode(AUTOMATIC); //Setting it to auto mode.
}

void loop()
{
  delay(50); // Add Delay to stablize the perofmance of the Board.
  int uS = sonar.ping();

  Serial.print("Ping: ");
  Serial.print(uS / US_ROUNDTRIP_CM);

  int UValue = uS / US_ROUNDTRIP_CM; // Time taken to travel back and froth for the USonic sensor 
  //for detecting the tank Level in centimeters
  Serial.println("cm");

  Input = 100 - UValue;
  Serial.print("Motor Speed is");
  Serial.println(Output * 4);
  Serial.print("Water Level is");
  Serial.println(Input);
  myPID.Compute(); // compute with PID algorithm
  analogWrite(transistorPin, Output); //write the values to the transistor

  sensorValue = analogRead(analogInPin) / 4;
  //  Serial.print("Speed of motor is ");
  //  Serial.println(sensorValue*4);
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  //  analogWrite(transistorPin, sensorValue);
  if (sensorValue >= 160) // value for emptying the tank
  {
    digitalWrite(9, LOW); // tank Full indicator as OFF
    digitalWrite(8, HIGH); // tank empty indicator as ON
  }
  else
  {
    digitalWrite(9, HIGH);// tank empty indicator as ON
    digitalWrite(8, LOW);// tank empty indicator as OFF
  }
}
