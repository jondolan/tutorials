/*
  Jon Dolan
  Arduino PWM motor driver
  Prepared for the Georgia Tech IEEE Hardware Team 2018
  
  hardware:
    Arduino UNO
    DC motor
    SparkFun TB6612FNG motor driver (doesn't have to be this driver specifically)

  hookups:
    Arduino pin 11->motor driver STBY
    Arduino pin 12->motor driver AIN1
    Arduino pin 13->motor driver AIN2
    Arduino pin 9->motor driver PWMA
    Arduino 5V->motor driver VCC
    Arduino GND->motor driver GND
    Power supply power and ground->motor driver VM and GND
*/

/* pin defintions */
#define PWMA (9)    // motor PWM signal
#define STBY (11)   // motor control standby (enable)
#define AIN1 (12)   // motor direction 1
#define AIN2 (13)   // motor direction 2

// initialize, runs once on power up
void setup() {
  // initialize the h bridge control signals to outputs
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // bring standby high to turn on the h bridge
  digitalWrite(STBY, HIGH);

  // motor forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

// the loop function runs over and over again forever after setup terminates
void loop() {
  // ramp a motor from 0 to full sleep slowly, run at full for 2 seconds, stop for 2 seconds, and repeat
  
  for (int i = 0; i < 255; i=i+5) { // for i=0->255 by 5
    analogWrite(PWMA, i); // set PWMA to the specified duty cycle
    delay(100); // delay so we don't ramp the whole way instantly
  }
  delay(2000);  // run the motor at full speed for 2 seconds

  // stop the motor
  analogWrite(PWMA, 0);

  // break for 2 seconds
  delay(2000);
}