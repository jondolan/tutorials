/*
  Jon Dolan
  Arduino quadrature encoder feedback PID controller
  Encoder interrupt function based off of http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
  Prepared for the Georgia Tech IEEE Hardware Team 2018
  
  hardware:
    Arduino UNO
    DC motor with quadrature encoder
    SparkFun TB6612FNG motor driver (doesn't have to be this driver specifically)
  hookups:
    Arduino pin 11->motor driver STBY
    Arduino pin 12->motor driver AIN1
    Arduino pin 13->motor driver AIN2
    Arduino pin 9->motor driver PWMA
    Arduino 5V->motor driver VCC
    Arduino GND->motor driver GND
    Power supply power and ground->motor driver VM and GND
    Quadature encoder A->Arduino pin 2
    Quadature encoder B->Arduino pin 3
*/
/* user configurable */
const double TARGET_POSITION_REVS = 1.0;        // the target position in revolutions, 64bit floating point
const double ENCODER_TICKS_PER_REV = 4741.44;   // the number of encoder ticks per revolution, 64bit floating point
const uint32_t LOOP_PERIOD_MILLIS = 10;          // period of the control loop in miliseconds, 32bit unsigned integrer
const double kp = 5.4;
const double ki = 0.000175;
const double kd = 0.00004375;
// const uint16_t PRINT_PERIOD_MILLIS = 10;        // how often to print the current encoder value in milliseconds
const uint32_t TEST_TIME_MILLIS = 2000;         // stop printing after this amount of time
/* pin defintions */
#define PWMA (9)    // output PWM signal
#define STBY (11)   // hbridge standby (enable) signal
#define AIN1 (12)   // motor direction 1
#define AIN2 (13)   // motor direction 2
#define ENC_A (2)   // encoder input A
#define ENC_B (3)   // encoder input B
/* program definitions */
static const int32_t target = ENCODER_TICKS_PER_REV * TARGET_POSITION_REVS;  // takes the user defined target_position and converts it to ticks
volatile int32_t count = 0;   // encoder step count, 32 bit volatile integer (volatile ensures the value is fetched everytime, necessary for global variables modified by interrupt handlers)
int32_t last_error = 0,       // the last error term, 32bit integer
        error_sum = 0;        // the current error term
uint32_t last_millis = 0,     // keep track of the last time the loop was executed
         print_millis = 0;    // keep track of the last time the data was printed
const int8_t encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
uint32_t stop_millis = 0;
// setup the kp, ki, and kd terms
// http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-sample-time/
// double sample_time_sec = ((double)LOOP_PERIOD_MILLIS)/1000;
// const uint32_t kp = KP_NUMERATOR / KP_DENOM;
// const uint32_t ki = (KI_NUMERATOR / KI_DENOM) * sample_time_sec;
// const uint32_t kd = (KD_NUMERATOR / KD_DENOM) / sample_time_sec;
// initialize, runs once on power up
void setup() {
  // start serial monitor
  Serial.begin(115200);
  Serial.print("Kp: ");
  Serial.println(kp);
  Serial.print(", Ki: ");
  Serial.println(ki);
  Serial.print(", Kd: ");
  Serial.println(kd);
  Serial.println(target);
  Serial.println(TEST_TIME_MILLIS);
  Serial.println(LOOP_PERIOD_MILLIS);
  // init the LED to signal we have completed
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize the h bridge control signals to outputs
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  // bring standby high to turn on the h bridge
  digitalWrite(STBY, HIGH);
  // setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoder_isr, CHANGE);
  // set the time used to stop printing test results
  stop_millis = millis() + TEST_TIME_MILLIS;  
}
// infinte run loop
void loop() {
    // calculate the PID output every LOOP_PERIOD_MILLIS
    if (millis() - last_millis > LOOP_PERIOD_MILLIS) {
        last_millis = millis();
        // determine the error according to encoder step count (proportional term)
        noInterrupts();
        int32_t error = target - count;
        interrupts();
        // derivative term
        int32_t d_error = (error - last_error); // http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
        // integral term
        error_sum += error;
        // calculate the output of the PID controller
        double output = (kp * error) + (ki * error_sum) + (kd * d_error);
        // update last error for the derivative term
        last_error = error;
        // check error and adjust motor direction
        if (count < target) { motorForward(); }
        if (count > target) { motorBackward(); }
        // write the output to PWMA
        analogWrite(PWMA, constrain(abs(output), 0, 255));
        // print out the output and count until the test ends
        if (millis() <= stop_millis) {
          Serial.print(constrain(abs(output), 0, 255));
          Serial.print(",");
          Serial.println(count);
        } else {
          Serial.println("end");
        }
    }
}
void motorForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}
void motorBackward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}
void motorBrake() {
  analogWrite(PWMA, 0);
}
void motorCoast() {
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, 0);
}
// encoder interrupt function, fires on change of either encoder input signal
// see tutorial/gif for description about how this function works
void encoder_isr() {
    static uint8_t enc_val = 0; // static allows this value to persist across function calls
    enc_val = enc_val << 2; // shift the previous state to the left
    enc_val = enc_val | ((PIND & 0b1100) >> 2); // or the current state into the 2 rightmost bits
    count += encoder_table[enc_val & 0b1111];    // preform the table lookup and increment count accordingly
}