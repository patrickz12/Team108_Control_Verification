#include <Servo.h>
#include <Wire.h>
#include <avr/sleep.h>

#define address 0x1E  //0011110b, I2C 7bit address of HMC5883
#define ServoPin 6
#define SolenoidPin 10
#define Reed 2 
#define Switch 2
#define deltaSw 2.4737 // (in inches per reed trigger) Circumference/#Magnets or 2*pi*wheelRaduis/numMagnets, MUST BE A DECIMAL NUMBER!
#define PotPin A0
#define buttPin 5 //Button to start executing code is in pin 5
#define startPin 0 // Button to define starting positions

Servo myServo;                      // compass servo

/*----- VARIABLE INITIALIZATION -----*/

/* Magnetometer Variables*/
int   magX, magY, magZ;             // triple axis data
float phi;                        // orientation of the robot based on magnetometer readings
float thetaOffset = -139;     // offset: value of -191 makes magnetic north 0 degrees, value of -139 makes down the channel 0 degrees

/* Orientation Variables */
float phi_desired = 90;    // desired orientation of robot (can be set or be vary)
/* For the Control Verification the value for phi_desired is arbitrarily set.
 *  in the actual code, the desired orientation of the robot will vary based on 
 *  other variables (i.e. the desired position of the robot)
 */

float theta_target;   // desired steering angle

/* Servo Position MIN/MAX */
int minServoPosition = 60;
int maxServoPosition = 120;

/* Solenoid Variables */
int solenoidState = 0; // zero is closed (valve)
unsigned long lastTimeSolenoid;
int solenoidInterval = 200; // (in milliseconds) sets time interval at which solenoid fires

/* Time Variables */
unsigned long startTime;
unsigned long currentTime;

/* Reed Variables */
int lastTimeReed = currentTime;

float wheelSpeed = 0; 
float wheelSpeedCM;
float wheelSpeedSEC;
int numReedTriggers = 0;

volatile unsigned long lastInterrupt;
long debounceTime = 15;

/*Robot Specification Variables*/
const float pi = 3.14159;
float a = 5.5; // (in inches) distance between rear wheels
float b = 5.5; // (in inches) distance between rear axle and front wheel 
float backWheelRadius = 1.5748; // (in inches) roller blade back wheel radius
float theta_max = 30; // maximum change in servo angle (relative to center)

/*Position Variables*/
float distanceX = 0; // (in inches) distance traveled in the x-direction
float distanceY = 0; // (in inches) distance traveled in the y-direction
float distance = 0; // (in inches) distance traveled
float currentPosX; // current x-position
float currentPosY; // current y-position 
float startingPosX = 0; // starting x-position (varies in competition code, arbitrarily set for verification)
float startingPosY = 0; // starting y-position (varies in competition code, arbitrarily set for verification)

/*----- SETUP -----*/

void setup() {
   myServo.attach(ServoPin); // Attach the servo to pin ServoPin

   /* Initialize Serial and I2C communications */
   Serial.begin(9600);
   Wire.begin();
   /* Put the HMC5883 IC into the correct operating mode */
   Wire.beginTransmission(address);  //open communication with HMC5883
   Wire.write(0x02);                 //select mode register
   Wire.write(0x00);                 //continuous measurement mode
   Wire.endTransmission();

   pinMode (Reed,INPUT_PULLUP); //reads 5V until switch turns off, "unfloat" the switch when open for a relfiable reading, note: logic is inverted
   attachInterrupt(digitalPinToInterrupt(Switch),debounceInterrupt,FALLING); // attaches the ISR to the reed switch trigger

   startTime = millis();
}

/*----- MAIN LOOP: ACTUATION AND SIMPLE ORIENTATION -----*/

void loop() {
  currentTime = millis(); //measures current time of the system
  
  if (distance > 120){ // Stop actuating after traveling 10 ft
    cli();
    sleep_enable();
    sleep_cpu();
  }
  /* Update the sensor readings and actuate cylinder */   
  updateMag();
  updateSolenoid(); 

  /* Calculate magnetometer angle */
  phi = 180. / pi * atan2(magY, magX) + thetaOffset; // convert to degrees, apply offset
  phi = modulo(phi, 360.); // ensure that theta is between 0 and 360 degrees

  /* Update Servo Position */
  theta_target = 90 - (phi - phi_desired);
  theta_target = modulo(theta_target,360);
  
  /* Corrects theta_target to be within the feasible range of the steering assembly */
  if (theta_target > 270 || theta_target < minServoPosition){
    theta_target = minServoPosition;
  }
  else if (theta_target > maxServoPosition){
    theta_target = maxServoPosition;
  }

  myServo.write(theta_target); // inputs desired steering angle 
}

/*----- INTERRUPT SERVICE ROUTINE CODE -----*/

/************************* [DEBOUNCE INTERRUPT ROUTINE] *************************/
void debounceInterrupt() {
  if((long)(micros() - lastInterrupt) >= debounceTime * 1000) { 
    // checks time since last interrupt, if < debounceTime do nothing, if > debounceTime call readReed()
    readReed();
    lastInterrupt = micros();
  }
}

/************************* [REED SWTICH READER] *************************/
void readReed() {
  currentTime = millis(); 
  int timeDiff = currentTime - lastTimeReed;
  wheelSpeed = deltaSw / timeDiff; // (in inches per millisecond)
  wheelSpeedCM = wheelSpeed * 2.54; // convert (in per ms) to (cm per ms)
  wheelSpeedSEC = wheelSpeedCM / .001; //convert to cm/s
  numReedTriggers++; // number of times a magnet has passed the reed switch
  //Serial.print("Speed (cm/s):"); Serial.print("\t"); 
  Serial.println(wheelSpeedSEC);
  //Serial.print("Distance Traveled:"); Serial.print("\t"); Serial.println(numReedTriggers*deltaSw); // print distance traveled
  
  /*Print for Troubleshooting*/
 /* Serial.print("deltaSw:"); Serial.print("\t"); Serial.println(deltaSw);
  Serial.print("timeDiff:"); Serial.print("\t"); Serial.println(timeDiff);
  Serial.print("wheelSpeed:"); Serial.print("\t"); Serial.println(wheelSpeed);
  Serial.print("numReedTriggers:"); Serial.print("\t"); Serial.println(numReedTriggers); */

  estimate(); //call estimate function to update position each time the reed triggers
  lastTimeReed = currentTime;
}

/************************* [POSITION ESTIMATOR] *************************/
void estimate(){
  /*assume deltaSw is >>1, therefore neglect curvature and use trig. to estimate x & y distances traveled*/
  distanceX += cos(phi) * deltaSw; // updates distance traveled in x-direction since last reed trigger
  distanceY += sin(phi) * deltaSw; // updates distance travled in y-direction since last reed trigger
  distance += deltaSw; // update distance traveled since last reed trigger
  currentPosX += distanceX; // updates the new current x-position of the robot (NEW)
  currentPosY += distanceY; // updates the new current y-position of the robot (NEW)
  
  float distanceCM = (distance * 2.54); //convert distance traveled from inches to cm
  //Serial.print("Distance Traveled (in cm):"); Serial.print("\t"); Serial.println(distanceCM);
}

/*----- MAGNETOMETER FUNCTIONS -----*/

/************************* [updateMag FUNCTION] *************************/
void updateMag(void) {
  /* Tell the HMC5883L where to begin reading data */
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  /* Read data from each axis, 2 registers per axis */
  Wire.requestFrom(address, 6);
  if (6 <= Wire.available()) {
    magX = Wire.read() << 8; //X msbs // Bitshift
    magX |= Wire.read();    //X lsb
    magZ = Wire.read() << 8; //Z msb
    magZ |= Wire.read();    //Z lsb
    magY = Wire.read() << 8; //Y msb
    magY |= Wire.read();    //Y lsb
  }

  /* print for troubleshooting */
    //Serial.print("X:"); Serial.print("\t"); Serial.print(magX); Serial.print("\t");
    //Serial.print("Y:"); Serial.print("\t"); Serial.print(magY); Serial.print("\t");
    //Serial.print("Z:"); Serial.print("\t"); Serial.print(magZ); Serial.print("\t");*/


}

/************************* [MODULO FUNCTION FOR FLOATS] *************************/
float modulo(float x, float y) {
  float n = floor(x/y); // find quotient rounded down
  return x - n*y; // return the remainder of the divison
}

/*----- PISTON/SOLENOID FIRE FUNCTION -----*/

/************************* [updateSolenoid FUNCTION] *************************/
void updateSolenoid(){ 
    if ((currentTime - lastTimeSolenoid) >= solenoidInterval && solenoidState == HIGH) {
      solenoidState = LOW;
      digitalWrite(SolenoidPin, solenoidState); // Opens the solenoid valve
      lastTimeSolenoid = currentTime;
      //Serial.println("SolenoidOff");
    } 
    if ((currentTime - lastTimeSolenoid) >= solenoidInterval && solenoidState == LOW) {
      solenoidState = HIGH; 
      digitalWrite(SolenoidPin, solenoidState); // Closes the solenoid valve
      lastTimeSolenoid = currentTime;
      //Serial.println("SolenoidOn");
    }
    //Serial.println("Solenoid");
    //Serial.print("solenoidInterval:"); Serial.print("\t"); Serial.println(solenoidInterval);
}
