/*
The ckt

Pin 4 = brake servo signal
pin 3 = accel servo signal
pin 18 = steering motor encoder input
pin 19 = manual-auto mode switch


+VCC
    O
    |
    /
    \    R = 330 ohms
    /
    \
    |
 |  o-----------Pin 19
0|  PushButton
 |  o------------
    |
    |
    |
    O
Ground


Entire coding works on interrupt, no Loop function is used

*/
#include<Servo.h>
#include<Wire.h>
#include<TinyGPS.h>

TinyGPS gps;
#define GPS Serial2
#define GPSEvent serialEvent2

//Attach arduino serial port connection with pc
//Attach serial2 connection with esp8266
#define dir_left 1
#define dir_right 2

#define PULSES_PER_ROTATION 120.00
#define GEAR_RATIO 75.0
#define factor 6.00

#define ACTIVE_RELAY LOW
#define INACTIVE_RELAY !ACTIVE_RELAY

#define NORTH true
#define EAST true
#define SOUTH false
#define WEST false


//pins definition

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15
#define SD3 10
#define SD2 9


boolean mode;
#define MANUAL false
#define AUTOMATIC true;

int steerval;        //hold motor steering angle value
int ser1val, ser2val; // hold angle values of servo motor

int interruptcall = 0;

int prevsteerval = 0, prevser1val = 0, prevser2val = 0, prevclutchval=0,clutchval=0;
float motorpos = 0.00;
float prevmotorpos = 0.00;
int prevdir, currdir;
int ecnt = 0; // count for encoder

#define MANUAL false
#define AUTOMATIC true;

const int manautorelay = 46; // manual auto relay
const int ser2pin = 5;       // brake servo
const int ser1pin = 6;       // accelerator servo
const int accrelay = 30; // control accessories pin
const int runcircuit = 32;  // control run circuit
const int ignitionrelay = 34; // control start circuit
const int motordirpina = 24;     //motor first terminal
const int motordirpinb = 26;     //motor first terminal
const int motorpwmpin = 11;     //motor second terminal
const int motoren = 22;      //motor driver enable pin
////pin 14,15 used for gps
const int  encpin = 18; // encoder pins
const int buttonpress = 2; // manual auto button
//// pin20 and 21 are used for mpu6050
const int motordrivpwmpin = 26;

int manautostate = LOW;
unsigned long lastdebtime = 0;
unsigned long debdelay = 10;
bool nn = false;


const int ignitionswitch = 40; // switch for gniition, active low
const int batteryVoltage = A6; // read battery voltage

const int gearshiftpin = 27;
const int clutchpin = 29;


char sep = ',';

Servo ms1;
Servo ms2;
Servo gearshiftservo;
Servo clutchservo;

String cc = "";
int v[3];

long accelX, accelY, accelZ;       //accelerometer values here
float gForceX, gForceY, gForceZ;   // gyro angle values here

long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;

float angelX = 0, angelY = 0, angelZ = 0;

long timePast = 0;
long timePresent = 0;




typedef struct vnh2sp30
{
  int currsense, en, da, db, pwm, dir;
  float currentval;
  const int MOTOR_CW = 1;
  const int MOTOR_CCW = 2;
  const int MOTOR_STOP = 0;
  void begin(unsigned int enb, unsigned int cs, unsigned int mma, unsigned int mmb, unsigned int pwmppin)
  {
    en = enb;
    currsense = cs;
    da = mma;
    db = mmb;
    pwm = pwmppin;
    pinMode(en, OUTPUT);
    pinMode(da, OUTPUT);
    pinMode(db, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(cs, INPUT);
  }

  float getCurrentInMilliAmps()
  {
    int val = analogRead(currsense);
    float ans = val * 5 * 11370 / 1500;
    return ans;
  }

  float getCurrentInAmps()
  {
    return getCurrentInMilliAmps() / 1000.00;
  }

  void write(int direction, int dutycycle)
  {
    int PWM = map(dutycycle, 0, 100, 0, 255);

    if (direction == MOTOR_CW)
    {
      digitalWrite(en, HIGH);
      digitalWrite(da, HIGH);
      digitalWrite(db, LOW);
      Serial.println("Again");
      analogWrite(pwm, PWM);
    }

    else if (direction == MOTOR_CCW)
    {
      digitalWrite(en, HIGH);
      digitalWrite(db, HIGH);
      digitalWrite(da, LOW);
      analogWrite(pwm, PWM);
    }

    else 
    {
      digitalWrite(en, HIGH);
      digitalWrite(da, LOW);
      digitalWrite(db, LOW);
      digitalWrite(pwm, LOW);
    }
  }
};

vnh2sp30 steeringmotor;
vnh2sp30 supply;


const int gearmin = 54;
const int gearmax = 140;


const int supplyen = 49;
const int supplyA = 45;
const int supplyB = 47;
const int supplyPWM = 13;




void setup()
{
  clutchservo.attach(clutchpin);
  gearshiftservo.attach(gearshiftpin);
  steeringmotor.begin(motoren, 70, motordirpina, motordirpinb, motorpwmpin);
  Serial.begin(9600);
  GPS.begin(9600);
  //  pinMode(buttonpress,INPUT);
  ms1.attach(ser1pin); //accel
  ms2.attach(ser2pin); //brake
  encint();
  ignitioninit();
  manualautobuttoninit();
  runcircuitinit();
  accessoriesinit();
  voltagereaderinit();
  supply.begin(supplyen,78,supplyA,supplyB,supplyPWM);
  supply.write(1,56);
}
// battery voltage reader module

void voltagereaderinit()
{
  pinMode(batteryVoltage, INPUT);
}
float readVoltage()
{
  int count = analogRead(batteryVoltage);
  float ans = (float)count * 5.00 / 1023.00;
  return ans * 4.00;
}

bool isEngineStarted()
{
  if (readVoltage() >= 13.60)
  {
    Serial.println("Engine Start");
    return true;
  }
  return false;
}

bool isBatteryLow()
{
  if (readVoltage() <= 11.30) {
    Serial.println("LOW");
    return true;
  }
  return false;
}

//run circuit
void runcircuitinit()
{
  pinMode(runcircuit, OUTPUT);
  runcircuitoff();
}

void runcircuitoff()
{
  digitalWrite(runcircuit, INACTIVE_RELAY);
}

void runcircuiton()
{
  digitalWrite(runcircuit, ACTIVE_RELAY);
}

//accessories
void accessoriesinit()
{
  pinMode(accrelay, OUTPUT);
  accessorieson();
}

void accessorieson()
{
  digitalWrite(accrelay, ACTIVE_RELAY);
}

void accessoriesoff()
{
  digitalWrite(accrelay, INACTIVE_RELAY);
}


//ignition
void ignitioninit()
{
  pinMode(ignitionswitch, INPUT);
  pinMode(ignitionrelay, OUTPUT);
  digitalWrite(ignitionrelay, INACTIVE_RELAY);
}

void ignitionstart()
{
  accessoriesoff();
  runcircuiton();
  digitalWrite(ignitionrelay, ACTIVE_RELAY);
}

void ignitionstop()
{
  accessorieson();
  digitalWrite(ignitionrelay, INACTIVE_RELAY);
}

// manual-auto




//gyro

void gyroprocess()
{ // call this functon to get values from gyro
  readAndProcessAccelData();
  readAndProcessGyroData();
  // process your values here
}

void setUpMPU() {
  // power management
  Wire.begin();
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();
  callibrateGyroValues();
}

void callibrateGyroValues() {
  for (int i = 0; i < 5000; i++) {
    getGyroValues();
    gyroXCalli = gyroXCalli + gyroXPresent;
    gyroYCalli = gyroYCalli + gyroYPresent;
    gyroZCalli = gyroZCalli + gyroZPresent;
  }
  gyroXCalli = gyroXCalli / 5000;
  gyroYCalli = gyroYCalli / 5000;
  gyroZCalli = gyroZCalli / 5000;
  timePresent = millis();
}

void readAndProcessAccelData() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void readAndProcessGyroData() {
  gyroXPast = gyroXPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroYPast = gyroYPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time

  getGyroValues();                                            // get gyro readings
  getAngularVelocity();                                       // get angular velocity
  calculateAngle();                                           // calculate the angle
}

void getGyroValues() {
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);                             // Request for 6 bytes from gyro registers (43 - 48)
  while (Wire.available() < 6);                               // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read() << 8 | Wire.read();              // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read() << 8 | Wire.read();              // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read() << 8 | Wire.read();              //Store last two bytes into gyroZPresent
}

void getAngularVelocity() {
  rotX = gyroXPresent / 131.0;
  rotY = gyroYPresent / 131.0;
  rotZ = gyroZPresent / 131.0;
}

void calculateAngle() {
  // same equation can be written as
  // angelZ = angelZ + ((timePresentZ - timePastZ)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) / (2*1000*131);
  // 1/(1000*2*131) = 0.00000382
  // 1000 --> convert milli seconds into seconds
  // 2 --> comes when calculation area of trapezium
  // substacted the callibated result two times because there are two gyro readings
  angelX = angelX + ((timePresent - timePast) * (gyroXPresent + gyroXPast - 2 * gyroXCalli)) * 0.00000382;
  angelY = angelY + ((timePresent - timePast) * (gyroYPresent + gyroYPast - 2 * gyroYCalli)) * 0.00000382;
  angelZ = angelZ + ((timePresent - timePast) * (gyroZPresent + gyroZPast - 2 * gyroZCalli)) * 0.00000382;
}
void encint()
{
  {
    pinMode(encpin, INPUT);
    attachInterrupt(digitalPinToInterrupt(encpin), enchandle, CHANGE);
  }
}




// manual auto

void manualautobuttoninit()
{
  pinMode(manautorelay, OUTPUT);
  //pinMode(buttonpress,INPUT);
   attachInterrupt(digitalPinToInterrupt(buttonpress), bsr, CHANGE);
}

void autorelay()
{
  digitalWrite(manautorelay, ACTIVE_RELAY);
  gearshiftservo.write(gearmin);
}
void manrelay()
{
  gearshiftservo.write(gearmax);
  digitalWrite(manautorelay, INACTIVE_RELAY);
}


long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
void bsr()
{
  manautostate = digitalRead(buttonpress);
  if (manautostate == HIGH) {
    Serial.println("M");  //manual
    
    manrelay();
  }
  else {
    Serial.println("A");  //auto
    autorelay();
  }
}

void enchandle()
{
  // detect position

  if (currdir == dir_left) {
    ecnt--;
  }
  else {
    ecnt++;
  }
  // do position calculation here
  motorpos = (360.00 / GEAR_RATIO) * (ecnt / PULSES_PER_ROTATION);
  //Serial.println(motorpos);

}

bool turnright(float angle)
{
  while (motorpos < angle)
  {
    steeringmotor.write(steeringmotor.MOTOR_CW, 100);
    Serial.println(motorpos);
  }
  Serial.println("right:" + (String)motorpos);
  Serial.println("New motorpos :" + (String)motorpos);
  steeringmotor.write(steeringmotor.MOTOR_STOP,100);
  //prevmotorpos = motorpos;
  return true;
}



bool turnleft(float angle)
{
  while ((motorpos) > angle)
  {
    steeringmotor.write(steeringmotor.MOTOR_CCW, 100);
    Serial.println("called");
  }
  //angle processing code here
  //while()
  // after procesing value
  Serial.println("LEft:" + (String)motorpos);
  Serial.println("New motorpos :" + (String)motorpos);
  steeringmotor.write(steeringmotor.MOTOR_STOP, 100);
  return true;
}

void loop()
{
  //isEngineStarted();
  //isBatteryLow();
  int a = digitalRead(ignitionswitch);
  a == LOW ? ignitionstart() : ignitionstop();




  bool newData = false;
  unsigned long chars;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    bsr();
    while (GPS.available())
    {
      bsr();
      char c = GPS.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("Positon:  ");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //String msg = "Positon:  " + 
  }
}





// Get data from PC
void serialEvent() // process new parameters when data is there in Serial (from ESP8266)
{
  ser1val = Serial.parseInt(); // accel
  ser2val = Serial.parseInt();  //brake
  steerval = Serial.parseInt(); // steer
  clutchval = Serial.parseInt(); //clutch
  int horn = Serial.parseInt(); //horn
  //buttonhandle();
  if ((prevser1val != ser1val) || (prevclutchval != clutchval) || (prevser2val != ser2val) || (prevsteerval != steerval)) Serial.print("Accel:" + (String)ser2val + "\tBrake:" + (String)ser1val + "\tSteering:" + (String)steerval);

  prevser2val = ser2val;
  prevser1val = ser1val;
  if (Serial.read() == '\0')
  {
    Serial.println("Terminated");
  }

  //accel
  switch (ser1val)
  {
    case 0:
      ms2.write(0);
      break;

    case 1:
      ms2.write(20);
      break;

    case 2:
      ms2.write(60);
      break;

    case 3:
      ms2.write(120);
      break;

    default:
      ms2.write(0);
      break;
  }

//brake
  switch (ser2val)
  {
    case 0:
      ms1.write(42);
      break;

    case 1:
      ms1.write(61);//just start
      break;

    case 2:
      ms1.write(72); // medium speed
      break;

    case 3:
      ms1.write(81); //full
      break;

    default:
      ms1.write(42);
      break;
  }


  //steering commander
  if (steerval < prevsteerval) {
    currdir = dir_left;
    turnleft((float)steerval * factor);
  }
  if (steerval > prevsteerval)
  {
    currdir = dir_right;
    turnright((float)steerval * factor);
  }
  prevsteerval = steerval;
  prevdir = currdir;

  //clutch control
  clutchval = map(clutchval,0,100,54,80);
  clutchservo.write(clutchval);
  prevclutchval = clutchval;

  //horn
  Serial.print("\tHorn");
}
