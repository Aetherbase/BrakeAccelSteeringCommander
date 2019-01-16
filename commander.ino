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

#define dir_left 1
#define dir_right 2

#define PULSES_PER_ROTATION 120.00
#define GEAR_RATIO 75.0
#define factor 6.00

#define NORTH true
#define EAST true
#define SOUTH false
#define WEST false

#define ACTIVE_RELAY LOW
#define INACTIVE_RELAY !ACTIVE_RELAY


int steerval;        //hold motor steering angle value
int ser1val, ser2val; // hold angle values of servo motor

int interruptcall = 0;

int prevsteerval = 0, prevser1val = 0, prevser2val = 0;
float motorpos = 0.00;
float prevmotorpos = 0.00;
int prevdir, currdir;
int ecnt = 0; // count for encoder

const int manautorelay = 2; // control manual auto mode power
const int ser2pin = 3;       // brake servo
const int ser1pin = 4;       // accelerator servo
const int accrelay = 5; // control accessories pin
const int runcircuit = 6;  // control run circuit
const int ignitionrelay = 7; // control start circuit
const int motordirpin = 8;     //motor first terminal
const int motorpwmpin = 9;     //motor second terminal
const int motoren = 10;      //motor driver enable pin
//pin 14,15 used for gps
const int  encpin = 18; // encoder pins
const int buttonpress = 19; // manual auto button
// pin20 and 21 are used for mpu6050



int manautostate=LOW;
unsigned long lastdebtime = 0;
unsigned long debdelay = 10;
bool nn=false;


const int ignitionswitch = A0; // switch for gniition, active low


char sep = ',';

Servo ms1;
Servo ms2;

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


void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(motordirpin, OUTPUT);
  pinMode(motorpwmpin, OUTPUT);
  //  pinMode(buttonpress,INPUT);
  pinMode(motoren, OUTPUT);
  digitalWrite(motoren, LOW); // disable motor
  ms1.attach(ser1pin); //accel
  ms2.attach(ser2pin); //brake
  //encint();
  ignitioninit();
  manualautobuttoninit();
  runcircuitinit();
  accessoriesinit();
}

//run circuit
void runcircuitinit()
{
  pinMode(runcircuit,OUTPUT);
  runcircuitoff();
}

void runcircuitoff()
{
  digitalWrite(runcircuit,INACTIVE_RELAY);
}

void runcircuiton()
{
  digitalWrite(runcircuit,ACTIVE_RELAY);
}

//accessories
void accessoriesinit()
{
  pinMode(accrelay,OUTPUT);
  accessorieson();
}

void accessorieson()
{
  digitalWrite(accrelay,ACTIVE_RELAY);
}

void accessoriesoff()
{
  digitalWrite(accrelay,INACTIVE_RELAY);
}


//ignition
void ignitioninit()
{
  pinMode(ignitionswitch,INPUT);
  pinMode(ignitionrelay,OUTPUT);
  digitalWrite(ignitionrelay,INACTIVE_RELAY);
}

void ignitionstart()
{
  accessoriesoff();
  runcircuiton();
  digitalWrite(ignitionrelay,ACTIVE_RELAY);
}

void ignitionstop()
{
  accessorieson();
  digitalWrite(ignitionrelay,INACTIVE_RELAY);
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

void manualautobuttoninit()
{
  pinMode(manautorelay,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonpress), bsr, CHANGE);
}

void autorelay()
{
  digitalWrite(manautorelay,ACTIVE_RELAY);
}
void manrelay()
{
  digitalWrite(manautorelay,INACTIVE_RELAY);
}


long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
void buttonhandle()
{
  if((long)(micros() - last_micros) >= debouncing_time * 1000) {
    bsr();
    last_micros = micros();
  }
}

void bsr()
{
  manautostate = digitalRead(buttonpress);
  if(manautostate==HIGH){Serial.println("    MODE0");manrelay();} //manual
  else {Serial.println("    MODE1");autorelay();} //auto
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
}

int itervar = 0;
bool turnright(float angle)
{
  Serial.println("Rightangle:" + (String)(angle));
  Serial.println("Motorpos:" + (String)motorpos);
  //angle processing code here
  while ((motorpos) < angle)
  {
    digitalWrite(motordirpin, HIGH);
    for (; itervar < 255; itervar++)
    {
      analogWrite(motorpwmpin, itervar);
      delay(1);
    }
    if (itervar == 255) digitalWrite(motorpwmpin, HIGH);
    digitalWrite(motoren, HIGH);
  }
  digitalWrite(motordirpin, LOW);
  digitalWrite(motorpwmpin, LOW);
  digitalWrite(motoren, LOW);
  itervar = 0;
  Serial.println("right:" + (String)motorpos);
  Serial.println("New motorpos :" + (String)motorpos);
  //prevmotorpos = motorpos;
  return true;
}



bool turnleft(float angle)
{
  Serial.println("Leftangle:" + (String)(angle));
  Serial.println("Motorpos:" + (String)motorpos);
  digitalWrite(motoren, HIGH);
  while ((motorpos) > angle)
  {
    digitalWrite(motordirpin, LOW);
    for (; itervar < 255; itervar++)
    {
      analogWrite(motorpwmpin, itervar);
      delay(1);
    }
    if (itervar == 255) digitalWrite(motorpwmpin, HIGH);
  }
  //angle processing code here
  //while()
  // after procesing value
  digitalWrite(motordirpin, LOW);
  digitalWrite(motorpwmpin, LOW);
  digitalWrite(motoren, LOW);
  Serial.println("LEft:" + (String)motorpos);
  Serial.println("New motorpos :" + (String)motorpos);
  itervar = 0;
  return true;
}

void loop()
{
  int a = digitalRead(ignitionswitch);
  a==LOW?ignitionstart():ignitionstop();
}

void serialEvent() // process new parameters when data is there in Serial
{
  while (Serial.available() > 0)
  {
    ser2val = Serial.parseInt();
    ser1val = Serial.parseInt();
    steerval = Serial.parseInt();
    //buttonhandle();
    if ((prevser1val != ser1val) || (prevser2val != ser2val) || (prevsteerval != steerval)) Serial.print("Accel:" + (String)ser2val + "\tBrake:" + (String)ser1val + "\tSteering:" + (String)steerval);

    prevser2val = ser2val;
    prevser1val = ser1val;
    if (Serial.read() == '\0')
    {
      Serial.println("Terminated");
      break;
    }
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
  }
}

String inputString="";
bool stringComplete = false;
String signal = "$GPGLL";
void serialEvent1()
{
    while (Serial1.available()) {
    // get the new byte:
    char inChar = (char) Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  double lat, lon;
  boolean latpos, lonpos;
  if (stringComplete) {
    String BB = inputString.substring(0, 6);
    if (BB == signal) {
      String LAT = inputString.substring(7, 16);
      int LATperiod = LAT.indexOf('.');
      int LATzero = LAT.indexOf('0');
      if (LATzero == 0) {
        LAT = LAT.substring(1);
      }
      if (inputString.substring(18) == 'N')
      {
        latpos = NORTH;
      }
      else
      {
        latpos = SOUTH;
      }

      String LON = inputString.substring(20, 29);
      int LONperiod = LON.indexOf('.');
      int LONTzero = LON.indexOf('0');
      if (LONTzero == 0) {
        LON = LON.substring(1);
      }
      if (inputString.substring(31) == 'E')
      {
        lonpos = EAST;
      }
      else
      {
        lonpos = WEST;
      }
      
      //Parse string to process data
      lat = LAT.toFloat() / (float)100;
      lon = LON.toFloat() / (float)100;
      
      //Convert NMEA data to degrees
      float latitude = (int)lat + ((100 * ((lat - (int)lat) / (float)60))*(1)); //latitude in degrees   latpos==NORTH?1:-1
      float longitude = (int)lon +  ((100 * ((lon - (int)lon) / (float)60))*(1)); //longitude in degrees   lonpos==EAST?1:-1
      Serial.print("Position: ");
      Serial.print(latitude, 6);
      Serial.print(" " + (latpos == NORTH) ? "N   " : "S   ");
      Serial.print(longitude, 6);
      Serial.println(" " + (lonpos == EAST) ? "E" : "W");

    }

    // Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

}
