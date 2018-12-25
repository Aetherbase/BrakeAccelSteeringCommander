#include<Servo.h>

#define dir_left 1
#define dir_right 2

#define PULSES_PER_ROTATION 120.00    //check encoder specifications
#define GEAR_RATIO 94.00              // check motor specifications

int steerval;        //hold motor steering angle value
int ser1val, ser2val; // hold angle values of servo motor

int prevsteerval = 0, prevser1val = 0, prevser2val = 0;
float motorpos = 0.00;
float prevmotorpos = 0.00;
int prevdir, currdir;
int ecnt = 0; // count for encoder


const int motordirpin = 8;     //motor first terminal
const int motorpwmpin = 9;     //motor second terminal
const int motoren = 10;      //motor driver enable pin
const int ser1pin = 4;       // accelerator servo
const int ser2pin = 3;       // brake servo

const int  encpin = 18; // encoder pins


char sep = ','; // separating character

Servo ms1;
Servo ms2;

String cc = "";
int v[3];
void setup()
{
  Serial.begin(9600);
  pinMode(motordirpin, OUTPUT);
  pinMode(motorpwmpin, OUTPUT);
  pinMode(motoren, OUTPUT);
  digitalWrite(motoren, LOW); // disable motor
  ms1.attach(ser1pin); //accel
  ms2.attach(ser2pin); //brake
  encint();
}


void encint()
{
  {
    pinMode(encpin, INPUT);
    attachInterrupt(digitalPinToInterrupt(encpin), enchandle, CHANGE);
  }
}


void enchandle()
{
  // detect position

  if (currdir == dir_left)ecnt--;
  else ecnt++;
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
  itervar = 0;
  return true;
}



void loop()
{
  while (Serial.available() > 0)
  {
    ser2val = Serial.parseInt();
    ser1val = Serial.parseInt();
    steerval = Serial.parseInt();
    if ((prevser1val != ser1val) || (prevser2val != ser2val) || (prevsteerval != steerval)) Serial.println("Accel:" + (String)ser2val + "\tBrake:" + (String)ser1val + "\tSteering:" + (String)steerval);
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
          turnleft((float)steerval);
        }
        if (steerval > prevsteerval)
        {
          currdir = dir_right;
          turnright((float)steerval);
        }
        prevsteerval = steerval;
    prevdir = currdir;
  }
}
