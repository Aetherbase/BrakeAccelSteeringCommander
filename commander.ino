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

*/
#include<Servo.h>

#define dir_left 1
#define dir_right 2

#define PULSES_PER_ROTATION 120.00    //check encoder specifications
#define GEAR_RATIO 75.00              // check motor specifications


#define REL_ACTIVE LOW               // Relay driver Active edge mode(low level active)
#define REL_INACTIVE !REL_ACTIVE


int steerval;        //hold motor steering angle value
int ser1val, ser2val; // hold angle values of servo motor

int prevsteerval = 0, prevser1val = 0, prevser2val = 0;
float motorpos = 0.00;
float prevmotorpos = 0.00;
int prevdir, currdir;
int ecnt = 0; // count for encoder

#define MANUAL false
#define AUTO true

const int motordirpin = 8;     //motor first terminal
const int motorpwmpin = 9;     //motor second terminal
const int motoren = 10;      //motor driver enable pin
const int ser1pin = 4;       // accelerator servo
const int ser2pin = 3;       // brake servo

const int  encpin = 18; // encoder pins
const int relPin = 42; // relay pn to control input power
const int selfautobutton = 20; // button to chnge mode
int debouncecount=0; // required debounce count
int prevButtonState = 0,currentButtonState = 0; // record previous button state
boolean mode = MANUAL;

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
  pinMode(relPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(selfautobutton),changeState,CHANGE);
  digitalWrite(motoren, LOW); // disable motor
  ms1.attach(ser1pin); //accel
  ms2.attach(ser2pin); //brake
  encint();
}

void changeState()
{
  debouncecount = (debouncecount+1)%2;
  if(debouncecount == 1) // consider odd debounce count
  {
    mode = !mode; // toggle mode
    mode==AUTO?digitalWrite(relPin,REL_ACTIVE):digitalWrite(relPin,REL_INACTIVE); // decide state of relPin. If auto, turn relPin Active
  }
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
    ser2val = Serial.parseInt(); //accel
    ser1val = Serial.parseInt(); // brake
    steerval = Serial.parseInt(); //steer
    if ((prevser1val != ser1val) || (prevser2val != ser2val) || (prevsteerval != steerval)) Serial.print("Accel:" + (String)ser2val + "\tBrake:" + (String)ser1val + "\tSteering:" + (String)steerval);
    prevser2val = ser2val;
    prevser1val = ser1val;
    mode==MANUAL?Serial.println("\tMode:MANUAL"):Serial.println("\tMode:AUTOMATIC");
    if (Serial.read() == '\0')
    {
      Serial.println("Terminated");
      break;
    }
    switch (ser1val) // brake
    {
      case 0:
        ms2.write(66);
        break;

      case 1:
        ms2.write(37);
        break;

      case 2:
        ms2.write(24);
        break;

      case 3:
        ms2.write(10);
        break;

      default:
      ms2.write(66);
        break;
    }

    switch (ser2val) //accel
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
