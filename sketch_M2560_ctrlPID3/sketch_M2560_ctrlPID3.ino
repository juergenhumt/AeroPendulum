/* Copyright 2025 Juergen Humt
# 
# This file is part of AeroPendulum.
# 
#
#     AeroPendulum, is free  software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by the 
#     Free Software Foundation, either version 3 of the License or any later 
#     version.
# 
#     AeroPendulum is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License along 
#     with AeroPendulum.  If not, see <http://www.gnu.org/licenses/>.
#
#     version v0.1.0
*/


#include <Servo.h>
#include <SKalmanF.h>
#include <PID_v1.h>
// use rdSerTm3.py to record data on PC
 
Servo escUnit;          //create servo object

int ledPin = 11;
int pwmPin = 10;
int snsPn  = A0;

int initVal, j, mOut, mX, mCur;
int jsVal = 260;

int snsVal = 0;

const long SERIAL_REFRESH_TIME = 50;
long refresh_time;

float maxVal, oldEstVal=0, tDmy=0, fInp, fOut, fOffst;

volatile int changedx = 0;
volatile int jSrvRun = 1;
volatile int curFnc=0, kT=0;


SimpleKalmanFilter skf(2, 2, 0.012);

//Define Variables we'll be connecting to
double pidInp, pidOut, pidMin, pidMax, cSc;

//double Kp, Ki, Kd, pidSetPT;

//Specify the links and initial tuning parameters
// double Kp= 0.125, Ki= 0.03, Kd= 0.075, pidSetPT= 180;  // rising slowly
double Kp= 0.041, Ki= 0.21, Kd= 0.006, pidSetPT= 350;  // stable with overshoot (=swo)  #1
// double Kp= 4.2, Ki= 0.586,  Kd= 1.5,   pidSetPT=  150;            #2 
// double  Kp= 0.1, Ki= 0.4, Kd= 0.05, pidSetPT= 75;
// double  Kp= 0.3, Ki= 0.4, Kd= 0.0, pidSetPT= 250;
// double  Kp= 0.062, Ki= 0.0, Kd= 0.0, pidSetPT= 250;

// double Kp= 0.0372, Ki= 0.06, Kd= 0.0058, pidSetPT=  250;
// double Kp= 0.135, Ki= 0.21, Kd= 0.008, pidSetPT=  300;   // 6V stable (wo)
// double Kp= 0.135, Ki= 0.21, Kd= 0.08,  pidSetPT=  350;      // saw tooth due to Kd 
// double Kp= 0.135, Ki= 0.21,  Kd= 0.008,   pidSetPT=  350;      // more saw teeth due to smaller Kd 
// double Kp= 0.135, Ki= 0.05,  Kd= 0.0095,   pidSetPT=  300;      // more saw teeth due to smaller Kd 
// double Kp= 0.3537, Ki= 0.3277, Kd= 0.0, pidSetPT= 300;  // stable after three osciallations
// double Kp= 0.184, Ki= 0.2388, Kd= 0.0, pidSetPT= 300;  // stable after three osciallations
// +++ 
// double Kp= 0.383, Ki= 0.805, Kd= 0.455,  pidSetPT=  250;   //
// double Kp= 0.025, Ki= 0.065,  Kd= 0.1,   pidSetPT=  250;   // slowly rising with saw tooth profile

// values below give a pretty sustained osciallation while a 
// value of kp= 0.145 is divergent.
// double Kp= 0.1425, Ki= 0.05,  Kd= 0.0,   pidSetPT=  250;  
// double Kp= 0.1425, Ki= 0.05,  Kd= 0.0,   pidSetPT=  250;  
// double Kp= 0.009, Ki= 0.0, Kd= 0.0,  pidSetPT=  250;   //  divergent oscillation
// double Kp= 0.005, Ki= 0.01, Kd= 0.0,  pidSetPT=  200;   //  stable very slowly rising with small oscillation
// double Kp= 0.005, Ki= 0.01, Kd= 0.0,  pidSetPT=  200;   // 
// double Kp= 7.7, Ki= 14.4, Kd= 1.03,  pidSetPT=  250;   // 


PID   myPID(&pidInp, &pidOut, &pidSetPT, Kp, Ki, Kd, DIRECT);
// myPID.SetOutputLimits(-255, 255);  // or whatever your motorPWM range is


// volatile byte state = HIGH;
typedef enum {Func_A = 0, Func_B} Func_type;
Func_type curr_func, curr_srv;

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;
FLOATUNION_t myFloat;



void sPrn_ML(float tFlt, float msrdVal, float estVal) {
  // Matlab expects just two float values, no separator
  //  srlFltWrt(tFlt);  
  //  Serial.print('\t');
  srlFltWrt(msrdVal);
  // if data are transmitted to simulink
  // a tab separator is misinterpreted as
  // a small number e.g. 2.32e-31
  //Serial.print('\t');
  srlFltWrt(estVal);
  Serial.print('\n');
  
}

void sPrn_py(float tFlt, float msrdVal, float estVal) {
// serial print routine working with python program rdSerSmpl.py
    Serial.print(tFlt,5);
    Serial.print(",");
    Serial.print(msrdVal,10);
    Serial.print(",");
    Serial.print(estVal,10); 
    Serial.print('\n');
}



void sPrn_x(float tFlt, float msrdVal, float estVal) {
  srlFltWrt(tFlt);  
  Serial.print('\t');
  srlFltWrt(msrdVal);
  Serial.print('\t');
  srlFltWrt(estVal);
  Serial.print('\n');
  
} 

 void srlFltWrt(float fOut) {
    myFloat.number= fOut;
    for (int i=0; i<4; i++)   {
      Serial.write(myFloat.bytes[i]); 
    }
 }  

 int srvVal(int mCur) {
   return mCur;
 }

 
 int srvValNull(int mCur) {
   int mOut = 130;
   return mOut;
 }

 int (*srvFnc)(int) = srvVal;
 int (*srvFncN)(int) = srvVal;
 int (*srv_table[])(int) = {srvFnc, srvFncN};
 
 void runFnc() {
    int mOut;
    snsVal = fOffst - analogRead(snsPn);
    float estVal = skf.updateEstimate(snsVal);

    if (abs(estVal) > maxVal)
      estVal = oldEstVal;
    else
      oldEstVal = estVal;        
        
    pidInp = estVal;
    myPID.Compute();
    fOut = pidOut;

    // send to Serial output every 100ms
    // use the Serial Ploter for a good visualization
    if (millis() > refresh_time) {
      sPrn_py(tDmy, fOut, estVal);
      refresh_time = millis() + SERIAL_REFRESH_TIME;
      tDmy++;         
    }

    mOut = floor(fOut);
    mOut = srv_table[curFnc](mOut);
    // uncomment line below for constant signal to servo +=+=+=
    // mOut = 130;
    escUnit.write(mOut);        

 }


 void stopFnc() {
   mOut = 130;
   escUnit.write(mOut);
   float t77 = 77777.77;
   tDmy= 9999.1999; 
   float tmX = 9999.3999;
   for (j= 0; j< 3; j++)
     sPrn_py(t77, tDmy, tmX);
   
   delay(1000);
 }

void (*fnc_table[])() = {runFnc, stopFnc};


void setup() {
  
  stopFnc();
  pinMode(ledPin, OUTPUT);
  curFnc=0;
  // initialize serial:
  Serial.begin(9600);

  escUnit.attach(9);

  cSc = 1.0;
  Ki=cSc*Ki;
  // Kp = 0.05;
  // Kd = 0.015;
  
  // set PID values
  myPID.SetTunings(Kp, Ki, Kd);
  
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  pidMin= 130;
  pidMax= 270;
  fOffst= 670;
  myPID.SetOutputLimits(pidMin, pidMax);

  
  for (j = 0; j < jsVal; j = j + 1) {
    snsVal = fOffst - analogRead(snsPn);
    float estVal = skf.updateEstimate(snsVal);
    delay(10); 
  }   
  digitalWrite(ledPin,HIGH);  
  attachInterrupt(1, srvStop, RISING);
  maxVal = 1200;
  refresh_time = millis() + SERIAL_REFRESH_TIME;

}


void loop()
{
  // indicate start  
 if (changedx == 1) {
      delay(5); //waits 5ms to avoid potential voltage spike on button depress.
      if (digitalRead(3) == HIGH)
      {
        changedx = 0;
      }
  }
  
  fnc_table[curFnc]();  
 
}  // end loop



 void sndVal(int mOutVal) {
    // Send a float
    myFloat.number = mOutVal;
    
    for (int i=0; i<4; i++)   {
      Serial.write(myFloat.bytes[i]); 
    }
    Serial.print('\n');
 }



 void srvStop()
    {
      if (changedx == 0) //This makes sure the contents of the loop() function
      //can't be re-triggered thru the interrupt if user mashes the button.
      {
        noInterrupts();
        digitalWrite(ledPin, LOW);            
        changedx =  1;
        curFnc = 1;
        interrupts();
      }
 }
