#include <Arduino.h>

#include "Wire.h"

#include "BlinkLed.h"
#include "pitches.h"


extern double x,y,theta, v, w;

typedef struct
{
  double vel_r;
  double vel_l;
} Vel;



BlinkLed blinkLed;
//BlinkMatrixLed blinkLed;
bool mROSConnected = false;

int sampleTime = 30; // (sample time 30 ms);
unsigned long prevSampleMillis;

void setup()
{

  Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(100);  
  Serial.begin(115200);
  delay(100);

  initMotor();
  initRobot();
  initController();
  initCommands();

  pinMode(13, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);


  blinkLed.init();
  blinkLed.normalBlink();

  Serial.println("READY!");

  interrupts();
 
  // bCount = 0;
  int melody[] = {
        NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4
    };
    int noteDurations[] = {
    4, 8, 8, 4, 4, 4, 4, 4
    };

  playMelody(melody, noteDurations, 8);

  #if OP_VOLTAGE == VOLT33
    Serial.println("Work of 3.3V...");
  #else
    Serial.println("Work at 5V...");
  #endif

  prevSampleMillis = millis();
}

void loop()
{
  checkCommands();
  // doBleHM10Loop();
  blinkLed.beSureToBlink();
  //ble cmd process
  // processSetingsRequire();
  unsigned long millisNow;
  millisNow = millis();

  if ( (millisNow - prevSampleMillis) >= sampleTime ) 
  {
      double dt = (double)(millisNow -  prevSampleMillis )/1000.0;
       prevSampleMillis = millisNow;

      updateRobotState(readLeftEncoder(), readRightEncoder(), dt);

      executeControl(v, theta, w, dt);      


        if( mROSConnected )
        {
          Serial.print("RP");
          Serial.print((int)(10000 * x));
          Serial.print(',');
          Serial.print((int)(10000 * y));
          Serial.print(',');
          Serial.print((int)(10000 * theta));
          Serial.print(',');
          Serial.print((int)(10000 * w));
          Serial.print(',');
          Serial.println((int)(10000 * v));
        }
  }

}



void stopRobot()
{
  blinkLed.normalBlink();
  // CurieTimerOne.kill();
  StopMotor();
  delay(100);
  updateRobotState(readLeftEncoder(), readRightEncoder(), 0.03); //处理当前运动的惯性
}


void setDriveGoal(double _v, double _w)
{
  driveRobot(_v, _w, theta);
}


void playMelody(int melody[], int noteDurations[], int len )
{
  
    for (int thisNote = 0; thisNote < len; thisNote++)
    {
      int noteDuration = 1000/noteDurations[thisNote];
      tone(11, melody[thisNote],noteDuration);
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      noTone(11);
    }

}
