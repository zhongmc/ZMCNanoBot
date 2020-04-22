#include <Arduino.h>

#include "Wire.h"

#include "BlinkLed.h"
#include "pitches.h"

#define OP_VOLTAGE VOLT5

void InitAdfMotor(uint8_t leftChannel, uint8_t rightChannel ); //channel 1-4

extern double x,y,theta, v, w;

typedef struct
{
  double vel_r;
  double vel_l;
} Vel;



BlinkLed blinkLed;
//BlinkMatrixLed blinkLed;
bool mROSConnected = false;

bool mBleConnected = false;


int sampleTime = 30; // (sample time 30 ms);
unsigned long prevSampleMillis, prevReportMillis;

void setup()
{

  // Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  // Wire.setClock(400000); // I2C frequency at 400 kHz
  // delay(100);  
  // Serial.begin(115200);
  // delay(100);

//HM10
  // initNanoBle(115200);
  
//  ble nano
  initNanoBle(115200);
  initMotor();
  InitAdfMotor(1, 2);
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

  SendMessages("READY!\n");

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
    SendMessages("Work of 3.3V...\n");
  #else
    SendMessages("Work of 5V...\n");
  #endif

  prevSampleMillis = millis();
  prevReportMillis = prevSampleMillis;
}

void loop()
{

  doSendRemainedPkg();
  
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

      executeControl(dt);      
  }

  if(  millisNow - prevReportMillis > 90 )
  { 
    prevReportMillis = millisNow;
    if( mROSConnected || mBleConnected )
    {
    
      SendRobotStateValue();

      // SendMessages("RP%d,%d,%d,%d,%d,0;",
      //   (int)(1000 * x),
      //   (int)(1000 * y),
      //   (int)(1000 * theta),
      //   (int)(1000 * w),
      //   (int)(1000 * v)
      // );
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
