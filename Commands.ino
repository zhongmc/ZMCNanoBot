#include <Arduino.h>


extern bool mROSConnected;
extern double d_kp, d_ki, d_kd, v_kp, v_ki, v_kd;
extern volatile long count1, count2;
static char comData[82];
int comDataCount = 0;

void initCommands()
{
    comDataCount = 0;
}

void checkCommands()
{

  //read speed setting from serial
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      char ch = Serial.read();
      comData[comDataCount++] = ch;
      if (ch == ';' || ch == '\r' || ch == '\n') //new command
      {
        processCommand(comData, comDataCount);
        comDataCount = 0;
      }
      if (comDataCount > 80) //some error
      {
        Serial.print("Err, CMD too long (>80)...");
        // comData[comDataCount] = 0;
        // Serial.println(comData);
        comDataCount = 0;
      }
    }
  }    

}


void processCommand(char *buffer, int bufferLen)
{
  *(buffer + bufferLen) = 0;
  if (bufferLen <= 2)
  {
    if (buffer[0] == 'b') //get baundrate
      Serial.println(115200);
    else if (buffer[0] == 'p')
      Serial.println(100);
    return;
  }

  char ch0, ch1;
  ch0 = tolower(buffer[0]);
  ch1 = tolower(buffer[1]);

  if (ch0 == 'g' && ch1 == 'r')
  {
    Serial.println("\r\n== OK ==");
  }
  else if (ch0 == 's' && ch1 == 't') //stop
  {
    // Serial.println("Stop!");
    stopRobot();
  }

  else if (ch0 == 'c' && ch1 == 'i') //count info
  {
        Serial.print("CI:");
        Serial.print(count1);
        Serial.write(',');
        Serial.println(count2);      
  }

  else if (ch0 == 'm' && ch1 == 'm') // move motor mmpwml,pwmr
  {
    int pwml = atoi(buffer + 2);
    char *buf = strchr((buffer + 2), ',');

    int pwmr = pwml;
    if (buf != NULL)
      pwmr = atoi(buf + 1);

    motorSpeed(pwml, pwmr);
    MoveMotor(0);
  }

  else if( ch0 == 'c' && ch1 == 'r') //ros connected
  {
    Serial.println("\nROS Connected OK!");
    mROSConnected = true;
  }

  else if (ch0 == 'p' && ch1 == 'i') //set pid cmd: pi type kp,ki,kd;
  {
    setPID(buffer + 2);
  }

  else if (ch0 == 's' && ch1 == 'd') //set drive Goal
  {
    double v, w = 0;

    v = atof(buffer + 2);
    char *buf = strchr(buffer, ',');
    if (buf != NULL)
      w = atof(buf + 1);

    setDriveGoal(v, w);
  }
  else
  {
    Serial.print("Na,");
    Serial.println(buffer );
  }
}




//cmd: pi type kp,ki,kd; type 2,3,4
void setPID(char *buffer)
{
  double p, i, d;
  int type = *buffer - '0';

  p = atof((buffer+1));
  char *buf = strchr(buffer, ',');
  i = atof((buf + 1));
  buf = strchr((buf + 1), ',');
  d = atof(buf + 1);
 
  Serial.print("Set PID:");
  Serial.print(type);
  Serial.print(',');
  Serial.print(p);
  Serial.print(',');
  Serial.print(i);
  Serial.print(',');
  Serial.println(d);

    if( type == 0 )
    {
        d_kp = p;
        d_ki = i;
        d_kd = d;
    }
    else if( type == 1)
    {
        v_kp = p;
        v_ki = i;
        v_kd = d;
    }

}


void motorSpeed(int pwml, int pwmr)
{
  long c1, c2, lt;
  MoveLeftMotor(pwml);
  MoveRightMotor(pwmr);

  Serial.print(pwml);
  Serial.print(',');
  Serial.print(pwmr);
  Serial.print(',');
  // Serial.print(pwm);
  // Serial.print(',');
  delay(500);
  c1 = count1;
  c2 = count2;
  lt = millis();
  delay(1000);
  lt = millis() - lt;

  c1 = count1 - c1;
  c2 = count2 - c2;
  // log("%d,%d,%d\n", lt, c1, c2);
  Serial.print(lt);
  Serial.write(',');
  Serial.print( c1 );
  Serial.write(',');
  Serial.println(c2);
}
