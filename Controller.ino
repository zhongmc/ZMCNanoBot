#include <Arduino.h>

double d_kp, d_ki, d_kd; //direction controll pid
double lastError_d, lastErrorIntegration_d;

double v_kp, v_ki, v_kd; //velocity controll pid
double lastError_v, lastErrorIntegration_v;

double drive_v, drive_w; //driving velocity and turning w; 

double turn_v, turn_w; //转圈控制量
double prevTheta, turnedTheta;
int turnAngle;
long sCount1, sCount2;

double ctrl_v, ctrl_w; //控制输出v, w
double vel_l, vel_r; //控制输出的vel 

double mTheta;  //目标方向
double curW;    //当前的转弯状态
bool keepTheta; //是否需要保存当前方向
int keepThetaTimer;
long thetaPrevMillis;
bool okToKeep;

double targetX, targetY, targetTheta, gtgV;

bool simulateMode = false;
double left_ticks = 0, right_ticks = 0;


#define IDLE 0   //空闲
#define DRV 1    //手动驾驶
#define GTG 2    //目标自动驾驶
#define GTG_T 3  //目标点方向调整
#define TURN 4   //目标转圈

byte mState = IDLE;



void initController()
{
    initController(5.0, 0, 0, 0.3, 0.8, 0);
}


void initController(double _dkp, double _dki, double _dkd, double _vkp, double _vki, double _vkd )
{
    d_kp = _dkp;
    d_ki = _dki;
    d_kd = _dkd;
    v_kp = _vkp;
    v_ki = _vki;
    v_kd = _vkd;

  resetController();
}


void resetController()
{
    stopRobot();

    mTheta = 0;
    curW = 0;
    drive_v = 0;
    drive_w = 0;
    
    ctrl_v = 0;
    ctrl_w = 0;
    lastError_d = 0;
    lastError_v = 0;

    lastErrorIntegration_d = 0;
    lastErrorIntegration_v = 0;

    mState = IDLE;

}

void turnAround(int dir, int angle )
{
  if( mState != IDLE )
  {
    lastError_d = 0;
    lastErrorIntegration_d = 0;
  }

  turnAngle = angle;
  turnedTheta = 0;

  sCount1 = readLeftEncoder();
  sCount2 = readRightEncoder();
  if( angle < 90 )
    return;

  mState = TURN;
  double _w = 0.8;
  double _v = 0.04;

  switch( dir )
  {
    case 0:
      turn_v = 0;
      turn_w = _w;
      break;
    case 1:
      turn_v = _v;
      turn_w = _w;
      break;
    case 2:
      turn_v = _v;
      turn_w = -_w;
      break;
    case 3:
      turn_v = 0;
      turn_w = -_w;
      break;
    case 4:
      turn_v = -_v;
      turn_w = _w;
      break;
    case 5:
      turn_v = -_v;
      turn_w = -_w;
      break;
  }
}

void executeTurnAround(double dt )
{
    double delta_theta = theta - prevTheta;
    delta_theta = atan2(sin(delta_theta), cos(delta_theta));
    turnedTheta = turnedTheta + abs(delta_theta);
    int turnedAngle = (int)(180.0*turnedTheta / PI );
    if( abs(turnedAngle - turnAngle) < 5  || turnedAngle >= turnAngle )
    {

      prevTheta = theta;
      stopRobot();
      mState = IDLE;

      if( !simulateMode )
      {      
        delay(100);
        updateRobotState(readLeftEncoder(), readRightEncoder(), dt);

        delta_theta = theta - prevTheta;
        delta_theta = atan2(sin(delta_theta), cos(delta_theta));
        turnedTheta = turnedTheta + abs(delta_theta);
        int turnedAngle = (int)(180.0*turnedTheta / PI );

        int c1,c2;
        c1 = readLeftEncoder() - sCount1;
        c2 = readRightEncoder() - sCount2;

        SendMessages("-t1:%d,%d,%d\n", turnedAngle, c1, c2);

      }
      else
        SendMessages("-t1:%d\n", turnedAngle);
    }

    ctrl_v = turn_v;
    ctrl_w = turn_w;
}


//设定控制目标，速度、转弯
void setDriveGoal(double _v, double _w)
{

  if( mState != IDLE && mState != DRV )
  {
    lastError_d = 0;
    lastErrorIntegration_d = 0;
  }

  mState = DRV;

  if( abs(_v) <= 0.0001)
    _v = 0;
  
  if( abs(_w) <= 0.001 )
    _w = 0;

  drive_v = _v;
  drive_w = _w;

  if( _v == 0 && _w == 0 )
  {
    mState = IDLE;
    if( curW != 0 )
      mTheta = theta;
    curW = 0;
    stopRobot();
    mTheta = theta;
    lastError_d = 0;
    lastError_v = 0;
    ctrl_v = 0;
    ctrl_w = 0;

    lastErrorIntegration_d = 0;
    lastErrorIntegration_v = 0;
    return;
  }

  if (_w == 0 && curW != 0) //remain the current theta; 加速过程中会有晃动；保留初始角度？
  {
    keepTheta = true;
    okToKeep = false;
    keepThetaTimer = 120;//(1 + 2 * abs(m_robot_w)) * 60;
    thetaPrevMillis = millis();
  }

  curW = _w;

  if( drive_w != 0  && drive_v != 0 ) //转弯，控制速度？？
  {
    drive_v = (0.09-abs(drive_v))/3.14 * abs(drive_w) + abs(drive_v);
    if( _v < 0 )
      drive_v = -drive_v;
  }
}

void setGTGGoal(double _x, double _y, double _theta , double _v )
{
  targetX = _x;
  targetY = _y;
  targetTheta = _theta;
  gtgV = _v;

}

void startGTG()
{
    lastError_d = 0;
    lastErrorIntegration_d = 0;
    mState = GTG; //start GTG
}



void executeGTG( double dt )
{

 double u_x, u_y, e, e_I, e_D, theta_g;
  u_x = targetX - x;
  u_y = targetY - y;

  theta_g = atan2(u_y, u_x);
  double d = sqrt(pow(u_x, 2) + pow(u_y, 2));

  if( mState == GTG_T ) // turn to garget angle
  {
    ctrl_v = 0;
    e = targetTheta - theta;
    e = atan2(sin(e), cos(e));

    if( abs(e) < 0.09 )
    {
      stopRobot();
      ctrl_w = 0;

      lastErrorIntegration_d = 0;
      lastError_d = 0;
      Serial.println("At Goal!");
      mState = IDLE;
      return;
    }

    ctrl_w = e/2;
    return;
  }

  ctrl_v = gtgV;
  e = theta_g - theta;
  e = atan2(sin(e), cos(e));
  e_D = (e - lastError_d) / dt;
  e_I = lastErrorIntegration_d + e * dt;
  ctrl_w = d_kp * e + d_ki * e_I + d_kd * e_D;
  lastErrorIntegration_d = e_I;
  lastError_d = e;

  if( d < 0.3 && d > 0.03)
  {
    ctrl_v = d *(gtgV - 0.05)/0.3 + 0.05;
  }

  if( d > 0.02 )
    return;  

  stopRobot();
  
  mState = GTG_T;

  ctrl_v = 0;
  ctrl_w = 0;

  lastError_d = 0;
  lastErrorIntegration_d = 0;

}



//执行控制，v 当前速度， theta 当前角度， w 当前转弯角速度
void executeDirControl( double dt)
{

  if (drive_w != 0) //转弯，自由控制
  {
    ctrl_v = drive_v;
    ctrl_w = drive_w; 
    return;
  }

  if (keepTheta == true )
  {
    if (millis() - thetaPrevMillis > keepThetaTimer) //
    {
      keepTheta = false; //next circle to keep the theta??
      okToKeep = true;
    }
    else
    {
      ctrl_v = drive_v;
      ctrl_w = 0;
  //reset the controller ???
      lastErrorIntegration_d = 0;
      lastError_d = 0;
      return;
    }
  }

  if (okToKeep)
  {
    okToKeep = false;
    mTheta = theta; // keep current direction
  }

  ctrl_v = drive_v;
  double e = mTheta - theta;
  e = atan2(sin(e), cos(e));
  double e_D = (e - lastError_d) / dt;
  double e_I = lastErrorIntegration_d + e * dt;
  ctrl_w = d_kp * e + d_ki * e_I + d_kd * e_D;
  lastErrorIntegration_d = e_I;
  lastError_d = e;    

}

void executeVelocityControl( double dt)
{

    Vel vel = uni_to_diff_oneside(ctrl_v, ctrl_w); //uni_to_diff_v(ctrl_v, ctrl_w);


    vel_l = vel.vel_l;
    vel_r = vel.vel_r;


    if( abs(ctrl_v) <= 0.001 ) //原地转圈
    {
      lastError_v = 0;
      lastErrorIntegration_v = 0;
      return;
    }


  if( drive_w != 0 ) //ctrl_w > 0.9 ) //拐弯，不做速度控制
      return;
  if( abs(ctrl_w) > 0.2) 
    return;

  double e, ei,ed;
  e = ctrl_v - v;
  ei = lastErrorIntegration_v + e* dt;
  ed = (e-lastError_v)/dt;

  double sv = v_kp * e + v_ki * ei + v_kd*ed;
  lastErrorIntegration_v = ei;
  lastError_v = e;
  vel = uni_to_diff_oneside(sv, ctrl_w); //uni_to_diff_v(sv, ctrl_w);
  vel_l = vel.vel_l;
  vel_r = vel.vel_r;   
}



void executeControl( double dt)
{
    //save the theta
    prevTheta = theta;
    if( simulateMode )
      updateRobotState(left_ticks, right_ticks, dt);
    else
      updateRobotState(readLeftEncoder(), readRightEncoder(), dt);


    if( mState == GTG || mState == GTG_T )
    {
        executeGTG( dt );
        if( mState == IDLE ) //at Goal
        {
           mTheta = theta;
          return;
        }
    }
    else if( mState == DRV )
    {

      if( drive_v == 0 && drive_w == 0 )
      {
        mTheta = theta;
        return;
      }

      executeDirControl( dt );
    }
    else if( mState == TURN )
    {
      executeTurnAround( dt );
      if( mState == IDLE )
      {
        mTheta = theta;
        return;
      }
    }
    else 
    {
      return;
    }

    if( mState == GTG || mState == DRV )
      executeVelocityControl( dt );
    else
    {
      Vel vel = uni_to_diff_v(ctrl_v, ctrl_w);
      vel_l = vel.vel_l;
      vel_r = vel.vel_r;
    }
    
    double pwm_l, pwm_r;
    pwm_l = vel_to_pwm(vel_l);
    pwm_r = vel_to_pwm(vel_r);

    if( simulateMode )
    {
      left_ticks = left_ticks + pwm_to_ticks_l(pwm_l, dt);
      right_ticks = right_ticks + pwm_to_ticks_r(pwm_r, dt);
    }
    else
    {
      // MoveLeftMotor(pwm_l);
      // MoveRightMotor(pwm_r);
      MoveAdfMotor(pwm_l, pwm_r);
    }

}



void stopRobot()
{
  mState = IDLE;
  drive_v = 0;
  drive_w = 0;
  ctrl_v = 0;
  ctrl_w = 0;

  if( simulateMode )
    return;

  // StopMotor();
  StopAdfMotor();
  delay(100);
  updateRobotState(readLeftEncoder(), readRightEncoder(), 0.03); //处理当前运动的惯性
}
