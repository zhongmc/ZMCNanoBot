#include <Arduino.h>

double d_kp, d_ki, d_kd; //direction controll pid
double lastError_d, lastErrorIntegration_d;

double v_kp, v_ki, v_kd; //velocity controll pid
double lastError_v, lastErrorIntegration_v;

double drive_v, drive_w; //driving velocity and turning w; 

double ctrl_v, ctrl_w; //控制输出v, mW
double vel_l, vel_r; //控制输出的vel 

double mTheta;  //目标方向
double mW;      //转弯
double curW;    //当前的转弯状态
bool keepTheta; //是否需要保存当前方向
int keepThetaTimer;
long thetaPrevMillis;
bool okToKeep;


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
    mTheta = 0;
    mW = 0;
    drive_v = 0;
    drive_w = 0;
    
    ctrl_v = 0;
    ctrl_w = 0;
    lastError_d = 0;
    lastError_v = 0;

    lastErrorIntegration_d = 0;
    lastErrorIntegration_v = 0;
}


//设定控制目标，速度、转弯
void driveRobot(double _v, double _w, double curTheta )
{

    if( abs(_v) <= 0.0001 && abs(_w) <= 0.001 )
    {
        drive_v = 0;
        drive_w = 0;
        StopMotor();

        lastError_d = 0;
        lastError_v = 0;

        lastErrorIntegration_d = 0;
        lastErrorIntegration_v = 0;

        return;

    }

  if (_w == 0 && curW != 0) //remain the current theta; 加速过程中会有晃动；保留初始角度？
  {
    keepTheta = true;
    keepThetaTimer = 120;//(1 + 2 * abs(m_robot_w)) * 60;
    thetaPrevMillis = millis();
    mTheta = curTheta; //转弯结束，保留当前角度
  }
  curW = _w;
  mW = _w;

    drive_v = _v;
    drive_w = _w;

  if( drive_w != 0  && drive_v != 0 ) //转弯，控制速度？？
  {
    drive_v = (0.1-abs(drive_v))/3.14 * abs(drive_w) + abs(drive_v);
    if( _v < 0 )
      drive_w = -drive_w;
  }
}

//执行控制，v 当前速度， theta 当前角度， w 当前转弯角速度
void executeControl( double _v, double _theta, double _w, double dt)
{
    if( drive_v == 0 && drive_w == 0 )
        return;

    executeDirControl(_v, _theta, _w, dt);
    executeVelocityControl(_v, _theta, _w, dt);

    double pwm_l, pwm_r;
    pwm_l = vel_to_pwm(vel_l);
    pwm_r = vel_to_pwm(vel_r);

    MoveLeftMotor(pwm_l);
    MoveRightMotor(pwm_r);

}


//执行控制，v 当前速度， theta 当前角度， w 当前转弯角速度
void executeDirControl( double _v, double _theta, double _w, double dt)
{

  if (mW != 0) //转弯，自由控制
  {
    ctrl_v = drive_v;
    ctrl_w = drive_w; 
    return;
  }

  if (keepTheta)
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
  double e = mTheta - _theta;
  e = atan2(sin(e), cos(e));
  double e_D = (e - lastError_d) / dt;
  double e_I = lastErrorIntegration_d + e * dt;
  drive_w = d_kp * e + d_ki * e_I + d_kd * e_D;
  lastErrorIntegration_d = e_I;
  lastError_d = e;    

}

void executeVelocityControl(double _v, double _theta, double _w, double dt)
{

    double _min_vel = getMinVel();
    Vel vel = uni_to_diff_v(ctrl_v, ctrl_w);


    vel_l = vel.vel_l;
    vel_r = vel.vel_r;


    if( abs(ctrl_v) <= 0.001 ) //原地转圈
    {
      if( abs(vel.vel_l ) < 1.2 * _min_vel )
      {
        if( vel.vel_l < 0 )
        {
          vel_l = -1.2 * _min_vel;
          vel_r = 1.2 * _min_vel;
        }
        else
        {
          vel_l = 1.2 * _min_vel;
          vel_r = -1.2 * _min_vel;
          
        }
        
      } 
      lastError_v = 0;
      lastErrorIntegration_v = 0;

      return;
    }


  if( ctrl_w > 0.2 ) //拐弯，不做速度控制
  {
      return;
  }
  else
  {
      double e, ei,ed;
      e = ctrl_v - _v;
      ei = lastErrorIntegration_v + e* dt;
      ed = (e-lastError_v)/dt;

      double sv = v_kp * e + v_ki * ei + v_kd*ed;
      lastErrorIntegration_v = ei;
      lastError_v = e;
      vel = uni_to_diff_v(sv, ctrl_w);
      vel_l = vel.vel_l;
      vel_r = vel.vel_r;   
  }
  

}