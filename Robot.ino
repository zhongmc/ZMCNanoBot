#include <Arduino.h>

double x, y, theta;
double max_vel, min_vel;
double r, l;
double v, w; 

double m_per_tick_l, m_per_tick_r;
long prev_left_ticks, prev_right_ticks;
int ticks_per_rev_l, ticks_per_rev_r;

void initRobot()
{
    x = 0;
    y = 0;
    theta = 0;
    v = 0;
    w = 0;

    initRobot(0.0312,  0.1565, 990, 990, 19, 86);
}


void initRobot(double R, double L, double ticksr_l, double ticksr_r, double min_rpm, double max_rpm)
{
  x = 0;
  y = 0;
  theta = 0;
  v = 0;
  w = 0;
  prev_left_ticks = 0;
  prev_right_ticks = 0;
  r = R;
  l = L;
  ticks_per_rev_l = ticksr_l; //20;
  ticks_per_rev_r = ticksr_r; //20;
  m_per_tick_l = 2 * PI * r / ticks_per_rev_l;
  m_per_tick_r = 2 * PI * r / ticks_per_rev_r;
  max_vel = max_rpm * 2 * PI / 60;
  min_vel = min_rpm * 2 * PI / 60;
  prev_left_ticks = 0;
  prev_right_ticks = 0;
}

double getMinVel()
{
    return min_vel;
}

double getMaxVel()
{
    return max_vel;
}

void updateRobotState(long left_ticks, long right_ticks, double dt)
{
  //  long left_ticks, right_ticks;
  if (prev_right_ticks == right_ticks && prev_left_ticks == left_ticks)
  {
    w = 0;
    v = 0;
    return; //no change
  }

  double d_right, d_left, d_center, vel_l, vel_r;

  vel_l = ((double)(left_ticks - prev_left_ticks) / dt) / (double)ticks_per_rev_l;
  vel_r = ((double)(right_ticks - prev_right_ticks) / dt) / (double)ticks_per_rev_r;
  vel_l = 2 * PI * vel_l;
  vel_r = 2 * PI * vel_r;

  d_left = (left_ticks - prev_left_ticks) * m_per_tick_l;
  d_right = (right_ticks - prev_right_ticks) * m_per_tick_r;

  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  d_center = (d_right + d_left) / 2;
  v = d_center / dt;

  double phi = (d_right - d_left) / l;

  w = phi / dt;

  x = x + d_center * cos(theta);
  y = y + d_center * sin(theta);
  theta = theta + phi;
  theta = atan2(sin(theta), cos(theta));
}


  Vel uni_to_diff_v(double v, double w )
  {
    Vel vel;
    if( abs(v) <= 0.001 )
    {
      vel.vel_r = (2 * v + w * l) / (2 * r);
      vel.vel_l = (2 * v - w * l) / (2 * r);
      return vel;
    }

    vel.vel_l = v/r;
    vel.vel_r = v/r;

    double wv =  ( w * l) / (2 * r);
    if( w < 0 )
    {
      if( v < 0 )
      {
          vel.vel_l = vel.vel_l - wv;
          if( vel.vel_l > 0 )
            vel.vel_l = 0;
      }
      else
      {
          vel.vel_r = vel.vel_r + wv;
          if( vel.vel_r < 0 )
            vel.vel_r = 0;
      }      
    }
    else
    {
      if( v < 0 )
      {
          vel.vel_r = vel.vel_r + wv;
          if( vel.vel_r > 0 )
            vel.vel_r = 0;
      }
      else
      {
          vel.vel_l = vel.vel_l - wv;
          if( vel.vel_l < 0 )
            vel.vel_l = 0;
      }      
    }
    return vel;
  }

double vel_to_pwm(double vel)
{
  double nvel = abs(vel);
  if( nvel < min_vel )
    return 0;
  if( nvel > max_vel )
    nvel = max_vel;
  double pwm = 13.0 * nvel + 24.0;
  if (vel < 0)
    return -pwm;
  return pwm;
}
