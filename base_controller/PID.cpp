
#include "PID.h"

//////////////////////////////////Constructor//////////////////////////////////////
PID::PID() {
  time_ = millis();
}

PID::PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time) {
  time_ = millis();
  setTunings(kp, ki, kd);
  setOutputLimits(out_min, out_max);
  setMaxErrorSum(max_error_sum);
  setSampleTime(sample_time);
}

//////////////////////////////////Compute//////////////////////////////////////

double PID::compute_dt(const double setpoint, const double input, const double sample_time_) {

  const double error = setpoint - input;
  
  double output = output1 + (kp_ + kd_ / sample_time_) * error +
  ((kp_) * (-1) + ki_ * sample_time_ - 2 * kd_ / sample_time_) * error1 + (kd_ / sample_time_) * error2;

  output1 = output;
  error2 = error1;
  error1 = error;
  
  if(output > 380.0)
    output = 380.0;
  if(output < 60.0)
    output = 60.0;
  

  //output = max(out_min_, min(out_max_, output));
  
  time_ = millis();
    
  return output;

}

//////////////////////////////////Set Methods//////////////////////////////////////
void PID::setTunings(const double kp,  const double ki , const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}     	 

void PID::setSampleTime(const unsigned long sample_time) {
  sample_time_ = sample_time;
}				

void PID::setMaxErrorSum(const double max_error_sum) {
  max_error_sum_ = max_error_sum;
}

void PID::setOutputLimits(const double out_min, const double out_max) {
  out_min_ = out_min;
  out_max_ = out_max;
}

void PID::reset() {
  error_sum_ = 0;
  error_pre_ = 0;
  output1 = 0;
}

//////////////////////////////////Get Methods//////////////////////////////////////
double PID::getKp() {
  return kp_;
}

double PID::getKi() {
  return ki_;
}

double PID::getKd() {
  return kd_;
}

double PID::getPre() {
  return error_pre_;
}
