/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min) {
   // TODO: Initialize PID coefficients (and errors, if needed)
   this->Kp = Kp;
   this->Ki = Ki;
   this->Kd = Kd;
   this->output_lim_max = output_lim_max;
   this->output_lim_min = output_lim_min;
   this->delta_t = 0.0;
   this->error_p = 0.0;
   this->error_i = 0.0;
   this->error_d = 0.0;
}


void PID::UpdateError(double cte) {
   // TODO: Update PID errors based on cte.
   // Update the proportional-gain error
   double old_error_p = this->error_p;
   this->error_p = cte;
   // Update the derivative-gain error term
   this->error_d = 0.0; 
   if (this->delta_t > 0.0) {
      this->error_d = (cte - old_error_p) / this->delta_t;
   }  
   // Update the integral-gain error term
   this->error_i += cte * this->delta_t;
}

double PID::TotalError() {
   // TODO: Calculate and return the total error
   // The code should return a value in the interval [output_lim_min, output_lim_max]
   double control;
   control = (this->Kp * this->error_p) + (this->Kd * this->error_d) + (this->Ki * this->error_i);
   if (control > this->output_lim_max){
   control = this->output_lim_max;
   } else if (control < this->output_lim_min){
   control = this->output_lim_min;
   } else {
   }
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   // TODO: Update the delta time with new value
   this->delta_t = new_delta_time;
   return this->delta_t;
}