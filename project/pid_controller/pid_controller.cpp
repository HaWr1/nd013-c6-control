/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this->Kp = Kpi;
  this->Ki = Kii;
  this->Kd = Kdi;

  this->output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_mini;

  this->cte = 0;
  this->int_cte = 0;
  this->div_cte = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   **/
  // avoid div by 0
  if (std::abs(this->dt) > __DBL_EPSILON__) {
    this->div_cte = (cte - this->prev_cte) / this->dt;
  } else {
    this->div_cte = 0;
  }
  this->prev_cte = cte;
  this->int_cte += cte * this->dt;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */
  double control;

  // calculate control law
  control = -(this->Kp * this->cte + this->Ki * this->int_cte +
              this->Kd * this->div_cte);

  // limit the controller output
  control = std::min(control, this->output_lim_max);
  control = std::max(control, this->output_lim_min);

  return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * TODO: Update the delta time with new value
   */
  this->dt = new_delta_time;
}