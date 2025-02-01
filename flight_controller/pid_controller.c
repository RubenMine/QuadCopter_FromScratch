#include "header/pid.h"

void pid_init(PID* pid, int kp, int ki, int kd, float UpperLimit, float LowerLimit, float setpoint, bool a_windup) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Filter_STATE = 0.0f;
    pid->Integrator_STATE = 0.0f;
    pid->UpperLimit = UpperLimit;
    pid->LowerLimit = LowerLimit;
    pid->setpoint = setpoint;
    pid->anti_windup = a_windup;
}


void pid_compute(PID* pid, float sensor_value){
  float FilterCoefficient;
  float Sum;
  float error = pid->setpoint-sensor_value;

  /* Gain: '<S36>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S28>/Filter'
   *  Gain: '<S27>/Derivative Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S28>/SumD'
   */
  FilterCoefficient = (pid->Kd * error - pid->Filter_STATE) *100.0;
  //printf("%f\n", FilterCoefficient);

  /* Sum: '<S43>/Sum' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S38>/Proportional Gain'
   *  Inport: '<Root>/u'
   */
  Sum = (pid->Kp * error + pid->Integrator_STATE) +
    FilterCoefficient;
  //printf("%f\n", Sum);

  /* Switch: '<S41>/Switch2' incorporates:
   *  Inport: '<Root>/LowerLimit'
   *  Inport: '<Root>/UpperLimit'
   *  RelationalOperator: '<S41>/LowerRelop1'
   *  RelationalOperator: '<S41>/UpperRelop'
   *  Switch: '<S41>/Switch'
   */
  if (Sum > pid->UpperLimit) {
    /* Switch: '<S41>/Switch2' */
    pid->output = pid->UpperLimit;
  } else if (Sum < pid->LowerLimit) {
    /* Switch: '<S41>/Switch2' incorporates:
     *  Inport: '<Root>/LowerLimit'
     *  Switch: '<S41>/Switch'
     */
    pid->output = pid->LowerLimit;
  } else {
    /* Switch: '<S41>/Switch2' incorporates:
     *  Switch: '<S41>/Switch'
     */
    pid->output = Sum;
  }
  //printf("%f\n", pid->output);

  /* End of Switch: '<S41>/Switch2' */

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S26>/Kb'
   *  Gain: '<S30>/Integral Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S26>/SumI2'
   *  Sum: '<S26>/SumI4'
   */
  float anti_windup;
  if(pid->anti_windup) anti_windup = (pid->output - Sum) * 2.0;
  else anti_windup = 0;

  pid->Integrator_STATE += (anti_windup + pid->Ki * error) * 0.01;
  
  /* Update for DiscreteIntegrator: '<S28>/Filter' */
  pid->Filter_STATE += 0.01 * FilterCoefficient;
}

