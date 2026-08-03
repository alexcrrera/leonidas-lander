#include "PID.h"
#include "Utilities.h"

PID::PID(float kp,
        float ki,
        float kd,
        float outputMin,
        float outputMax,
        float integralMin,
        float integralMax)
        :
        kp(kp) , ki(ki) , kd(kd)
        
        {

            setOutputLimits(outputMin,outputMax);
            setIntegralLimits(integralMin,integralMax);
            setGains(kp,ki,kd);
            initOutputLimits = true;

        }


void PID::setOutputLimits(float u_min,  float  u_max){
    outputMin = u_min;
    outputMax = u_max;
}


 void PID::setIntegralLimits(float i_min, float i_max){
    integralMin = i_min;
    integralMax = i_max;
}


void PID::setGains(float kp_, float ki_, float kd_){
    kp = kp_;
    ki = ki_;
    kd = kd_;
}


void PID::reset(){
    integral = 0.0;
}

void PID::setOffset(float offset_){
    offset = offset;
}


float  PID::compute(float dt, float measurement){

    float error = desiredTarget - measurement;

    integral = updateIntegral(dt,error);

    float error_derivative = (error - previousError)/dt;

    
    // ki is included in the integral term calculation

    previousError = error;


    return(kp * error + integral + kd*error_derivative);
}


float  PID::compute(float dt, float measurement, float measurement_derivative){
    // overwriting of the function to allow direct injection of the derivative of the error 
    float error = desiredTarget - measurement;

    integral = updateIntegral(dt,error);
    
    // ki is included in the integral term calculation

    previousError = error;

    return(kp * error + integral + kd*measurement_derivative);
}


void PID::setTarget(float target){
    // sets target 
    desiredTarget = target;
}

float PID::updateIntegral(float dt, float error){
    // simple update that bounds the output

    integral += ki*error*dt;

    integral = Utilities::constrain(integral, integralMin,integralMax); // limits the integral output

    return(integral);
}