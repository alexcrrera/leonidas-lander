#pragma once
#ifndef PID_H

#define PID_H

#include "PID.h"

struct PID_3D{ // group of 3 PID's
    PID axis_x;
    PID axis_y;
    PID axis_z;

}

class PID {


    public:
        PID(PID_gains gains);

        float compute(float dt, float measurement); // outputs PID correction term
        float compute(float dt, float measurement, float measurement_derivative); // outputs PID correction term

        void reset(); // resets the PID - integral term

        void setGains(float kp_, float ki_, float kd_); 
        void setGains(PID_Gains gains); 
        void setOutputLimits(float u_min, float  u_max);
        void setIntegralLimits(float i_min, float i_max);

        void setOffset(float offset_);

        void setTarget(float target); // set PID setpoint

        float updateIntegral(float dt, float error); // updates  integral term within bounds




    private:

        float offset = 0.0;

        float kp;
        float ki;
        float kd;
        float integral = 0.0;

        float outputMin;
        float outputMax;

        float integralMin;
        float integralMax;

        float desiredTarget;

        float previousError = 0.0;
        

    
};

#endif
