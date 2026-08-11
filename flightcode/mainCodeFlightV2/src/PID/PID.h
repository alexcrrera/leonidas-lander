#pragma once
#ifndef PID_H

#define PID_H
#include "../Utilities/Utilities.h"


class PID {


    public:
        PID(PID_Gains gains)
            : kp(gains.kp), ki(gains.ki), kd(gains.kd) {}

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

        float outputMin=0.0;
        float outputMax=0.0;

        float integralMin=0.0;
        float integralMax=0.0;

        float desiredTarget;

        float previousError = 0.0;
        

    
};


struct PID_3D{ // group of 3 PID's
    PID axis_x;
    PID axis_y;
    PID axis_z;

};

#endif


