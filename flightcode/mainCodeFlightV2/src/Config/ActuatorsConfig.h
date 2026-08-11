#pragma once
#ifndef ACTUATORS_CONFIG_H
#define ACTUATORS_CONFIG_H
#include "../Utilities/Utilities.h"
//#include "Arduino.h"
/*
ACTUATORS PARAMETERS AND CONFIG

*/

struct ActuatorPWM_config{
   uint8_t pin;
   float frequency;
   uint16_t pulseMin;
   uint16_t pulseMax;

   float valueMin; // associated to pulseMin
   float valueMax; // associated to pulseMin

   float trim;

};


struct ActuatorCommand{
   float vaneX1_deg;
   float vaneX2_deg;
   float vaneY1_deg;
   float vaneY2_deg;
   float thrust_percentage;
};

 namespace ActuatorsConfig {

   // TVC  LIMITS
   constexpr float TVC_TOTAL_AUTHORITY_BUDGET_deg = 12.0; // absolute max value before stalling of the VANES
   constexpr float TVC_YAW_AUTHORITY_BUDGET_deg = 5.0;  // YAW (Z) authority budget


   static_assert(TVC_YAW_AUTHORITY_BUDGET_deg <=TVC_TOTAL_AUTHORITY_BUDGET_deg, "YAW AUTHORITY BUDGET IS TOO LARGE");  
   // Automatic
   constexpr float TVC_PITCH_ROLL_AUTHORITY_BUDGET_deg = TVC_TOTAL_AUTHORITY_BUDGET_deg - TVC_YAW_AUTHORITY_BUDGET_deg; // pitch/yaw (X/Y) authority budget


   // ESC LIMITS

   constexpr float ESC_thrust_min_percentage = 50.0; // in %
   constexpr float ESC_thrust_max_percentage = 90.0; // in %

   constexpr float ESC_thrust_initial_spooling_up_percentage = 65.0; // in %




   constexpr float ESC_thrust_testing_percentage = 21.0; // in %




   static_assert(ESC_thrust_min_percentage <ESC_thrust_max_percentage, "MAX THRUST SHOULD BE GREATER THAN MIN THRUST");  
   static_assert(ESC_thrust_max_percentage <=100, "MAX THRUST SHOULD BE GEQ THAN 100%");  
   static_assert(0 < ESC_thrust_min_percentage, "MIN THRUST SHOULD BE GREATER THAN 0%");  



   // Servo and ESC HARDWARE CONFIGS
   constexpr float Servo_frequency = 333.0;
   
   constexpr uint8_t ESC_pin = 5;

   constexpr uint8_t Servo_X1_pin = 6;
   constexpr uint8_t Servo_X2_pin = 7;
   constexpr uint8_t Servo_Y1_pin = 8;
   constexpr uint8_t Servo_Y2_pin = 9;

   // Trim for servos

   constexpr float Servo_X1_trim = 0.0;
   constexpr float Servo_X2_trim = 0.0;
   constexpr float Servo_Y1_trim = 0.0;
   constexpr float Servo_Y2_trim = 0.0;



   static_assert(Servo_frequency >0, "Servo Frequency must be greater than 0");  



   constexpr ActuatorPWM_config ESC = {
                  .pin = ESC_pin,
                  .frequency = 50.0,
                  .pulseMin = 1000,
                  .pulseMax = 2000,
                  .valueMin = 0.0,
                  .valueMax = 100.0,
                  .trim = 0.0
    };




    constexpr ActuatorPWM_config Servo_X1 = {
                  .pin = Servo_X1_pin,
                  .frequency = Servo_frequency,
                  .pulseMin = 2000,
                  .pulseMax = 2800,
                  .valueMin = -90,
                  .valueMax = 90,
                  .trim = Servo_X1_trim
    };

        constexpr ActuatorPWM_config Servo_X2 = {
                  .pin = Servo_X2_pin,
                  .frequency = Servo_X1.frequency,
                  .pulseMin = Servo_X1.pulseMin,
                  .pulseMax = Servo_X1.pulseMax,
                  .valueMin = Servo_X1.valueMin,
                  .valueMax = Servo_X1.valueMax,
                  .trim = Servo_X2_trim
    };

    
        constexpr ActuatorPWM_config Servo_Y1 = {
                  .pin = Servo_Y1_pin,
                  .frequency = Servo_X1.frequency,
                  .pulseMin = Servo_X1.pulseMin,
                  .pulseMax = Servo_X1.pulseMax,
                  .valueMin = Servo_X1.valueMin,
                  .valueMax = Servo_X1.valueMax,
                  .trim = Servo_Y1_trim
    };

    
        constexpr ActuatorPWM_config Servo_Y2 = {
                  .pin = Servo_Y2_pin,
                  .frequency = Servo_X1.frequency,
                  .pulseMin = Servo_X1.pulseMin,
                  .pulseMax = Servo_X1.pulseMax,
                  .valueMin = Servo_X1.valueMin,
                  .valueMax = Servo_X1.valueMax,
                  .trim = Servo_Y2_trim
    };
    




 }

 #endif


