
#include "MotorManager.h"

void MotorManager::begin(){
    // attaches actuators to hardware pins
    ESC.attach(ActuatorsConfig::ESC);
    vane_X1.attach(ActuatorsConfig::Servo_X1);
    vane_X2.attach(ActuatorsConfig::Servo_X2);
    vane_Y1.attach(ActuatorsConfig::Servo_Y1);
    vane_Y2.attach(ActuatorsConfig::Servo_Y2);

}




void MotorManager::actuate(const ActuatorCommand command){
    // converts ActuatorCommand into actual
    vane_X1.write(command.vaneX1_deg);
    vane_X2.write(command.vaneX2_deg);
    vane_Y1.write(command.vaneY1_deg);
    vane_Y2.write(command.vaneY2_deg);
}