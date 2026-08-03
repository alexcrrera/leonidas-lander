#include "Utilities.h"
#include <cmath>


float Utilities::constrain(float u, float u_min, float u_max){
    if(u < u_min){
        return u_min;
    }

    if(u > u_max){
        return u_max;
    }

    return u;
}

float Utilities::clamping(float u, float u_max){
    return constrain(u,-u_max,u_max); // 1D constraining
}


float Utilities::EWA(float alpha, float u, float measurement){
    return alpha * measurement + (1-alpha)*u;
}


float Utilities::distance1D(float u, float u_ref){
    return(fabsf(u-u_ref));
}

float Utilities::distance2D(float x, float y, float x_ref, float y_ref){
    float dx = (x-x_ref);
    float dy = (y-y_ref);
    return(sqrtf(dx*dx + dy*dy));
}


float Utilities::distance3D(float x, float y, float z, float x_ref, float y_ref, float z_ref){ 
    float dx = (x-x_ref);
    float dy = (y-y_ref);
    float dz = (z-z_ref);
    return(sqrtf(dx*dx + dy*dy + dz*dz));
}

float Utilities::distance3D(const Vector3& p, const Vector3& p_ref){
    return(distance3D(p.x,p.y,p.z, p_ref.x, p_ref.y , p_ref.z));
}

float Utilities::verticalDistanceNED(const Vector3& p, const Vector3& t){
    return distance1D(p.z,t.z);
}

float Utilities::horizontalDistanceNED(const Vector3& p, const Vector3& t){
    return distance2D(p.x,p.y,t.x,t.y);
}


bool Utilities::isWithinEps_1D(float eps, float u, float u_ref){
    return(distance1D(u,u_ref)<=eps);
}
bool Utilities::isWithinEps_2D(float eps, float x, float y, float x_ref, float y_ref){
    return(distance2D(x,y,x_ref,y_ref)<=eps);
}
bool Utilities::isWithinEps_3D(float epsH, float epsV, float x, float y, float z, float x_ref, float y_ref, float z_ref){
    return(isWithinEps_1D(epsV,z,z_ref) && isWithinEps_2D(epsH,x,y,x_ref,y_ref));
}

bool Utilities::isWithinEps_3D(float epsH, float epsV,const Vector3& p, const Vector3& p_ref){
    return(isWithinEps_3D(epsH,epsV,p.x,p.y,p.z,p_ref.x,p_ref.y,p_ref.z));
}

bool Utilities::isWithinEps_NED(float epsH, float epsV, const Vector3& p, const Vector3& p_ref){
    return(isWithinEps_3D(epsH,epsV,p.x,p.y,p.z,p_ref.x,p_ref.y,p_ref.z));
}

bool Utilities::isWithinEps_Yaw(float epsYaw, float yaw, float yaw_ref){
    // is current yaw within the tolerance of the target value
    return(isWithinEps_1D(epsYaw,yaw,yaw_ref));
}

bool Utilities::isBounded(float u, float u_min, float u_max){
    // verify that u_min <= u <= u_max
    if(u_min>u_max){
        return(false); // potential bugs
    }
    return(u_min<=u&& u<=u_max );
}