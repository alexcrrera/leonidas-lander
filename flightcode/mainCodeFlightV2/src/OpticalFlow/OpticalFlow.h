#pragma once
#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <Arduino.h>
#include <stdint.h>



#include "OpticalFlow_structs.h"

#include "OpticalFlow_utilities.h"
#include "../Config/SensorConfig.h"
#include "../Lander/Lander_structs.h"

class OpticalFlow
{
public:

    OpticalFlow();

    bool begin();
    void update(LanderState& landerState);
   
    OpticalFlowMeasurement& getMeasurement(){return opti_measurement;}




private:

   OpticalFlowMeasurement opti_measurement;
   OpticalFlow_data measurement ;

    

   static constexpr uint8_t packetHeader = 0xEF;

    static constexpr uint8_t expectedDeviceId = 0x0F;
    static constexpr uint8_t expectedSystemId = 0x00;
    static constexpr uint8_t expectedMessageId = 0x51;

    static constexpr uint8_t expectedPayloadLength = 0x14;

    static constexpr uint8_t headerLength = 6;
    static constexpr uint8_t payloadLength = 20;
    static constexpr uint8_t packetLength = 27;

    static constexpr float pixelScaling = 1.76e-3f;

    HardwareSerial* serialPort = nullptr;

    uint8_t packetBuffer[packetLength] = {};
    uint8_t packetIndex = 0;

    void projectOpticalFlowToNED(LanderState& landerState);


    bool readPacket(LanderState& landerState);
    bool decodePacket(const uint8_t* packet, LanderState& landerState);

    void resetParser();

    
    uint8_t calculateChecksum(const uint8_t* packet) const;

};

#endif // OPTICAL_FLOW_H