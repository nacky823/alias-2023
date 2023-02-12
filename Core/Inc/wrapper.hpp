#ifndef WRAPPER_HPP_
#define WRAPPER_HPP_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

void Init();
void ExternalInterrupt(uint16_t);
void InterruptTim7();
void InterruptTim6();
void InterruptTim5();
void InterruptTim2();
void Loop();

void EmergencyStop();
float TargetVelocity(float target, float min);
float TargetDuty(float target, float min);

#ifdef DEBUG_MODE
uint8_t FlashTest();
void InitialTest();
uint8_t ImuReadByte(uint8_t, char);
void ImuWriteByte(uint8_t, uint8_t, char);
#endif // DEBUG_MODE

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // WRAPPER_HPP_