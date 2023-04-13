#ifndef WRAPPER_HPP_
#define WRAPPER_HPP_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "stm32f4xx_hal.h"
#include "declare_extern.h"
#include "swo_setting.h"

void Init();
void InterruptTim7();
void InterruptTim6();
void InterruptTim2();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // WRAPPER_HPP_