#ifndef WRAPPER_HPP_
#define WRAPPER_HPP_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void Init();
void Interrupt100us();
void Interrupt1ms();

uint8_t ImuActive();
void EmergencyStop();

#ifdef DEBUG_MODE
void Monitor();
uint8_t ImuReadByte(uint8_t, char);
void ImuWriteByte(uint8_t, uint8_t, char);
#endif // DEBUG_MODE

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // WRAPPER_HPP_