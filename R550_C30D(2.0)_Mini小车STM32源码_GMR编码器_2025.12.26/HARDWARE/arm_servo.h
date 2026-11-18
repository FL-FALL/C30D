#ifndef ARM_SERVO_H
#define ARM_SERVO_H

#include "stm32f4xx.h"

/* Index 0..3 maps to PC6..PC9 / TIM8 CH1..CH4. */
void ArmServo_Init(void);
void ArmServo_SetPulseUs(uint8_t index, uint16_t pulse_us);
/* Signed target offset from calibrated zero; returns 1 when accepted. */
int Arm_SetAngle(uint8_t index, int16_t angle_deg);
void ArmServo_Disable(uint8_t index);
void ArmServo_DisableAll(void);
/* Joint 3 absolute nominal 0..270 degrees: PC9, 500..2500 us, 50 Hz.
 * Legacy Arm_SetAngle retains its signed offset mapping for existing poses.
 */
int Arm3_SetPwmPositionDeg(uint16_t degrees);
#endif
