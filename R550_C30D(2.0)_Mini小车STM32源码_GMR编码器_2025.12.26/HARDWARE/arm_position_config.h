#ifndef ARM_POSITION_CONFIG_H
#define ARM_POSITION_CONFIG_H
/* Position-servo calibration, NOT a measured joint position.
 * Zero is this pulse, never the physical position at power-up.
 * Positive sign is configurable per joint. 1000 us/180 deg is provisional.
 */
#define ARM_PC6_ZERO_US 1500U
#define ARM_PC7_ZERO_US 1500U
#define ARM_PC8_ZERO_US 1500U
#define ARM_PC9_ZERO_US 1500U
#define ARM_PC6_DIRECTION 1
#define ARM_PC7_DIRECTION 1
#define ARM_PC8_DIRECTION 1
#define ARM_PC9_DIRECTION 1
#define ARM_POSITION_SPAN_US 1000L
#define ARM_POSITION_SPAN_DEG 180L
#define ARM_POSITION_LIMIT_DEG 600
#endif
