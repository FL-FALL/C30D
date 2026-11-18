/* Legacy reference only. Current arm_task.c uses direct numeric arguments. */
#ifndef ARM_BUCKET_CONFIG_H
#define ARM_BUCKET_CONFIG_H
#include "arm_grasp_config.h"
/* Joint 3: absolute PWM position 0..270, center=135.
 * Joints 0/1/2: original signed offsets from calibrated zero. */
#define ARM_BUCKET_TEST_SLOT 1U /* Select ONE physical pickup location: 1,2,3. */
#define ARM_BUCKET_TEST_PICKUP_DEG 135U /* Heading facing the ball, 0..270. */
#define ARM_BUCKET_TEST_START_WAIT_MS 2000U
#define ARM_BUCKET_TEST_DROP_ENABLE 0 /* 0 hold over bucket; 1 lower and release. */
#if ARM_BUCKET_TEST_SLOT < 1 || ARM_BUCKET_TEST_SLOT > 3
#error Invalid_ARM_BUCKET_TEST_SLOT
#endif
#if ARM_BUCKET_TEST_PICKUP_DEG > 270
#error Invalid_ARM_BUCKET_TEST_PICKUP_DEG
#endif
#define ARM_PREP_JOINT0_DEG 90
#define ARM_PREP_JOINT1_DEG (-20)
#define ARM_PREP_JOINT2_DEG (-180)
#define ARM_PREP_WAIT_MS 1000U
/* Raise/withdraw before rotating; joint 2 remains at grasp angle. Tune these. */
#define ARM_LIFT_JOINT0_DEG 50
#define ARM_LIFT_WAIT_MS 1000U
/* SLOT 1: initial turn example, must be calibrated at physical ball position 1. */
#define ARM_SLOT1_LOWER_JOINT1_DEG 0
#define ARM_SLOT1_LOWER_WAIT_MS 1000U
#define ARM_SLOT1_TURN_JOINT3_DEG 75
#define ARM_SLOT1_TURN_WAIT_MS 2000U
#define ARM_SLOT1_RELEASE_JOINT2_DEG (-180)
#define ARM_SLOT1_RELEASE_WAIT_MS 1000U
/* SLOT 2 */
#define ARM_SLOT2_LOWER_JOINT1_DEG 0
#define ARM_SLOT2_LOWER_WAIT_MS 1000U
#define ARM_SLOT2_TURN_JOINT3_DEG 85
#define ARM_SLOT2_TURN_WAIT_MS 2000U
#define ARM_SLOT2_RELEASE_JOINT2_DEG (-180)
#define ARM_SLOT2_RELEASE_WAIT_MS 1000U
/* SLOT 3 */
#define ARM_SLOT3_LOWER_JOINT1_DEG 0
#define ARM_SLOT3_LOWER_WAIT_MS 1000U
#define ARM_SLOT3_TURN_JOINT3_DEG 150
#define ARM_SLOT3_TURN_WAIT_MS 2000U
#define ARM_SLOT3_RELEASE_JOINT2_DEG (-180)
#define ARM_SLOT3_RELEASE_WAIT_MS 1000U
#endif
