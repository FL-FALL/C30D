/* Legacy reference only. Current arm_task.c uses direct numeric arguments. */
#ifndef ARM_GRASP_CONFIG_H
#define ARM_GRASP_CONFIG_H
/* Absolute signed target angles relative to each calibrated zero (degrees).
 * All three targets are issued together, once; edit for your actual linkage.
 * Initial values taken from existing APPROACH / PICK actions.
 */
#define ARM_GRASP_JOINT0_DEG 90
#define ARM_GRASP_JOINT1_DEG (-20)
#define ARM_GRASP_JOINT2_DEG (-70)
#define ARM_GRASP_SETTLE_MS 2000U
#endif
