#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* Default build is local-only. Legacy host code is excluded by preprocessing. */
#define CHASSIS_STANDALONE 1
#define CHASSIS_BALL_ALIGN_ONLY_TEST 1 /* Center then grasp once in route mode. */
#define CHASSIS_BALL_STOP_ONLY_TEST 0 /* Corner 5: extend, search, stop; no alignment/grasp. */
#define CHASSIS_ROUTE_ONLY_TEST 0 /* Full map, no camera or arm missions. */
/* 1: one-shot arm test, chassis locked; 0: normal autonomous route. */
#define CHASSIS_ARM_ONLY_TEST 0
/* Camera-to-PC notification test: keep chassis stopped. */
#define CHASSIS_QR_NOTIFY_ONLY 0

/* Standalone R550 mecanum. For V550 hardware use Mec_Car_V550. */
#define CHASSIS_FIXED_CAR_MODE Mec_Car
#define CHASSIS_LOCAL_MAX_LINEAR_MPS 0.30f
#define CHASSIS_LOCAL_MAX_ANGULAR_RPS 0.80f
#define CHASSIS_LOCAL_TIMEOUT_MS 500U

/* Physical lateral direction verified on this car. Also used by odometry. */
#define CHASSIS_LOCAL_Y_SIGN (1.0f)

/* All four signed wheel speeds must track for 20 cycles at 100 Hz. */
#define CHASSIS_WHEEL_SPEED_TOLERANCE_MPS 0.01f
#define CHASSIS_WHEEL_SPEED_STABLE_CYCLES 20U

/* Wheel PI at 100 Hz. KI is per control sample, not per second.
 * Set USE_FLASH to 0 to tune using these source values consistently.
 * Keeping it at 1 preserves valid gains saved on the board.
 */
#define CHASSIS_WHEEL_PI_KP_DEFAULT 300.0f
#define CHASSIS_WHEEL_PI_KI_DEFAULT 300.0f
#define CHASSIS_WHEEL_PI_USE_FLASH 0

#endif
