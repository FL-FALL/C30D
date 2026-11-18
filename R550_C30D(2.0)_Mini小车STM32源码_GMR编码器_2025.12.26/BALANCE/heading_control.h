#ifndef __HEADING_CONTROL_H
#define __HEADING_CONTROL_H
#define HEADING_CONTROL_ENABLE 1
/* Both installed drivers configure +/-500 deg/s. */
#define HEADING_CONTROL_GYRO_FULL_SCALE_DPS 500.0f
/* Positive body yaw is left rotation. Reverse only if sensor mounting requires it. */
#define HEADING_CONTROL_GYRO_SIGN (-1.0f)
#define HEADING_CONTROL_KP 7.5f
#define HEADING_CONTROL_KD 0.35f
#define HEADING_CONTROL_MAX_ANGULAR_VELOCITY 1.0f
#define HEADING_CONTROL_YAW_TOLERANCE 0.00005f
#define HEADING_CONTROL_GYRO_TOLERANCE 0.0001f
#define HEADING_CONTROL_SAMPLE_TIMEOUT_MS 50U
/* Called once per complete, bias-corrected IMU sample from the IMU task. */
void HeadingControl_Publish(short gyro_z,int calibrated);
int HeadingControl_Ready(void);
/* Capture current direction once on arming, not at corners/task pauses. */
void HeadingControl_Reset(void);
/* Called by the motor task. Integrates during stops but emits no motion then. */
float HeadingControl_Update(int translating);
/* Signed measured yaw minus departure heading, in degrees. */
float HeadingControl_GetYawOffsetDeg(void);
#endif
