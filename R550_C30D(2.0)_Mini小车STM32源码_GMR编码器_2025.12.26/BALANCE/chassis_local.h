#ifndef CHASSIS_LOCAL_H
#define CHASSIS_LOCAL_H

/* Task-context APIs only; not callable from interrupt handlers.
 * x: forward positive, y: left positive, wz: counterclockwise positive.
 * Units: m/s and rad/s. The selected mecanum model/wiring must be verified.
 * Refresh nonzero commands at least every 500 ms (recommended 10-20 ms).
 */
void Chassis_LocalInit(void);
int Chassis_Arm(void);
void Chassis_Disarm(void);
int Chassis_IsArmed(void);
int Chassis_SetVelocity(float vx, float vy, float wz);
int Chassis_Forward(float speed);
int Chassis_Backward(float speed);
int Chassis_Left(float speed);
int Chassis_Right(float speed);
int Chassis_Rotate(float angular_speed);
void Chassis_Stop(void);

/* Called only by the existing 100 Hz Balance_task. */
void Chassis_LocalUpdate(void);
int Chassis_MotionEnabled(void);
/* Edit this nonblocking hook to add a board-local controller/state machine. */
void Chassis_UserControl(void);

#endif
