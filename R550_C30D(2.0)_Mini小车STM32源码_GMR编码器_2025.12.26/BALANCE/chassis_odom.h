#ifndef CHASSIS_ODOM_H
#define CHASSIS_ODOM_H

/* Map frame: +x initially forward, +y initially left, yaw CCW positive.
 * x/y are metres, yaw is radians. For the race map the robot starts facing
 * the left side of the printed map, so +y appears down the page.
 * Route_task owns these APIs. It runs at 100 Hz below Balance_task priority.
 * Encoder speeds come from Balance_task; heading is held at startup.
 */
typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
} ChassisPose;

void Chassis_OdomImuPublish(int calibrated);
void Chassis_OdomReset(void);
void Chassis_OdomResumeAfterStop(void);
/* Startup only: set both estimators to the known physical starting location. */
int Chassis_OdomStartAt(float x_m,float y_m);
int Chassis_OdomUpdate(void); /* 1: updated, 0: invalid sample/timing */
ChassisPose Chassis_OdomGetPose(void);
/* Legacy signed wheel integration, without acceleration fusion or yaw rotation. */
ChassisPose Chassis_OdomGetWheelPose(void);
int Chassis_OdomSetPose(ChassisPose corrected); /* absolute pose correction */

/* 停车时按已识别地标校正一个位置轴，同时更新跑图使用的车轮坐标。 */
void Chassis_OdomCalibrateAxis(uint8_t axis, float position_m);
#endif
