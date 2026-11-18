#include "system.h"
#include "chassis_odom.h"
#include "heading_control.h"
#include <math.h>
#include <stdlib.h>

/* Tuning: larger TAU gives acceleration prediction more weight.
 * Encoders remain the long-term velocity reference; this is NOT absolute positioning.
 */
#define ODOM_FUSION_TAU_S 0.15f
#define ODOM_ACCEL_FILTER_TAU_S 0.03f
#define ODOM_ACCEL_LIMIT_MPS2 3.0f
#define ODOM_IMU_MAX_AGE_MS 30U
#define CHASSIS_ODOM_X_SCALE 1.0f
#define CHASSIS_ODOM_Y_SCALE 1.0f
#define ACCEL_TO_MPS2 (9.80665f/16384.0f)
#define GYRO_TO_RPS (500.0f/32768.0f*0.01745329252f)

static ChassisPose odom_pose;
/* Legacy wheel-only coordinates: independent of acceleration and yaw fusion. */
static ChassisPose wheel_pose;
static TickType_t odom_last_tick;
static float velocity_x,velocity_y,yaw_offset;
static unsigned int stationary_ms;

/* IMU task owns gravity propagation. Route task reads the published snapshot. */
static float gravity[3],bias[3],filtered_ax,filtered_ay;
static int imu_initialized;
static TickType_t imu_last_tick;
static struct {float ax,ay; TickType_t tick; int valid;} sample;
volatile float Odom_WheelResidualX,Odom_WheelResidualY;
volatile int Odom_AccelFusionActive;

static int WheelsStill(void)
{
    return fabsf(MOTOR_A.Encoder)<0.005f && fabsf(MOTOR_B.Encoder)<0.005f &&
           fabsf(MOTOR_C.Encoder)<0.005f && fabsf(MOTOR_D.Encoder)<0.005f &&
           Move_X==0.0f && Move_Y==0.0f && Move_Z==0.0f;
}

/* Run after both sensor reads and startup calibration, in the IMU task.
 * Use Original_accel: the legacy accel.z has already had artificial 1g added.
 * Sensor axes confirmed by user: X forward, Y right; body Y is therefore -sensor Y.
 */
void Chassis_OdomImuPublish(int calibrated)
{
    TickType_t now=xTaskGetTickCount();
    float a[3],w[3],g[3],norm,dt,alpha,ax,ay;
    unsigned int i;
    int still,usable;
    if(!calibrated) {
        imu_initialized=0;
        taskENTER_CRITICAL(); sample.valid=0; taskEXIT_CRITICAL();
        return;
    }
    a[0]=imu.Original_accel.x*ACCEL_TO_MPS2;
    a[1]=imu.Original_accel.y*ACCEL_TO_MPS2;
    a[2]=imu.Original_accel.z*ACCEL_TO_MPS2;
    w[0]=imu.gyro.x*GYRO_TO_RPS;
    w[1]=imu.gyro.y*GYRO_TO_RPS;
    w[2]=imu.gyro.z*GYRO_TO_RPS;
    if(!imu_initialized) {
        g[0]=imu.Deviation_accel.x*ACCEL_TO_MPS2;
        g[1]=imu.Deviation_accel.y*ACCEL_TO_MPS2;
        g[2]=imu.Deviation_accel.z*ACCEL_TO_MPS2;
        norm=sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
        if(!(norm>7.0f && norm<12.0f)) return;
        for(i=0;i<3;i++) { gravity[i]=g[i]*9.80665f/norm; bias[i]=g[i]-gravity[i]; }
        imu_last_tick=now; imu_initialized=1;
        filtered_ax=filtered_ay=0.0f;
    }
    dt=(float)(TickType_t)(now-imu_last_tick)*portTICK_PERIOD_MS*0.001f;
    imu_last_tick=now;
    if(dt>0.05f) {
        taskENTER_CRITICAL(); sample.valid=0; taskEXIT_CRITICAL();
        imu_initialized=0;
        return;
    }
    /* Gravity rotates opposite to sensor angular velocity in sensor coordinates. */
    g[0]=gravity[0]+(w[2]*gravity[1]-w[1]*gravity[2])*dt;
    g[1]=gravity[1]+(w[0]*gravity[2]-w[2]*gravity[0])*dt;
    g[2]=gravity[2]+(w[1]*gravity[0]-w[0]*gravity[1])*dt;
    taskENTER_CRITICAL(); still=WheelsStill(); taskEXIT_CRITICAL();
    norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    /* Slowly correct tilt only when apparently stationary, not while accelerating. */
    if(still && fabsf(norm-9.80665f)<0.30f &&
       fabsf(w[0])+fabsf(w[1])+fabsf(w[2])<0.03f) {
        alpha=dt/(1.0f+dt);
        for(i=0;i<3;i++) g[i]+=alpha*(a[i]-bias[i]-g[i]);
    }
    norm=sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
    for(i=0;i<3;i++) gravity[i]=g[i]*9.80665f/norm;
    ax=a[0]-bias[0]-gravity[0];
    ay=-(a[1]-bias[1]-gravity[1]);
    usable=fabsf(ax)<=ODOM_ACCEL_LIMIT_MPS2 && fabsf(ay)<=ODOM_ACCEL_LIMIT_MPS2 &&
           fabsf(gravity[2])>8.5f &&
           abs(imu.Original_accel.x)<32000 && abs(imu.Original_accel.y)<32000 &&
           abs(imu.Original_accel.z)<32000;
    alpha=dt/(ODOM_ACCEL_FILTER_TAU_S+dt);
    if(usable) {
        filtered_ax+=alpha*(ax-filtered_ax);
        filtered_ay+=alpha*(ay-filtered_ay);
    } else filtered_ax=filtered_ay=0.0f;
    taskENTER_CRITICAL();
    sample.ax=filtered_ax; sample.ay=filtered_ay; sample.tick=now; sample.valid=usable;
    taskEXIT_CRITICAL();
}

void Chassis_OdomReset(void)
{
    taskENTER_CRITICAL();
    odom_pose.x_m=odom_pose.y_m=odom_pose.yaw_rad=0.0f;
    wheel_pose.x_m=wheel_pose.y_m=wheel_pose.yaw_rad=0.0f;
    odom_last_tick=xTaskGetTickCount();
    yaw_offset=-HeadingControl_GetYawOffsetDeg()*0.01745329252f;
    velocity_x=velocity_y=0.0f; stationary_ms=0;
    taskEXIT_CRITICAL();
}

int Chassis_OdomStartAt(float x_m,float y_m)
{
    if(!(x_m>=-20.0f && x_m<=20.0f && y_m>=-20.0f && y_m<=20.0f)) return 0;
    Chassis_OdomReset();
    taskENTER_CRITICAL();
    odom_pose.x_m=wheel_pose.x_m=x_m;
    odom_pose.y_m=wheel_pose.y_m=y_m;
    taskEXIT_CRITICAL();
    return 1;
}

int Chassis_OdomUpdate(void)
{
    TickType_t now,ticks;
    float dt,a,b,c,d,wx,wy,ax,ay,theta,cs,sn,world_wx,world_wy,px,py,blend;
    int use_imu,still;
    taskENTER_CRITICAL();
    now=xTaskGetTickCount();
    a=MOTOR_A.Encoder; b=MOTOR_B.Encoder; c=MOTOR_C.Encoder; d=MOTOR_D.Encoder;
    ax=sample.ax; ay=sample.ay;
    use_imu=sample.valid && (TickType_t)(now-sample.tick)<=pdMS_TO_TICKS(ODOM_IMU_MAX_AGE_MS);
    still=WheelsStill();
    theta=HeadingControl_GetYawOffsetDeg()*0.01745329252f+yaw_offset;
    taskEXIT_CRITICAL();
    ticks=(TickType_t)(now-odom_last_tick);
    if(ticks==0) return 1;
    dt=(float)ticks*portTICK_PERIOD_MS*0.001f;
    if(!(dt>0.0f && dt<=0.05f)) return 0;
    wx=(a+b+c+d)*0.25f*CHASSIS_ODOM_X_SCALE;
    wy=(a-b+c-d)*0.25f*CHASSIS_ODOM_Y_SCALE*CHASSIS_LOCAL_Y_SIGN;
    if(!(wx>=-1.0f && wx<=1.0f && wy>=-1.0f && wy<=1.0f &&
         theta>=-100.0f && theta<=100.0f)) return 0;
    cs=cosf(theta); sn=sinf(theta);
    world_wx=cs*wx-sn*wy; world_wy=sn*wx+cs*wy;
    px=velocity_x+(cs*ax-sn*ay)*dt;
    py=velocity_y+(sn*ax+cs*ay)*dt;
    Odom_WheelResidualX=world_wx-px; Odom_WheelResidualY=world_wy-py;
    /* Complementary velocity fusion. Residual is an inconsistency indicator,
     * not proof of slip: encoder scale, IMU bias and timing also affect it. */
    blend=use_imu ? dt/(ODOM_FUSION_TAU_S+dt) : 1.0f;
    velocity_x=use_imu ? px+blend*(world_wx-px) : world_wx;
    velocity_y=use_imu ? py+blend*(world_wy-py) : world_wy;
    if(still && (!use_imu || (fabsf(ax)<0.12f && fabsf(ay)<0.12f))) {
        stationary_ms+=(unsigned int)(ticks*portTICK_PERIOD_MS);
        if(stationary_ms>=200U) { stationary_ms=200U; velocity_x=velocity_y=0; }
    } else stationary_ms=0;
    Odom_AccelFusionActive=use_imu;
    odom_last_tick=now;
    taskENTER_CRITICAL();
    wheel_pose.x_m+=wx*dt; wheel_pose.y_m+=wy*dt;
    odom_pose.x_m+=velocity_x*dt; odom_pose.y_m+=velocity_y*dt;
    odom_pose.yaw_rad=theta;
    taskEXIT_CRITICAL();
    return 1;
}

ChassisPose Chassis_OdomGetPose(void)
{
    ChassisPose result;
    taskENTER_CRITICAL(); result=odom_pose; taskEXIT_CRITICAL();
    return result;
}

int Chassis_OdomSetPose(ChassisPose corrected)
{
    if(!(corrected.x_m>=-20 && corrected.x_m<=20 && corrected.y_m>=-20 && corrected.y_m<=20 &&
         corrected.yaw_rad>=-100 && corrected.yaw_rad<=100)) return 0;
    taskENTER_CRITICAL();
    odom_pose=corrected;
    yaw_offset=corrected.yaw_rad-HeadingControl_GetYawOffsetDeg()*0.01745329252f;
    taskEXIT_CRITICAL();
    return 1;
}

/* Absolute fused-pose corrections must not change wheel travel. */
ChassisPose Chassis_OdomGetWheelPose(void)
{
    ChassisPose result;
    taskENTER_CRITICAL(); result=wheel_pose; taskEXIT_CRITICAL();
    return result;
}

/* Resume after the existing parked arm sequence; retain both positions and yaw. */
void Chassis_OdomResumeAfterStop(void)
{
    taskENTER_CRITICAL();
    odom_last_tick=xTaskGetTickCount();
    velocity_x=velocity_y=0.0f;
    taskEXIT_CRITICAL();
}

/* axis=0校正X，axis=1校正Y；另一位置轴和当前航向保持不变。 */
void Chassis_OdomCalibrateAxis(uint8_t axis, float position_m)
{
    taskENTER_CRITICAL();
    if(axis==0U) {odom_pose.x_m=position_m;wheel_pose.x_m=position_m;}
    else {odom_pose.y_m=position_m;wheel_pose.y_m=position_m;}
    velocity_x=velocity_y=0.0f;
    odom_last_tick=xTaskGetTickCount();
    taskEXIT_CRITICAL();
}
