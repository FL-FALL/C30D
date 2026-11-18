#include "system.h"
#include "chassis_local.h"
#include "chassis_config.h"
#include "heading_control.h"

static float local_vx, local_vy, local_wz;
static TickType_t command_tick;
static int motion_enabled;

static int HardwareReady(void)
{
    return !CHASSIS_ARM_ONLY_TEST && SysVal.Time_count >= CONTROL_DELAY && EN != 0 &&
           Voltage >= 10.0f && Voltage < 100.0f &&
           (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550);
}

static void ClearCommand(void)
{
    local_vx=0; local_vy=0; local_wz=0;
    Move_X=0; Move_Y=0; Move_Z=0;
    motion_enabled=0;
}

void Chassis_LocalInit(void)
{
    /* Called once before scheduling. Do not enter a FreeRTOS critical section. */
    Flag_Stop=1;
    ClearCommand();
    command_tick=0;
    Allow_Recharge=0;
    PS2_ON_Flag=0; APP_ON_Flag=0; Remote_ON_Flag=0;
    CAN_ON_Flag=0; Usart1_ON_Flag=0; Usart5_ON_Flag=0;
}

int Chassis_Arm(void)
{
    int ok;
    taskENTER_CRITICAL();
#if CHASSIS_QR_NOTIFY_ONLY || CHASSIS_ARM_ONLY_TEST
    ok=0; /* Notification-only test must never enable motors, including by key. */
#else
    ok=HardwareReady();
#if HEADING_CONTROL_ENABLE
    ok=ok && HeadingControl_Ready();
#endif
#endif
    ClearCommand();
    Flag_Stop=ok ? 0 : 1;
#if HEADING_CONTROL_ENABLE
    if(ok) HeadingControl_Reset();
#endif
    command_tick=xTaskGetTickCount();
    taskEXIT_CRITICAL();
    return ok;
}

void Chassis_Disarm(void)
{
    taskENTER_CRITICAL();
    Flag_Stop=1;
    ClearCommand();
    taskEXIT_CRITICAL();
}

int Chassis_IsArmed(void)
{
    return Flag_Stop==0 && HardwareReady();
}

int Chassis_SetVelocity(float vx,float vy,float wz)
{
    int ok;
    /* Inclusive comparisons also reject NaN and infinities. */
    taskENTER_CRITICAL();
    ok=Chassis_IsArmed() &&
       vx>=-CHASSIS_LOCAL_MAX_LINEAR_MPS && vx<=CHASSIS_LOCAL_MAX_LINEAR_MPS &&
       vy>=-CHASSIS_LOCAL_MAX_LINEAR_MPS && vy<=CHASSIS_LOCAL_MAX_LINEAR_MPS &&
       wz>=-CHASSIS_LOCAL_MAX_ANGULAR_RPS && wz<=CHASSIS_LOCAL_MAX_ANGULAR_RPS;
    if(ok) {
        local_vx=vx; local_vy=vy; local_wz=wz;
        command_tick=xTaskGetTickCount();
    } else {
        ClearCommand();
        Flag_Stop=1;
    }
    taskEXIT_CRITICAL();
    return ok;
}

static int Translate(float speed,int axis,int sign)
{
    if(!(speed>=0.0f && speed<=CHASSIS_LOCAL_MAX_LINEAR_MPS)) {
        Chassis_Disarm();
        return 0;
    }
    return Chassis_SetVelocity(axis==0 ? sign*speed : 0.0f,
                               axis==1 ? sign*speed : 0.0f,0.0f);
}
int Chassis_Forward(float speed)  { return Translate(speed,0, 1); }
int Chassis_Backward(float speed) { return Translate(speed,0,-1); }
int Chassis_Left(float speed)     { return Translate(speed,1, 1); }
int Chassis_Right(float speed)    { return Translate(speed,1,-1); }
int Chassis_Rotate(float speed)   { return Chassis_SetVelocity(0,0,speed); }

void Chassis_Stop(void)
{
    taskENTER_CRITICAL();
    ClearCommand(); /* Keep armed, so a subsequent command may start a new segment. */
    taskEXIT_CRITICAL();
}

void Chassis_LocalUpdate(void)
{
    taskENTER_CRITICAL();
    if(!Chassis_IsArmed()) {
        Flag_Stop=1;
        ClearCommand();
    } else if ((local_vx!=0 || local_vy!=0 || local_wz!=0) &&
               (TickType_t)(xTaskGetTickCount()-command_tick)>=pdMS_TO_TICKS(CHASSIS_LOCAL_TIMEOUT_MS)) {
        Flag_Stop=1; /* Timeout requires a deliberate re-arm. */
        ClearCommand();
    }
#if HEADING_CONTROL_ENABLE
    if(Flag_Stop==0 && !HeadingControl_Ready()) {
        Flag_Stop=1;
        ClearCommand();
    }
#endif
    Move_X=local_vx; Move_Y=local_vy; Move_Z=local_wz;
#if HEADING_CONTROL_ENABLE
    if(Flag_Stop==0) {
        if(local_wz!=0.0f) HeadingControl_Reset();
        else Move_Z=HeadingControl_Update(local_vx!=0.0f || local_vy!=0.0f);
    }
#endif
    motion_enabled=(Move_X!=0 || Move_Y!=0 || Move_Z!=0) && Flag_Stop==0;
    taskEXIT_CRITICAL();
    /* Convert route/body Y to the physical wheel convention once. */
    Drive_Motor(Move_X,CHASSIS_LOCAL_Y_SIGN*Move_Y,Move_Z);
}

int Chassis_MotionEnabled(void)
{
    return motion_enabled && Chassis_IsArmed();
}
