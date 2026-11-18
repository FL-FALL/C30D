#include "system.h"
#include "heading_control.h"

static float yaw,rate,target;
static TickType_t sample_tick;
static int valid,have_sample;
/* Debugger-visible diagnostics; existing VOFA channels stay unchanged. */
volatile float HeadingControl_ErrorRad,HeadingControl_CorrectionRps;

static float Wrap(float x)
{
    while(x>3.14159265f) x-=6.28318530f;
    while(x< -3.14159265f) x+=6.28318530f;
    return x;
}

void HeadingControl_Publish(short gyro_z,int calibrated)
{
    TickType_t now=xTaskGetTickCount(),elapsed;
    float new_rate=(float)gyro_z*(HEADING_CONTROL_GYRO_FULL_SCALE_DPS/32768.0f)*
                   (3.14159265f/180.0f)*HEADING_CONTROL_GYRO_SIGN;
    taskENTER_CRITICAL();
    if(!calibrated) {
        yaw=rate=0.0f; valid=have_sample=0;
        sample_tick=now;
        taskEXIT_CRITICAL();
        return;
    }
    elapsed=(TickType_t)(now-sample_tick);
    if(have_sample && elapsed>pdMS_TO_TICKS(HEADING_CONTROL_SAMPLE_TIMEOUT_MS)) {
        /* Lost integration interval: require a reboot/recalibration. */
        valid=0;
    } else if(have_sample && valid) {
        yaw=Wrap(yaw+0.5f*(rate+new_rate)*(float)elapsed*
                 (float)portTICK_PERIOD_MS*0.001f);
    } else if(!have_sample) {
        valid=1;
    }
    rate=new_rate; sample_tick=now; have_sample=1;
    taskEXIT_CRITICAL();
}

int HeadingControl_Ready(void)
{
    int ready;
    taskENTER_CRITICAL();
    ready=valid && have_sample &&
          (TickType_t)(xTaskGetTickCount()-sample_tick)<=pdMS_TO_TICKS(HEADING_CONTROL_SAMPLE_TIMEOUT_MS);
    taskEXIT_CRITICAL();
    return ready;
}

void HeadingControl_Reset(void)
{
    taskENTER_CRITICAL();
    target=yaw;
    HeadingControl_ErrorRad=HeadingControl_CorrectionRps=0.0f;
    taskEXIT_CRITICAL();
}

float HeadingControl_Update(int translating)
{
    float error,w,output;
    taskENTER_CRITICAL();
    error=Wrap(target-yaw); w=rate;
    taskEXIT_CRITICAL();
    HeadingControl_ErrorRad=error;
    output=0.0f;
    if(translating && HeadingControl_Ready()) {
        if(!(error>=-HEADING_CONTROL_YAW_TOLERANCE && error<=HEADING_CONTROL_YAW_TOLERANCE &&
             w>=-HEADING_CONTROL_GYRO_TOLERANCE && w<=HEADING_CONTROL_GYRO_TOLERANCE))
            output=HEADING_CONTROL_KP*error-HEADING_CONTROL_KD*w;
        if(output>HEADING_CONTROL_MAX_ANGULAR_VELOCITY) output=HEADING_CONTROL_MAX_ANGULAR_VELOCITY;
        if(output< -HEADING_CONTROL_MAX_ANGULAR_VELOCITY) output=-HEADING_CONTROL_MAX_ANGULAR_VELOCITY;
    }
    HeadingControl_CorrectionRps=output;
    return output;
}

float HeadingControl_GetYawOffsetDeg(void)
{
    float offset;
    taskENTER_CRITICAL();
    offset=Wrap(yaw-target);
    taskEXIT_CRITICAL();
    return offset*(180.0f/3.14159265f);
}
