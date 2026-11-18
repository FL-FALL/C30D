#include "system.h"
#include "arm_servo.h"
#include "vision_uart.h"
#include "arm_task.h"
#include <stdio.h>


static char last_vision_line[VISION_UART_LINE_MAX];
static uint32_t vision_line_count;

uint32_t ArmTask_GetLastVisionLine(char *out, uint8_t capacity)
{
    uint8_t i;
    uint32_t received;
    if (out == 0 || capacity == 0U) return 0U;
    taskENTER_CRITICAL();
    for (i = 0U; i < capacity - 1U && last_vision_line[i] != '\0'; ++i)
        out[i] = last_vision_line[i];
    out[i] = '\0';
    received = vision_line_count;
    taskEXIT_CRITICAL();
    return received;
}

/* Old autonomous demo removed. */
void Arm_task(void *argument)
{
    (void)argument;
    vTaskDelete(NULL);
}

/* Physical slot 1/2/3 along the search direction, NOT color. */
static uint8_t mission_ball_slot;
void ArmMission_SetBallSlot(uint8_t slot) { mission_ball_slot=slot; }

/* Edit the nine angle numbers HERE. Index: 0=PC6, 1=PC7, 2=PC8, 3=PC9.
 * Angles are signed targets relative to calibrated zero, not increments.
 * The route also reuses these actions when normal route mode is restored.
 */
uint32_t ArmMission_Apply(ArmMissionAction action)
{
    switch(action) {
    case ARM_MISSION_CENTER_GRASP: /* One shot after 1s centered; absolute targets. */
        (void)Arm_SetAngle(0, 90);
        (void)Arm_SetAngle(1, (-20));
        (void)Arm_SetAngle(2, (-100));
        return 2000U;
    case ARM_MISSION_EXTEND: /* Step 1: two joints */
        (void)Arm_SetAngle(0, 50);
        (void)Arm_SetAngle(1, 50);
        return 2000U;
    case ARM_MISSION_APPROACH: /* After camera X alignment: reach a little forward. */
        (void)Arm_SetAngle(0, 90); /* EDIT reach pose here; absolute target. */
        (void)Arm_SetAngle(1, -50);
        return 1000U;
    case ARM_MISSION_PICK:   /* EDIT grasp joint/angle here (retains existing joint 2). */
        (void)Arm_SetAngle(2, -80);
        return 2000U;
    case ARM_MISSION_CARRY: /* Keep claw closed while withdrawing from the ball stand. */
        (void)Arm_SetAngle(0, 90);
        (void)Arm_SetAngle(1, 0);
        return 1000U;
    case ARM_MISSION_BUCKET: /* Turn/pose FIRST. Do not open joint 2 here. */
        (void)Arm_SetAngle(0, -80);
        (void)Arm_SetAngle(1, 90);
        /* Initial examples only: calibrate these three absolute pan targets. */
        if(mission_ball_slot==1U) (void)Arm_SetAngle(3, 75);
        else if(mission_ball_slot==2U) (void)Arm_SetAngle(3, 85);
        else if(mission_ball_slot==3U) (void)Arm_SetAngle(3, 95);
        return 2000U;
    case ARM_MISSION_RELEASE: /* At bucket, after turn/pose settling. */
        (void)Arm_SetAngle(2, -180);
        return 1000U;
    case ARM_MISSION_BUCKET_FINISH: /* Retract, keep claw open. */
        (void)Arm_SetAngle(0, 0);
        (void)Arm_SetAngle(1, 0);
        (void)Arm_SetAngle(3, 85);
        return 1000U;
    default: return 0U;
    }
}

/* One shot per boot. PWM holds the last targets after this task exits. */
void ArmMission_TestTask(void *argument)
{
    (void)argument;
    Chassis_Disarm();
    /* Grasp-only test: no extend, carry, bucket or release sequence. */
    vTaskDelay(pdMS_TO_TICKS(ArmMission_Apply(ARM_MISSION_CENTER_GRASP)));
    vTaskDelete(NULL);
}

/* Nonblocking stationary-chassis pick/drop sequence. */

volatile ArmBucketState Arm_BucketState=BUCKET_IDLE;
static uint8_t bucket_slot;
static uint32_t bucket_tick,bucket_delay;
static int BucketPose3(int a,int b,int claw)
{
    if(!Arm_SetAngle(0,a)) return 0;
    if(!Arm_SetAngle(1,b)) return 0;
    return Arm_SetAngle(2,claw);
}
int ArmBucket_Begin(uint8_t slot,uint32_t now)
{
    if(Arm_BucketState!=BUCKET_IDLE || slot<1U || slot>3U) return 0;
    bucket_slot=slot;
    if(!BucketPose3(90,(-20),(-180))) {
        Arm_BucketState=BUCKET_ERROR;return 0;
    }
    Arm_BucketState=BUCKET_PREPARE;bucket_tick=now;bucket_delay=1000U;
    return 1;
}
int ArmBucket_Update(uint32_t now)
{
    int ok=1;
    if(Arm_BucketState==BUCKET_ERROR) return -1;
    if(Arm_BucketState==BUCKET_DONE) return 1;
    if(Arm_BucketState==BUCKET_IDLE) return -1;
    if((uint32_t)(now-bucket_tick)<bucket_delay) return 0;
    bucket_tick=now;
    switch(Arm_BucketState) {
    case BUCKET_PREPARE:
        ok=BucketPose3(90,(-20),(-70));
        Arm_BucketState=BUCKET_GRASP;bucket_delay=2000U;break;
    case BUCKET_GRASP:
        ok=Arm_SetAngle(0,50);
        Arm_BucketState=BUCKET_LIFT;bucket_delay=1000U;break;
    case BUCKET_LIFT:
        if(bucket_slot==1U) {
            ok=Arm3_SetPwmPositionDeg(75); /* EDIT slot 1 bucket heading, 0..270. */
            Arm_BucketState=BUCKET_SLOT1_TURN;bucket_delay=2000U;
        } else if(bucket_slot==2U) {
            ok=Arm3_SetPwmPositionDeg(100); /* EDIT slot 2 bucket heading, 0..270. */
            Arm_BucketState=BUCKET_SLOT2_TURN;bucket_delay=2000U;
        } else {
            ok=Arm3_SetPwmPositionDeg(135); /* EDIT slot 3 bucket heading, 0..270. */
            Arm_BucketState=BUCKET_SLOT3_TURN;bucket_delay=2000U;
        }
        break;
    case BUCKET_SLOT1_TURN:
        ok=Arm_SetAngle(1,0);
        Arm_BucketState=BUCKET_SLOT1_LOWER;bucket_delay=1000U;break;
    case BUCKET_SLOT1_LOWER:
        ok=Arm_SetAngle(2,(-180));
        Arm_BucketState=BUCKET_SLOT1_RELEASE;bucket_delay=1000U;break;
    case BUCKET_SLOT2_TURN:
        ok=Arm_SetAngle(1,0);
        Arm_BucketState=BUCKET_SLOT2_LOWER;bucket_delay=1000U;break;
    case BUCKET_SLOT2_LOWER:
        ok=Arm_SetAngle(2,(-180));
        Arm_BucketState=BUCKET_SLOT2_RELEASE;bucket_delay=1000U;break;
    case BUCKET_SLOT3_TURN:
        ok=Arm_SetAngle(1,0);
        Arm_BucketState=BUCKET_SLOT3_LOWER;bucket_delay=1000U;break;
    case BUCKET_SLOT3_LOWER:
        ok=Arm_SetAngle(2,(-180));
        Arm_BucketState=BUCKET_SLOT3_RELEASE;bucket_delay=1000U;break;
    case BUCKET_SLOT1_RELEASE:case BUCKET_SLOT2_RELEASE:case BUCKET_SLOT3_RELEASE:
        Arm_BucketState=BUCKET_DONE;return 1;
    default:ok=0;break;
    }
    if(!ok) {Arm_BucketState=BUCKET_ERROR;return -1;}
    return 0;
}

/* Corner-5 observation pose, once at power-up while chassis is locked. */
/* One immediate pose step; route task owns the nonblocking 1-second waits. */
void Arm_Corner5ObserveStep(uint8_t step)
{
    if(step==1U) {
        (void)Arm_SetAngle(2,-180);
    } else if(step==2U) {
        (void)Arm_SetAngle(1,80);
        (void)Arm_SetAngle(0,20);
    }
}

void Arm_Corner5Observe(void)
{
    delay_ms(1000U);
    Arm_Corner5ObserveStep(1U);
    delay_ms(1000U);
    Arm_Corner5ObserveStep(2U);
    delay_ms(1000U);
}

/* Called once AFTER centered and wheels stopped. Chassis is disarmed first.
 * All calibrated actions live here; no other arm task runs in this test.
 */
int Arm_Corner5PickDrop(uint8_t slot)
{
    if(slot<1U || slot>3U) return 0;
    /* Reach. */
    (void)Arm_SetAngle(0,-45);
    (void)Arm_SetAngle(2,-220);
    (void)Arm_SetAngle(1,100);
    delay_ms(2000U);
    /* Grasp. */
    (void)Arm_SetAngle(2,-70);
    delay_ms(2000U);
    /* Lift and turn, as in the calibrated bench sequence. */
    (void)Arm_SetAngle(1,0);
	delay_ms(2000U);
    if(slot==1U) {
        if(!Arm3_SetPwmPositionDeg(105)) return 0;
			delay_ms(1500U);
			 ArmServo_Disable(3);
			(void)Arm_SetAngle(0,-20);
			(void)Arm_SetAngle(1,70);
    } else if(slot==2U) {
        if(!Arm3_SetPwmPositionDeg(72)) return 0;
						delay_ms(1500U);
			 ArmServo_Disable(3);
			(void)Arm_SetAngle(0,0);
						(void)Arm_SetAngle(1,50);

    } else {
        if(!Arm3_SetPwmPositionDeg(30)) return 0;
						delay_ms(2000U);
			 ArmServo_Disable(3);
			(void)Arm_SetAngle(0,-20);
						(void)Arm_SetAngle(1,70);
    }
    delay_ms(1000U);
    /* 松爪后保持输出，打印脉宽以确认指令执行且没有被覆盖。 */
    (void)Arm_SetAngle(2,-180);
    printf("[CLAW] release pwm=%lu\r\n", (unsigned long)TIM8->CCR3);
    delay_ms(1000U);
    printf("[CLAW] hold pwm=%lu\r\n", (unsigned long)TIM8->CCR3);


    return 1;
}

/* 救援观察姿态在这里改：以下为待现场调整的初始角度，不影响抓球动作。 */
uint32_t Arm_RescueObserveStep(uint8_t step)
{
    switch(step) {
    case 0U:
        (void)Arm3_SetPwmPositionDeg(135); /* 修改相机朝向救援台的角度，范围0到270。 */
        return 2000U;
    case 1U:

        Arm3_SetPwmPositionDeg(200);
        delay_ms(3000U);        			 
                ArmServo_Disable(3);
        (void)Arm_SetAngle(0,30); /* 修改救援观察姿态的0号关节。 */
        (void)Arm_SetAngle(1,-50);  /* 修改救援观察姿态的1号关节。 */
        return 2000U;
    default:return 0U;
    }
}

/* 救援抓取动作在这里改：按伸爪、闭爪、抬起顺序执行，最终保持夹紧。 */
uint32_t Arm_RescuePickStep(uint8_t step)
{
    switch(step) {
    case 0U:
        (void)Arm_SetAngle(0,-30);
        (void)Arm_SetAngle(2,-180);
        (void)Arm_SetAngle(1,100);
        return 2000U;
    case 1U:
        (void)Arm_SetAngle(2,-70);
        return 2000U;
    case 2U:
        (void)Arm_SetAngle(1,0);
        return 2000U;
    default:return 0U;
    }
}
