#include "system.h"
#include "chassis_odom.h"
#include "vision_uart.h"
#include "vofa_uart.h"
#include <math.h>
#include "arm_task.h"
#include "arm_servo.h"
#include "target_align_config.h"
#include "arm_mission_config.h"

/* 路线点为底盘中心相对实际起点的坐标，单位：米。
 * X正方向为初始前方，Y正方向为左方；向右横移时Y减小。
 * FIELD_X会减去起点前移补偿量0.10米。
 * 路线数组每项依次为：X坐标、Y坐标、到点执行的动作；不是抓球动作的case。
 * 相邻路线点必须仅改变一个坐标，不能同时改变X和Y。
 */
#define START_FORWARD_OFFSET_M 0.10f
#define FIELD_X(x) ((x)-START_FORWARD_OFFSET_M)

typedef enum {
    PASS_POINT, /* 普通路口：停稳后继续下一段 */
    SCAN_QR, /* 扫码路口：等待二维码成功后放行 */
    PAUSE_3S, /* 旧版任务暂停3秒 */
    ARM_EXTEND, /* 第五拐口：观察姿态并开始搜球 */
    BALL_SEARCH_END, /* 搜索终点：未识别到目标球时停下等待 */
    ARM_BUCKET /* 旧版移动车身到桶边的动作，当前原地放球不走此分支 */
} RouteAction;
typedef struct { float x, y; RouteAction action; } RoutePoint;
static const RoutePoint route[] = {
    {0.000f, 0.000f, PASS_POINT}, /* 0：实际起点；初始化及姿态条件满足后，开始驶向第1路口。 */
    {FIELD_X(0.720f), 0.000f, SCAN_QR}, /* 1：第一路口；停稳、伸臂，等待二维码成功，通知相机待机后继续跑图。 */
    {FIELD_X(0.720f), 0.630f, PASS_POINT}, /* 2：扫码后的左移路口；从第1点沿Y正方向移动到这里。 */
    {FIELD_X(1.660f), 0.630f, PASS_POINT}, /* 3：前进路口；保持Y不变，沿X正方向前进。 */
    {FIELD_X(1.660f), 1.460f, PASS_POINT}, /* 4：再次左移的路口；保持X不变，沿Y正方向移动。 */
    {FIELD_X(2.590f), 1.460f, ARM_EXTEND}, /* 5：抓球准备点；停稳并分步进入观察姿态，通知相机识球，随后向右搜索。 */
    {FIELD_X(2.590f), ARM_LAST_BALL_Y_M, BALL_SEARCH_END}, /* 6：抓球搜索终点；未收到目标球停车信号时，最多右移到此处停车等待。 */
    {FIELD_X(2.590f), 0.855f, ARM_BUCKET}, /* 7：旧版桶位置，当前抓取后原地转臂放球，不再驶向此点。 */
    {FIELD_X(2.590f),-0.930f, PAUSE_3S},   /* 8：旧版打靶路线点；当前由独立打靶状态控制，减速位置见target_align_config.h。 */
    {FIELD_X(2.590f),-1.400f, PASS_POINT}, /* 9：打靶完成后的外侧拐角；到达这里后沿X负方向前往救援区。 */
    {FIELD_X(1.670f),-1.400f, PAUSE_3S},   /* 10：救援任务中间位置；在此点前0.3米停车摆臂，随后慢速搜索。 */
    {FIELD_X(0.100f),-1.400f, PASS_POINT}  /* 11：返回区终点；救援标定后到达这里停车，保持夹爪抓紧。 */
};

#define ROUTE_COUNT ((unsigned int)(sizeof(route)/sizeof(route[0])))
#define ROUTE_FIFTH_CORNER_TEST 0 /* 0：从起点完整跑图；1：从第五拐口开始测试。 */
#define ROUTE_MAX_SPEED 0.30f
/* Hold the measured cross-axis coordinate captured at each segment start. */
#define ROUTE_HOLD_OUTPUT_SIGN (1.0f) /* Cross-axis trim sign; independent of IMU yaw sign. */
#define ROUTE_HOLD_KP 4.0f
#define ROUTE_HOLD_DEADBAND_M 0.005f
#define ROUTE_HOLD_MAX_MPS 0.70f
#define ROUTE_HOLD_SLEW_MPS2 0.80f
/* Body axes approximate map axes while heading hold keeps yaw small. */
#define ROUTE_HOLD_MAX_YAW_RAD 0.52359878f
#define ROUTE_TASK_PAUSE_MS 3000U
#define ROUTE_ARRIVAL_M 0.005f
#define ROUTE_MAX_DURATION_MS 240000U /* Movement time; excludes task waits. */
#define ROUTE_STALL_MS 3000U
#define ROUTE_STOP_SPEED_MPS 0.005f
#define ROUTE_STOP_STABLE_MS 100U
#define ROUTE_STOP_TIMEOUT_MS 2000U

typedef enum {
    ROUTE_MOVING, /* 按路线行驶 */
    ROUTE_BRAKING, /* 到点制动并确认停稳 */
    ROUTE_WAIT_QR, /* 第一路口等待二维码信息 */
    ROUTE_WAIT_BALL, /* 到达搜球终点，停车等待目标球 */
    ROUTE_ALIGN_BALL, /* 收到目标球后，按XY平移对中；居中稳定后执行抓取放球 */
    ROUTE_DWELL, /* 动作后的非阻塞等待阶段 */
    ROUTE_ARM_BUCKET, /* 旧版机械臂放桶状态 */
    ROUTE_OBSERVE, /* 第五拐口观察动作分步执行，等待期间继续更新里程计 */
    ROUTE_TARGET_TRAVEL, /* 抓球完成后，以实际X为基准，0.3米每秒前往打靶区 */
    ROUTE_TARGET_POSE, /* 打靶前停车调整机械臂姿态，非阻塞等待1秒 */
    ROUTE_TARGET_SEARCH, /* 以0.15米每秒向右寻找二维码指定颜色靶 */
    ROUTE_TARGET_ALIGN, /* 靶心对中后进入激光保持阶段。 */
    ROUTE_TARGET_LASER, /* 停车保持激光高电平2秒。 */
    ROUTE_TARGET_COOLDOWN, /* 激光关闭，完成地标标定后等待0.5秒。 */
    ROUTE_AFTER_TARGET, /* 以0.3米每秒沿Y负方向到最后外侧拐角。 */
    ROUTE_RESCUE_TRAVEL, /* 车头不转，沿X负方向前往救援准备点。 */
    ROUTE_RESCUE_POSE, /* 在任务点前0.3米停车，执行可编辑的观察姿态。 */
    ROUTE_RESCUE_SEARCH, /* 以0.15米每秒沿X负方向搜索指定形状。 */
    ROUTE_RESCUE_ALIGN, /* 只使用图像中心X，沿底盘X轴对中。 */
    ROUTE_RESCUE_PICK, /* 停车顺序执行可编辑的救援抓取动作。 */
    ROUTE_RETURN /* 按救援位次标定后，沿X负方向驶向终点。 */
} RoutePhase;
static RoutePhase phase;
volatile unsigned int Route_FailureReason;
unsigned int Chassis_RoutePhase(void) { return (unsigned int)phase; }
static unsigned int next_point=1;
static unsigned int previous_point=0; /* Last visited point, not necessarily next_point-1. */
static unsigned char departure_reported;
static uint32_t qr_baseline;
static unsigned char qr_requested;
static TickType_t qr_request_tick;
static TickType_t observe_tick;
static uint8_t observe_step;
static TickType_t target_tick,target_request_tick,target_center_tick;
static uint32_t target_stop_baseline,target_frame_baseline;
static uint8_t target_centered;
static uint8_t target_pose_step;
static float target_reference_x;

/* 以下坐标按现有路线任务中心和示意图约0.15米间距估算，现场实测后改这里。 */
static const float target_slot_y[3]={-0.680f,-0.830f,-0.980f}; /* 实测提前0.1米拐角：标定Y加0.1米，增加到拐角的剩余距离。 */
static const float rescue_slot_x[3]={FIELD_X(1.820f),FIELD_X(1.670f),FIELD_X(1.520f)};
#define RESCUE_PREPARE_AHEAD_M 0.30f /* 救援任务点前0.3米停车摆臂。 */
#define RESCUE_SEARCH_MPS 0.15f
#define RESCUE_ALIGN_TOL_X 0.10f /* 救援X死区，320宽时为中心正负16像素。 */
#define RESCUE_ALIGN_SIGN (-1.0f) /* 图像目标偏右时沿地图X负方向移动。 */
static uint8_t target_slot,rescue_slot,rescue_arm_step;
static TickType_t rescue_tick,rescue_delay,rescue_request_tick;
static uint32_t rescue_stop_baseline,rescue_frame_baseline;
volatile unsigned int Route_TargetSlot,Route_RescueSlot;

static unsigned char balls_mode_pending,balls_wait_ack;
static uint32_t balls_ack_baseline;
static TickType_t balls_request_tick;
static TickType_t last_tick,moving_ticks,progress_tick,stop_tick,still_tick,wait_tick;
static float progress_x,progress_y;
static int segment_hold_captured;
static float segment_hold_reference,segment_hold_velocity;
volatile float Route_HoldReferenceM,Route_HoldErrorM,Route_HoldCorrectionMps;

#include "ball_align_config.h"
static VisionBallSample ball_xy;
static uint32_t ball_frame_baseline,align_generation;
static uint16_t align_sequence;
static uint32_t align_baseline;
static TickType_t align_request_tick;
static unsigned char align_requested;
static unsigned char align_frames,align_still;
static TickType_t align_still_tick;
static float align_start_x,align_start_y;
volatile unsigned char Route_BallAligned;
volatile unsigned char Route_BallColor,Route_BallSlot;
volatile float Route_BallReferenceX,Route_BallReferenceY;
volatile float Route_BallMeasuredX,Route_BallMeasuredY;
/* Estimated chassis positions abreast of physical slots, anchored to bucket.
 * Drawing: lower -> middle -> upper, 150 mm spacing. Colors are drawn by lot.
 * These are initial references, NOT absolute localization measurements. */
#if !CHASSIS_BALL_ALIGN_ONLY_TEST
static const float ball_slot_offset_y[3]={0.150f,0.000f,-0.150f};
#endif
#define BALL_SLOT_MATCH_TOLERANCE_M 0.070f
static uint8_t locked_ball_color,locked_ball_slot;

static void ResetSegmentHold(void)
{
    segment_hold_captured=0;
    segment_hold_velocity=0.0f;
    Route_HoldErrorM=Route_HoldCorrectionMps=0.0f;
}

static float SegmentHold(ChassisPose pose,int along_x,float dt,float limit,float slew)
{
    float coordinate=along_x ? pose.y_m : pose.x_m;
    float error,desired,step;
    if(!segment_hold_captured) {
        segment_hold_reference=coordinate;
        Route_HoldReferenceM=coordinate;
        segment_hold_velocity=0.0f;
        segment_hold_captured=1;
    }
    error=segment_hold_reference-coordinate;
    Route_HoldErrorM=error;
    /* No whole-velocity yaw rotation. Suspend this small body-axis trim if
     * heading departs too far from the startup axes for the approximation. */
    if(!(pose.yaw_rad>=-ROUTE_HOLD_MAX_YAW_RAD && pose.yaw_rad<=ROUTE_HOLD_MAX_YAW_RAD)) {
        segment_hold_velocity=Route_HoldCorrectionMps=0.0f;
        return 0.0f;
    }
    if(error>ROUTE_HOLD_DEADBAND_M) error-=ROUTE_HOLD_DEADBAND_M;
    else if(error< -ROUTE_HOLD_DEADBAND_M) error+=ROUTE_HOLD_DEADBAND_M;
    else error=0.0f;
    desired=ROUTE_HOLD_OUTPUT_SIGN*ROUTE_HOLD_KP*error;
    if(desired>limit) desired=limit;
    if(desired< -limit) desired=-limit;
    /* Do not retain an old-direction trim after crossing the reference. */
    if(desired==0.0f || desired*segment_hold_velocity<0.0f) segment_hold_velocity=0.0f;
    if(!(dt>0.0f && dt<=0.05f)) dt=0.0f;
    step=slew*dt;
    if(desired>segment_hold_velocity+step) segment_hold_velocity+=step;
    else if(desired<segment_hold_velocity-step) segment_hold_velocity-=step;
    else segment_hold_velocity=desired;
    Route_HoldCorrectionMps=segment_hold_velocity;
    return segment_hold_velocity;
}

enum { SEARCH_END_POINT=6, BUCKET_POINT=7 };
typedef enum { AFTER_ADVANCE, AFTER_APPROACH, AFTER_PICK, AFTER_CARRY, AFTER_TEST_FINISH, AFTER_BUCKET_RELEASE, AFTER_BUCKET_FIRST } AfterDwell;
static AfterDwell after_dwell;
static uint32_t ball_baseline;
static unsigned char ball_enabled,ball_pending,ball_captured,braking_for_ball;
static float bucket_start_x,bucket_start_y;
static unsigned char direct_bucket;

static TickType_t dwell_ticks;
static unsigned char route_started,route_finished,route_failed,still;

#if !CHASSIS_BALL_ALIGN_ONLY_TEST
static int IdentifyBallSlot(ChassisPose wheel)
{
    unsigned int i,best=0U;
    float error,min_error=1000.0f;
    for(i=0;i<3U;i++) {
        error=fabsf(wheel.y_m-(route[BUCKET_POINT].y+ball_slot_offset_y[i]));
        if(error<min_error) {min_error=error;best=i;}
    }
    Route_BallMeasuredX=wheel.x_m; Route_BallMeasuredY=wheel.y_m;
    if(min_error>BALL_SLOT_MATCH_TOLERANCE_M) return 0;
    Route_BallSlot=(unsigned char)(best+1U);
    Route_BallColor=locked_ball_color;
    Route_BallReferenceX=route[BUCKET_POINT].x;
    Route_BallReferenceY=route[BUCKET_POINT].y+ball_slot_offset_y[best];
    ArmMission_SetBallSlot(Route_BallSlot);
    return 1;
}

#endif

static void FailAndStop(void)
{
    if(!Route_FailureReason) Route_FailureReason=99;
    ResetSegmentHold();
    route_failed=1;
    if(phase>=ROUTE_TARGET_TRAVEL) GPIO_ResetBits(GPIOC,GPIO_Pin_11); /* 后续任务异常结束时关闭激光。 */
    Chassis_Disarm();
}

static int AllWheelsStopped(void)
{
    float a,b,c,d;
    taskENTER_CRITICAL();
    a=MOTOR_A.Encoder; b=MOTOR_B.Encoder;
    c=MOTOR_C.Encoder; d=MOTOR_D.Encoder;
    taskEXIT_CRITICAL();
    return a>=-ROUTE_STOP_SPEED_MPS && a<=ROUTE_STOP_SPEED_MPS &&
           b>=-ROUTE_STOP_SPEED_MPS && b<=ROUTE_STOP_SPEED_MPS &&
           c>=-ROUTE_STOP_SPEED_MPS && c<=ROUTE_STOP_SPEED_MPS &&
           d>=-ROUTE_STOP_SPEED_MPS && d<=ROUTE_STOP_SPEED_MPS;
}


static void ResumeMotion(ChassisPose wheel,TickType_t now)
{
    ResetSegmentHold();
    phase=ROUTE_MOVING;
    progress_x=wheel.x_m; progress_y=wheel.y_m; progress_tick=now;
}

static void GoToBucket(ChassisPose wheel,TickType_t now)
{
    /* Only used on the outer vertical lane; keep an axis-aligned Y move.
     * The ball can be either side of the bucket, so use the actual start Y. */
    next_point=BUCKET_POINT;
    bucket_start_x=wheel.x_m; bucket_start_y=wheel.y_m;
    /* Restore lane X first, then drive Y to bucket. Preserve measured odometry. */
    direct_bucket=fabsf(route[BUCKET_POINT].x-wheel.x_m)>ROUTE_ARRIVAL_M ? 1U:2U;
    ResumeMotion(wheel,now);
}

static void AdvancePoint(ChassisPose wheel,TickType_t now)
{
    direct_bucket=0U;
    previous_point=next_point;
    ++next_point;
#if CHASSIS_ROUTE_ONLY_TEST
    if(next_point==SEARCH_END_POINT) ++next_point; /* No last-ball search detour. */
#endif
    if(next_point>=ROUTE_COUNT) {
        Chassis_Disarm(); route_finished=1U; return;
    }
    /* Already picked on an earlier leg: follow the original corners to the
     * outer lane, then skip the obsolete last-ball scan endpoint. */
    if(next_point==SEARCH_END_POINT && ball_captured) GoToBucket(wheel,now);
    else ResumeMotion(wheel,now);
}

static void StartBraking(TickType_t now,int for_ball)
{
    Chassis_Stop();
    ResetSegmentHold();
    braking_for_ball=(unsigned char)for_ball;
    phase=ROUTE_BRAKING; still=0U; stop_tick=now;
}

static void StartArmDwell(ArmMissionAction action,AfterDwell next,TickType_t now)
{
    dwell_ticks=pdMS_TO_TICKS(ArmMission_Apply(action));
    after_dwell=next; wait_tick=now; phase=ROUTE_DWELL;
}

/* 打靶和救援共用按整幅图像中心X对中的比例控制，保持与抓球相同的误差定义。 */
static int LandmarkCentered(const VisionTargetSample *sample,uint8_t axis,float sign,
                            float gain,float max_speed,float tolerance,TickType_t now,
                            TickType_t stable_ms)
{
    float error,velocity;
    error=((float)sample->x-0.5f*sample->width)/(0.5f*sample->width);
    if(fabsf(error)>tolerance) {
        target_centered=0U;
        velocity=sign*gain*error;
        if(velocity>max_speed) velocity=max_speed;
        if(velocity< -max_speed) velocity=-max_speed;
        if(!Chassis_SetVelocity(axis==0U?velocity:0.0f,axis==1U?velocity:0.0f,0.0f)) {
            if(axis!=1U) {Route_FailureReason=8;FailAndStop();}
        }
        return 0;
    }
    /* 打靶只按图像X死区计时，不再以编码器停稳判定阻断发射。 */
    if(stable_ms && axis!=1U && !AllWheelsStopped()) {target_centered=0U;return 0;}
    if(!target_centered) {target_centered=1U;target_center_tick=now;}
    return (TickType_t)(now-target_center_tick)>=stable_ms;
}

void Chassis_AutoRouteUpdate(void)
{
    TickType_t now=xTaskGetTickCount();
    ChassisPose pose,wheel_pose;
    float sx,sy,remaining,direction,dx,dy,dt,trim,route_speed;
    int along_x;
    int target_active=(phase>=ROUTE_TARGET_TRAVEL && phase<=ROUTE_TARGET_COOLDOWN);
    if(route_failed || route_finished) return;
    if(!route_started) {
        /* Begin the first leg after the existing startup/IMU readiness checks. */
        if(SysVal.Time_count<CONTROL_DELAY) return;
        if(!Chassis_Arm()) return;
        route_started=1;
        Chassis_OdomReset(); ResetSegmentHold();
        last_tick=progress_tick=now;
#if ROUTE_FIFTH_CORNER_TEST
        if(!Chassis_OdomStartAt(route[5].x,route[5].y)) {Route_FailureReason=1;FailAndStop(); return;}
        next_point=5U;
        progress_x=route[5].x; progress_y=route[5].y;
        StartBraking(now,0); /* Existing corner-5 handler extends arm, then searches. */
#endif
    }
    dt=(float)(TickType_t)(now-last_tick)*portTICK_PERIOD_MS*0.001f;
    if(phase==ROUTE_MOVING) moving_ticks+=(TickType_t)(now-last_tick);
    last_tick=now;
    if(!Chassis_IsArmed()) {
        if(target_active) {
            GPIO_ResetBits(GPIOC,GPIO_Pin_11); /* 底层失能时关闭激光，不锁死打靶任务。 */
            return;
        }
        Route_FailureReason=2;FailAndStop();return;
    }
    if(!target_active && moving_ticks>pdMS_TO_TICKS(ROUTE_MAX_DURATION_MS)) {
        Route_FailureReason=2;FailAndStop();return;
    }
    /* 打靶阶段偶发里程计采样失败不结束任务，下次从当前时刻继续更新。 */
    if(!Chassis_OdomUpdate()) {
        if(target_active) Chassis_OdomResumeAfterStop();
        else {Route_FailureReason=3;FailAndStop();return;}
    }
    pose=Chassis_OdomGetPose(); wheel_pose=Chassis_OdomGetWheelPose();
    /* Appended target mission; earlier ball/QR states retain their logic. */
    if(phase>=ROUTE_TARGET_TRAVEL) {
        VisionTargetSample target;
        /* 打靶完成后新增的路线和救援任务，所有等待都继续更新里程计。 */
        if(phase==ROUTE_TARGET_LASER) {
            Chassis_Stop();
            if((TickType_t)(now-target_tick)<pdMS_TO_TICKS(2000U)) return;
            GPIO_ResetBits(GPIOC,GPIO_Pin_11);
            Chassis_OdomCalibrateAxis(1U,target_slot_y[target_slot-1U]);
            Route_TargetSlot=target_slot;
            printf("[CAL] target slot=%u Y=%.3f\r\n",(unsigned)target_slot,(double)target_slot_y[target_slot-1U]);
            ResetSegmentHold();
            /* 激光关闭并完成原有靶位标定后，立即恢复原速跑向外侧拐角。 */
            pose=Chassis_OdomGetPose();wheel_pose=Chassis_OdomGetWheelPose();
            phase=ROUTE_AFTER_TARGET;
        }
        if(phase==ROUTE_TARGET_COOLDOWN) {
            Chassis_Stop();
            if((TickType_t)(now-target_tick)<pdMS_TO_TICKS(500U)) return;
            phase=ROUTE_AFTER_TARGET;
        }
        if(phase==ROUTE_AFTER_TARGET) {
            if(wheel_pose.y_m<=route[9].y+ROUTE_ARRIVAL_M) {
                Chassis_Stop();
                if(!AllWheelsStopped()) return;
                /* 到拐角后直接驶向终点，跳过人质准备、搜索、对中及抓取。 */
                ResetSegmentHold();phase=ROUTE_RETURN;
            } else {
                trim=SegmentHold(pose,0,dt,ROUTE_HOLD_MAX_MPS,ROUTE_HOLD_SLEW_MPS2);
                if(!Chassis_SetVelocity(trim,-ROUTE_MAX_SPEED,0.0f)) {Route_FailureReason=8;FailAndStop();}
                return;
            }
        }
        if(phase==ROUTE_RESCUE_TRAVEL || phase==ROUTE_RETURN) {
            float finish_x=(phase==ROUTE_RETURN)?route[11].x:route[10].x+RESCUE_PREPARE_AHEAD_M;
            if(wheel_pose.x_m<=finish_x+ROUTE_ARRIVAL_M) {
                Chassis_Stop();
                if(!AllWheelsStopped()) return;
                if(phase==ROUTE_RETURN) {Chassis_Disarm();route_finished=1U;return;}
                ResetSegmentHold();phase=ROUTE_RESCUE_POSE;
                rescue_arm_step=0U;rescue_delay=0U;rescue_tick=now;
            } else {
                trim=SegmentHold(pose,1,dt,ROUTE_HOLD_MAX_MPS,ROUTE_HOLD_SLEW_MPS2);
                if(!Chassis_SetVelocity(-ROUTE_MAX_SPEED,trim,0.0f)) {Route_FailureReason=8;FailAndStop();}
                return;
            }
        }
        if(phase==ROUTE_RESCUE_POSE || phase==ROUTE_RESCUE_PICK) {
            Chassis_Stop();
            if((TickType_t)(now-rescue_tick)<rescue_delay) return;
            if(phase==ROUTE_RESCUE_POSE) {
                rescue_delay=pdMS_TO_TICKS(Arm_RescueObserveStep(rescue_arm_step++));
                if(rescue_delay) {rescue_tick=now;return;}
                rescue_slot=0U;rescue_stop_baseline=VisionUart_RescueStopCount();
                phase=ROUTE_RESCUE_SEARCH;ResetSegmentHold();
                (void)VisionUart_SendRescueMode();rescue_request_tick=now;
            } else {
                rescue_delay=pdMS_TO_TICKS(Arm_RescuePickStep(rescue_arm_step++));
                if(rescue_delay) {rescue_tick=now;return;}
                Chassis_OdomCalibrateAxis(0U,rescue_slot_x[rescue_slot-1U]);
                Route_RescueSlot=rescue_slot;
                printf("[CAL] rescue slot=%u X=%.3f\r\n",(unsigned)rescue_slot,(double)rescue_slot_x[rescue_slot-1U]);
                ResetSegmentHold();phase=ROUTE_RETURN;
                return;
            }
        }
        if(phase==ROUTE_RESCUE_SEARCH) {
            if(VisionUart_RescueStopCount()!=rescue_stop_baseline) {
                Chassis_Stop();phase=ROUTE_RESCUE_ALIGN;target_centered=0U;
                (void)VisionUart_GetRescue(&target);rescue_frame_baseline=target.generation;
                (void)VisionUart_SendChassisStopped();rescue_request_tick=now;return;
            }
            if((TickType_t)(now-rescue_request_tick)>=pdMS_TO_TICKS(500U)) {
                if(VisionUart_SendRescueMode()) rescue_request_tick=now;
            }
            trim=SegmentHold(pose,1,dt,ROUTE_HOLD_MAX_MPS,ROUTE_HOLD_SLEW_MPS2);
            if(!Chassis_SetVelocity(-RESCUE_SEARCH_MPS,trim,0.0f)) {Route_FailureReason=8;FailAndStop();}
            return;
        }
        if(phase==ROUTE_RESCUE_ALIGN) {
            Chassis_Stop();
            (void)VisionUart_GetRescue(&target);
            if(!target.generation || target.generation==rescue_frame_baseline ||
               (TickType_t)(now-target.received_tick)>pdMS_TO_TICKS(BALL_ALIGN_FRESH_MS)) {
                if((TickType_t)(now-rescue_request_tick)>=pdMS_TO_TICKS(BALL_ALIGN_REQUEST_MS)) {
                    if(VisionUart_SendChassisStopped()) rescue_request_tick=now;
                }
                target_centered=0U;return;
            }
            if(!target.valid) {target_centered=0U;return;}
            if(!rescue_slot) rescue_slot=target.slot;
            if(LandmarkCentered(&target,0U,RESCUE_ALIGN_SIGN,BALL_ALIGN_KP_Y,
                 BALL_ALIGN_MAX_MPS,RESCUE_ALIGN_TOL_X,now,0U) && rescue_slot) {
                /* X进入死区即停车并转入抓取，下一调度周期开始动作，不增加居中等待。 */
                phase=ROUTE_RESCUE_PICK;rescue_arm_step=0U;rescue_delay=0U;rescue_tick=now;
            }
            return;
        }

        if(phase==ROUTE_TARGET_TRAVEL || phase==ROUTE_TARGET_SEARCH) {
            if(phase==ROUTE_TARGET_SEARCH && VisionUart_TargetStopCount()!=target_stop_baseline) {
                Chassis_Stop();phase=ROUTE_TARGET_ALIGN;target_centered=0U;

                (void)VisionUart_GetTarget(&target);target_frame_baseline=target.generation;
                target_request_tick=now;
                (void)VisionUart_SendChassisStopped();return;
            }
            if(phase==ROUTE_TARGET_TRAVEL && wheel_pose.y_m<=TARGET_SLOW_Y_M) {
                Chassis_Stop();
                /* 到启动位置先制动，确认车轮停止后再执行打靶姿态。 */
                target_pose_step=0U;
                target_tick=now;phase=ROUTE_TARGET_POSE;return;
            }
            /* 搜靶不设置终点，持续右移直到收到靶子识别信号。 */
            if(phase==ROUTE_TARGET_SEARCH && (TickType_t)(now-target_request_tick)>=pdMS_TO_TICKS(500U)) {
                if(VisionUart_SendTargetMode()) target_request_tick=now;
            }
            trim=SegmentHold(pose,0,dt,TARGET_SEARCH_MPS,ROUTE_HOLD_SLEW_MPS2);
            route_speed=(phase==ROUTE_TARGET_TRAVEL)?TARGET_TRAVEL_MPS:TARGET_SEARCH_MPS;
            (void)Chassis_SetVelocity(trim,-route_speed,0.0f);
            return;
        }
        Chassis_Stop();
        if(phase==ROUTE_TARGET_POSE) {
            if(target_pose_step==0U) {
                /* 停稳后摆臂，不再因停稳等待超时结束任务。 */
                if(!AllWheelsStopped()) return;
                /* 打靶姿态：保留当前关节角度，转向等待期间继续更新里程计。 */
                (void)Arm3_SetPwmPositionDeg(250);
                target_tick=now;target_pose_step=1U;return;
            }
            if(target_pose_step==1U) {
                if((TickType_t)(now-target_tick)<pdMS_TO_TICKS(3000U)) return;
                ArmServo_Disable(3);
                (void)Arm_SetAngle(0,-110);
                (void)Arm_SetAngle(1,110);
                target_tick=now;target_pose_step=2U;return;
            }
            if((TickType_t)(now-target_tick)<pdMS_TO_TICKS(3000U)) return;
            /* 姿态到位后直接右移并通知相机搜靶，不等待相机确认。 */
            target_slot=0U;
            target_stop_baseline=VisionUart_TargetStopCount();
            phase=ROUTE_TARGET_SEARCH;progress_tick=now;
            progress_x=wheel_pose.x_m;progress_y=wheel_pose.y_m;
            (void)Chassis_SetVelocity(0.0f,-TARGET_SEARCH_MPS,0.0f);
            (void)VisionUart_SendTargetMode();target_request_tick=now;
            return;
        }
        /* 对中不限制累计位移，持续根据有效中心坐标左右调整。 */
        (void)VisionUart_GetTarget(&target);
        if((!target.generation || target.generation==target_frame_baseline ||
            (TickType_t)(now-target.received_tick)>pdMS_TO_TICKS(TARGET_FRESH_MS)) &&
           (TickType_t)(now-target_request_tick)>=pdMS_TO_TICKS(TARGET_REQUEST_MS)) {
            if(VisionUart_SendChassisStopped()) target_request_tick=now;
        }
        if(!target.generation || target.generation==target_frame_baseline || !target.valid ||
           (TickType_t)(now-target.received_tick)>pdMS_TO_TICKS(TARGET_FRESH_MS)) {target_centered=0U;return;}
        if(!target_slot) target_slot=target.slot;
        if(LandmarkCentered(&target,1U,TARGET_ALIGN_SIGN,TARGET_ALIGN_KP,
             TARGET_ALIGN_MAX_MPS,TARGET_ALIGN_TOL_X,now,pdMS_TO_TICKS(TARGET_ALIGN_STABLE_MS)) && target_slot) {
            GPIO_SetBits(GPIOC,GPIO_Pin_11);
            target_tick=now;phase=ROUTE_TARGET_LASER;
        }
        return;
    }
    (void)VisionUart_GetBall(&ball_xy);
    /* Retry the camera mode command in the background; never gate motion on ACK. */
    if(balls_mode_pending || balls_wait_ack) {
        if(VisionUart_BallsReadyCount()!=balls_ack_baseline) {
            balls_mode_pending=balls_wait_ack=0U;
        } else if(balls_mode_pending || (TickType_t)(now-balls_request_tick)>=pdMS_TO_TICKS(500U)) {
            if(VisionUart_SendBallsMode()) {
                balls_mode_pending=0U;balls_wait_ack=1U;balls_request_tick=now;
            }
        }
    }
    if(ball_enabled && !ball_captured && (VisionUart_BallStopEventCount()!=ball_baseline ||
       (ball_xy.generation!=ball_frame_baseline && ball_xy.valid && ball_xy.color>=1U && ball_xy.color<=3U &&
        (TickType_t)(now-ball_xy.received_tick)<=pdMS_TO_TICKS(BALL_ALIGN_FRESH_MS))))
        ball_pending=1U;

    /* RED stop -> zero velocity -> request XY -> align. No extra handshake state. */
    if(ball_pending && !ball_captured) {
        Chassis_Stop();ResetSegmentHold();
        ball_pending=0U;ball_enabled=0U;
        balls_mode_pending=balls_wait_ack=0U;
        phase=ROUTE_ALIGN_BALL;
        align_start_x=wheel_pose.x_m;align_start_y=wheel_pose.y_m;
        align_generation=ball_xy.generation;align_sequence=ball_xy.sequence;
        align_baseline=ball_xy.generation;
        align_frames=align_still=0U;locked_ball_color=locked_ball_slot=0U;
        align_requested=0U;
        if(VisionUart_SendChassisStopped()) {align_requested=1U;align_request_tick=now;}
        return;
    }

    if(phase!=ROUTE_MOVING) {
        ResetSegmentHold();
        if(phase!=ROUTE_ALIGN_BALL) Chassis_Stop();
        if(phase==ROUTE_OBSERVE) {
            if((TickType_t)(now-observe_tick)<pdMS_TO_TICKS(1000U)) return;
            observe_tick=now;
            if(observe_step<2U) {
                Arm_Corner5ObserveStep(++observe_step);
            } else {
                balls_ack_baseline=VisionUart_BallsReadyCount();
                ball_baseline=VisionUart_BallStopEventCount();
                ball_frame_baseline=ball_xy.generation;
                balls_mode_pending=1U;balls_wait_ack=0U;ball_enabled=1U;
                if(VisionUart_SendBallsMode()) {
                    balls_mode_pending=0U;balls_wait_ack=1U;balls_request_tick=now;
                }
                AdvancePoint(wheel_pose,now); /* Search right immediately, without ACK. */
            }
            return;
        }
        if(phase==ROUTE_ARM_BUCKET) {
            int result=ArmBucket_Update((uint32_t)now*portTICK_PERIOD_MS);
            if(result<0) FailAndStop();
            else if(result>0) {Chassis_Disarm();route_finished=1U;}
            return;
        }
        if(phase==ROUTE_BRAKING) {
            if((TickType_t)(now-stop_tick)>=pdMS_TO_TICKS(ROUTE_STOP_TIMEOUT_MS)) {
                Route_FailureReason=5;FailAndStop(); return;
            }
            if(!AllWheelsStopped()) { still=0; return; }
            if(!still) { still=1; still_tick=now; return; }
            if((TickType_t)(now-still_tick)<pdMS_TO_TICKS(ROUTE_STOP_STABLE_MS)) return;
            if(braking_for_ball || (ball_pending && !ball_captured)) {
                ball_pending=0U; ball_enabled=0U;
#if CHASSIS_BALL_STOP_ONLY_TEST
                Chassis_Disarm(); route_finished=1U; return;
#else
                phase=ROUTE_ALIGN_BALL;
                 align_start_x=wheel_pose.x_m; align_start_y=wheel_pose.y_m;
                align_generation=ball_xy.generation; align_sequence=ball_xy.sequence;
                align_frames=align_still=0U;
                locked_ball_color=locked_ball_slot=0U;
                align_baseline=ball_xy.generation;
                align_requested=0U;
                /* Notify immediately after encoder-confirmed standstill. */
                if(VisionUart_SendChassisStopped()) {
                    align_requested=1U;align_request_tick=now;
                }
#endif
            } else if(direct_bucket==1U) {
                direct_bucket=2U; bucket_start_y=wheel_pose.y_m;
                ResumeMotion(wheel_pose,now);
#if CHASSIS_ROUTE_ONLY_TEST
            } else if(route[next_point].action==ARM_EXTEND) {
                StartArmDwell(ARM_MISSION_EXTEND,AFTER_ADVANCE,now);
            } else if(route[next_point].action==ARM_BUCKET || route[next_point].action==PAUSE_3S) {
                dwell_ticks=pdMS_TO_TICKS(ROUTE_TASK_PAUSE_MS);
                after_dwell=AFTER_ADVANCE; wait_tick=now; phase=ROUTE_DWELL;
            } else {
                AdvancePoint(wheel_pose,now);
#else
            } else if(route[next_point].action==SCAN_QR) {
                (void)Arm_SetAngle(0,-110);
                (void)Arm_SetAngle(1,110);
                qr_baseline=0U; /* Retain a QR decoded before arrival. */
                qr_requested=0U;
                wait_tick=now;
                phase=ROUTE_WAIT_QR;
            } else if(route[next_point].action==ARM_EXTEND) {
                observe_step=0U;observe_tick=now;phase=ROUTE_OBSERVE;
            } else if(route[next_point].action==BALL_SEARCH_END) {
                phase=ROUTE_WAIT_BALL;
            } else if(route[next_point].action==ARM_BUCKET) {
                StartArmDwell(ARM_MISSION_BUCKET,AFTER_BUCKET_FIRST,now);
            } else if(route[next_point].action==PAUSE_3S) {
                dwell_ticks=pdMS_TO_TICKS(ROUTE_TASK_PAUSE_MS);
                after_dwell=AFTER_ADVANCE; wait_tick=now; phase=ROUTE_DWELL;
            } else AdvancePoint(wheel_pose,now);
#endif
#if CHASSIS_ROUTE_ONLY_TEST
            }
#endif
        } else if(phase==ROUTE_ALIGN_BALL) {
            float error_x,error_y,vx,vy,speed,travel_x,travel_y;
            travel_x=wheel_pose.x_m-align_start_x;
            travel_y=wheel_pose.y_m-align_start_y;
            if(travel_x*travel_x+travel_y*travel_y>BALL_ALIGN_MAX_TRAVEL_M*BALL_ALIGN_MAX_TRAVEL_M) {
                Route_FailureReason=7U;
                FailAndStop(); return;
            }
            /* Retry stopped notification until a post-handshake coordinate arrives. */
            if((ball_xy.generation==align_baseline ||
                (TickType_t)(now-ball_xy.received_tick)>pdMS_TO_TICKS(BALL_ALIGN_FRESH_MS)) &&
               (!align_requested || (TickType_t)(now-align_request_tick)>=pdMS_TO_TICKS(BALL_ALIGN_REQUEST_MS))) {
                if(VisionUart_SendChassisStopped()) {
                    align_requested=1U; align_request_tick=now;
                }
            }
            /* Use received XY directly; only missing/stale coordinates stop alignment. */
            if(!ball_xy.generation ||
               (TickType_t)(now-ball_xy.received_tick)>pdMS_TO_TICKS(BALL_ALIGN_FRESH_MS)) {
                Chassis_Stop(); align_frames=align_still=0U; return;
            }
            if(!locked_ball_slot) locked_ball_slot=(ball_xy.slot>=1U && ball_xy.slot<=3U)?ball_xy.slot:1U;
            locked_ball_color=ball_xy.color;
            error_x=((float)ball_xy.x-0.5f*ball_xy.width)/(0.5f*ball_xy.width);
            error_y=((float)ball_xy.y-0.5f*ball_xy.height)/(0.5f*ball_xy.height);
            if(fabsf(error_x)>BALL_ALIGN_TOL_X || fabsf(error_y)>BALL_ALIGN_TOL_Y) {
                align_frames=align_still=0U;
                /* Already centered axes stay at zero; heading hold remains active. */
                vx=fabsf(error_y)>BALL_ALIGN_TOL_Y ? BALL_ALIGN_X_SIGN*BALL_ALIGN_KP_X*error_y : 0.0f;
                vy=fabsf(error_x)>BALL_ALIGN_TOL_X ? BALL_ALIGN_Y_SIGN*BALL_ALIGN_KP_Y*error_x : 0.0f;
                speed=sqrtf(vx*vx+vy*vy);
                if(speed>BALL_ALIGN_MAX_MPS) {
                    vx*=BALL_ALIGN_MAX_MPS/speed;
                    vy*=BALL_ALIGN_MAX_MPS/speed;
                }
                if(!Chassis_SetVelocity(vx,vy,0.0f)) {Route_FailureReason=8;FailAndStop();}
            } else {
                Chassis_Stop();
                if(ball_xy.generation!=align_generation && ball_xy.sequence!=align_sequence) {
                    if(align_frames<BALL_ALIGN_STABLE_FRAMES) ++align_frames;
                }
                if(!AllWheelsStopped()) align_still=0U;
                else if(!align_still) {align_still=1U; align_still_tick=now;}
                if(align_still &&
                   (TickType_t)(now-align_still_tick)>=pdMS_TO_TICKS(BALL_ALIGN_STABLE_MS))
                {
                    Route_BallAligned=1U;Route_BallSlot=locked_ball_slot;Route_BallColor=locked_ball_color;
                    /* One owner, one sequential arm function; wheels remain disabled. */
                    ball_captured=1U;
                    Chassis_Disarm();
                    if(!Arm_Corner5PickDrop(locked_ball_slot)) {Route_FailureReason=9;FailAndStop();return;}
                    /* Append the target task after the unchanged pick/drop sequence. */
                    Chassis_OdomResumeAfterStop();
                    now=xTaskGetTickCount();last_tick=now;
                    target_reference_x=Chassis_OdomGetPose().x_m;
                    ResetSegmentHold();
                    segment_hold_reference=target_reference_x;segment_hold_captured=1;
                    Route_HoldReferenceM=target_reference_x;
                    ball_enabled=ball_pending=0U;balls_mode_pending=balls_wait_ack=0U;
                    GPIO_ResetBits(GPIOC,GPIO_Pin_11);
                    if(!Chassis_Arm()) {Route_FailureReason=10;FailAndStop();return;}
                    phase=ROUTE_TARGET_TRAVEL;
                    progress_x=Chassis_OdomGetWheelPose().x_m;
                    progress_y=Chassis_OdomGetWheelPose().y_m;progress_tick=now;
                    return;
                }
            }
            align_generation=ball_xy.generation; align_sequence=ball_xy.sequence;
        } else if(phase==ROUTE_WAIT_QR) {
            /* Let the QR pose settle; accept the boot-time QR latch. */
            if((TickType_t)(now-wait_tick)<pdMS_TO_TICKS(1000U)) return;
            if(VisionUart_QrEventCount()!=qr_baseline) {
                if(VisionUart_SendStandbyMode()) AdvancePoint(wheel_pose,now);
            } else if(!qr_requested || (TickType_t)(now-qr_request_tick)>=pdMS_TO_TICKS(500U)) {
                if(VisionUart_SendQrMode()) {
                    qr_requested=1U;
                    qr_request_tick=now;
                }
            }
        } else if(phase==ROUTE_WAIT_BALL) {
            if(ball_pending) StartBraking(now,1);
        } else if((TickType_t)(now-wait_tick)>=dwell_ticks) {
            if(after_dwell==AFTER_APPROACH) {
                StartArmDwell(ARM_MISSION_PICK,AFTER_PICK,now);
            } else if(after_dwell==AFTER_PICK) {
                ball_captured=1U; /* Command sequence completed; no claw feedback sensor. */
                StartArmDwell(ARM_MISSION_CARRY,AFTER_CARRY,now);
            } else if(after_dwell==AFTER_CARRY) {
#if ROUTE_FIFTH_CORNER_TEST
                StartArmDwell(ARM_MISSION_BUCKET,AFTER_BUCKET_FIRST,now); /* Drop in place. */
#else
                GoToBucket(wheel_pose,now);
#endif
            } else if(after_dwell==AFTER_BUCKET_FIRST) {
                StartArmDwell(ARM_MISSION_RELEASE,AFTER_BUCKET_RELEASE,now);
            } else if(after_dwell==AFTER_BUCKET_RELEASE) {
#if ROUTE_FIFTH_CORNER_TEST
                StartArmDwell(ARM_MISSION_BUCKET_FINISH,AFTER_TEST_FINISH,now);
#else
                StartArmDwell(ARM_MISSION_BUCKET_FINISH,AFTER_ADVANCE,now);
#endif
            } else if(after_dwell==AFTER_TEST_FINISH) {
                Chassis_Disarm(); route_finished=1U;
            } else AdvancePoint(wheel_pose,now);
        }
        return;
    }
    /* At most one accepted ball request per run; stop before any grasp motion. */
    if(ball_pending && !ball_captured) { StartBraking(now,1); return; }
    dx=wheel_pose.x_m-progress_x; dy=wheel_pose.y_m-progress_y;
    if(dx*dx+dy*dy>0.0016f) {
        progress_x=wheel_pose.x_m; progress_y=wheel_pose.y_m; progress_tick=now;
    } else if((TickType_t)(now-progress_tick)>pdMS_TO_TICKS(ROUTE_STALL_MS)) {
        FailAndStop(); return;
    }
    sx=direct_bucket==1U ? route[next_point].x-bucket_start_x :
       direct_bucket==2U ? 0.0f : route[next_point].x-route[previous_point].x;
    sy=direct_bucket==1U ? 0.0f : direct_bucket==2U ? route[next_point].y-bucket_start_y :
       route[next_point].y-route[previous_point].y;
    if(direct_bucket && fabsf(sx)+fabsf(sy)<=ROUTE_ARRIVAL_M) { StartBraking(now,0); return; }
    if((sx==0.0f)==(sy==0.0f)) { FailAndStop(); return; }
    along_x=(sx!=0.0f);
    direction=(along_x?sx:sy)>0.0f ? 1.0f : -1.0f;
    remaining=direction*((along_x?route[next_point].x:route[next_point].y)-(along_x?wheel_pose.x_m:wheel_pose.y_m));
    if(remaining<=ROUTE_ARRIVAL_M) { StartBraking(now,0); return; }
    route_speed=(next_point>5U) ? ROUTE_MAX_SPEED*0.5f : ROUTE_MAX_SPEED; /* Half speed after corner 5. */
    /* All route legs use the same cross-axis hold, including ball search. */
    trim=SegmentHold(pose,along_x,dt,ROUTE_HOLD_MAX_MPS,ROUTE_HOLD_SLEW_MPS2);
    if(!Chassis_SetVelocity(along_x?direction*route_speed:trim,
                           along_x?trim:direction*route_speed,0.0f)) FailAndStop();
    else if(!departure_reported) {
        departure_reported=1U;
        VOFA_NotifyRouteStarted();
    }
}

void Chassis_RouteTask(void *argument)
{
    TickType_t last=xTaskGetTickCount();
    (void)argument;
    for(;;) {
        vTaskDelayUntil(&last,pdMS_TO_TICKS(10U));
        Chassis_AutoRouteUpdate();
    }
}
