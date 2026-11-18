#ifndef ARM_TASK_H
#define ARM_TASK_H
#include "stm32f4xx.h"
void Arm_task(void *argument);
void ArmMission_TestTask(void *argument);
/* Returns the number of complete vision lines seen; copies the latest line. */
uint32_t ArmTask_GetLastVisionLine(char *out, uint8_t capacity);
typedef enum { ARM_MISSION_CARRY, ARM_MISSION_RELEASE, ARM_MISSION_APPROACH, ARM_MISSION_EXTEND, ARM_MISSION_PICK, ARM_MISSION_BUCKET, ARM_MISSION_BUCKET_FINISH, ARM_MISSION_CENTER_GRASP } ArmMissionAction;
/* Called once by Route_task while stopped; returns settling time in ms. */
void ArmMission_SetBallSlot(uint8_t slot);
uint32_t ArmMission_Apply(ArmMissionAction action);
typedef enum {
    BUCKET_IDLE, BUCKET_PREPARE, BUCKET_GRASP, BUCKET_LIFT,
    BUCKET_SLOT1_TURN, BUCKET_SLOT1_LOWER, BUCKET_SLOT1_RELEASE,
    BUCKET_SLOT2_TURN, BUCKET_SLOT2_LOWER, BUCKET_SLOT2_RELEASE,
    BUCKET_SLOT3_TURN, BUCKET_SLOT3_LOWER, BUCKET_SLOT3_RELEASE,
    BUCKET_DONE, BUCKET_ERROR
} ArmBucketState;
extern volatile ArmBucketState Arm_BucketState;
int ArmBucket_Begin(uint8_t slot, uint32_t now);
int ArmBucket_Update(uint32_t now); /* 0 busy, 1 done, -1 invalid servo command. */
void Arm_Corner5Observe(void);
void Arm_Corner5ObserveStep(uint8_t step);
int Arm_Corner5PickDrop(uint8_t slot);
/* 救援动作：每步返回等待毫秒数，等待由路线任务完成，不在动作内阻塞。 */
uint32_t Arm_RescueObserveStep(uint8_t step);
uint32_t Arm_RescuePickStep(uint8_t step);
#endif
